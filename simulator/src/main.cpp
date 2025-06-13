#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/program_options.hpp>

#include "drivers/i2c_driver.h"
#include "drivers/imu_driver.h"

namespace bip = boost::interprocess;
namespace blog = boost::log;
namespace bpo = boost::program_options;

void init_logging() {
    blog::add_console_log(std::clog, blog::keywords::format = "[%TimeStamp%] <%Severity%>: %Message%");
    blog::add_common_attributes();
}

void parse_headers(std::ifstream &csv_file, std::vector<std::string> &headers) {
    std::string line, col;

    std::getline(csv_file, line);
    std::stringstream ss(line);

    while (std::getline(ss, col, ',')) {
        boost::algorithm::trim(col);
        headers.push_back(col);
    }
}

bool skip_rows(std::ifstream &csv_file, int start_row = 0) {
    std::string skip_line;

    while (start_row-- > 0) {
        if (!std::getline(csv_file, skip_line)) {
            BOOST_LOG_TRIVIAL(error) << "CSV file does not contain enough rows to skip to start_row="
                                     << (start_row + 1);
            return false;
        }
    }

    return true;
}

bool load_next_sample(std::ifstream &csv_file, std::vector<std::string> &headers, imu_sample &sample) {
    std::vector<double> data_row;
    std::string line, cell;

    // Return false if there are no more logs in the CSV file
    if (!std::getline(csv_file, line)) {
        BOOST_LOG_TRIVIAL(warning) << "No subsequent IMU reading.";
        return false;
    }

    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) {
        boost::algorithm::trim(cell);
        data_row.push_back(std::stod(cell));
    }

    auto get_column = [&](const std::string &col_name) -> double {
        for (std::size_t i = 0; i < headers.size(); ++i) {
            if (headers[i] == col_name) {
                return data_row[i];
            }
        }
        return 0.0;
    };

    sample.ax = get_column("ax");
    sample.ay = get_column("ay");
    sample.az = get_column("az");
    sample.gx = get_column("gx");
    sample.gy = get_column("gy");
    sample.gz = get_column("gz");

    return true;
}

void configure_imu(float &accel_range, float &gyro_range, float &sampling_rate, std::uint8_t imu_bank_0[]) {
    uint8_t accel_fss, accel_odr, gyro_fss, gyro_odr;

    switch (static_cast<int>(accel_range)) {
        case 16: accel_fss = 0x00; break;
        case 8:  accel_fss = 0x01; break;
        case 4:  accel_fss = 0x02; break;
        case 2:  accel_fss = 0x03; break;
        default: accel_fss = 0x00; break;
    }

    switch (static_cast<int>(gyro_range)) {
        case 2000: gyro_fss = 0x00; break;
        case 1000: gyro_fss = 0x01; break;
        case 500:  gyro_fss = 0x02; break;
        case 250:  gyro_fss = 0x03; break;
        default:   gyro_fss = 0x00; break;
    }

    if (sampling_rate >= 1600.0f) {
        accel_odr = 0x05; gyro_odr = 0x05;
    } else if (sampling_rate >= 800.0f) {
        accel_odr = 0x06; gyro_odr = 0x06;
    } else if (sampling_rate >= 400.0f) {
        accel_odr = 0x07; gyro_odr = 0x07;
    } else if (sampling_rate >= 200.0f) {
        accel_odr = 0x08; gyro_odr = 0x08;
    } else if (sampling_rate >= 100.0f) {
        accel_odr = 0x09; gyro_odr = 0x09;
    } else if (sampling_rate >= 50.0f) {
        accel_odr = 0x0A; gyro_odr = 0x0A;
    } else if (sampling_rate >= 25.0f) {
        accel_odr = 0x0B; gyro_odr = 0x0B;
    } else if (sampling_rate >= 12.5f) {
        accel_odr = 0x0C; gyro_odr = 0x0C;
    } else if (sampling_rate >= 6.25f) {
        accel_odr = 0x0D;
    } else if (sampling_rate >= 3.125f) {
        accel_odr = 0x0E;
    } else if (sampling_rate >= 1.5625f) {
        accel_odr = 0x0F;
    } else {
        accel_odr = 0x0A; gyro_odr = 0x0A;  // 50 Hz
    }

    imu_bank_0[ACCEL_CONFIG0] = static_cast<std::uint8_t>((accel_fss << 5) | (accel_odr & 0x0F));
    imu_bank_0[GYRO_CONFIG0]  = static_cast<std::uint8_t>((gyro_fss << 5)  | (gyro_odr & 0x0F));
}

void update_imu_bank_0(float &accel_range, float &gyro_range, imu_sample &sample, std::uint8_t imu_bank_0[]) {
    double accel_scale;
    double gyro_scale;

    switch (static_cast<int>(accel_range)) {
        case 16: accel_scale = 2048.0; break;
        case 8:  accel_scale = 4096.0; break;
        case 4:  accel_scale = 8192.0; break;
        case 2:  accel_scale = 16384.0; break;
    }

    switch (static_cast<int>(gyro_range)) {
        case 2000: gyro_scale = 16.4; break;
        case 1000: gyro_scale = 32.8; break;
        case 500:  gyro_scale = 65.5; break;
        case 250:  gyro_scale = 131.0; break;
    }

    std::int16_t ax_raw = static_cast<std::int16_t>(sample.ax * accel_scale);
    std::int16_t ay_raw = static_cast<std::int16_t>(sample.ay * accel_scale);
    std::int16_t az_raw = static_cast<std::int16_t>(sample.az * accel_scale);
    std::int16_t gx_raw = static_cast<std::int16_t>(sample.gx * gyro_scale);
    std::int16_t gy_raw = static_cast<std::int16_t>(sample.gy * gyro_scale);
    std::int16_t gz_raw = static_cast<std::int16_t>(sample.gz * gyro_scale);

    imu_bank_0[ACCEL_DATA_X1] = static_cast<std::uint8_t>((ax_raw >> 8) & 0xFF);
    imu_bank_0[ACCEL_DATA_X0] = static_cast<std::uint8_t>(ax_raw & 0xFF);
    imu_bank_0[ACCEL_DATA_Y1] = static_cast<std::uint8_t>((ay_raw >> 8) & 0xFF);
    imu_bank_0[ACCEL_DATA_Y0] = static_cast<std::uint8_t>(ay_raw & 0xFF);
    imu_bank_0[ACCEL_DATA_Z1] = static_cast<std::uint8_t>((az_raw >> 8) & 0xFF);
    imu_bank_0[ACCEL_DATA_Z0] = static_cast<std::uint8_t>(az_raw & 0xFF);

    imu_bank_0[GYRO_DATA_X1] = static_cast<std::uint8_t>((gx_raw >> 8) & 0xFF);
    imu_bank_0[GYRO_DATA_X0] = static_cast<std::uint8_t>(gx_raw & 0xFF);
    imu_bank_0[GYRO_DATA_Y1] = static_cast<std::uint8_t>((gy_raw >> 8) & 0xFF);
    imu_bank_0[GYRO_DATA_Y0] = static_cast<std::uint8_t>(gy_raw & 0xFF);
    imu_bank_0[GYRO_DATA_Z1] = static_cast<std::uint8_t>((gz_raw >> 8) & 0xFF);
    imu_bank_0[GYRO_DATA_Z0] = static_cast<std::uint8_t>(gz_raw & 0xFF);
}
   
int main() {
    init_logging();
    BOOST_LOG_TRIVIAL(info) << "IMU simulator started.";

    float accel_range, gyro_range, sampling_rate;
    int start_row;

    // Create the configuration
    bpo::options_description config("configuration");
    config.add_options()
        ("sampling_rate", bpo::value<float>(&sampling_rate)->default_value(50))
        ("accel_range", bpo::value<float>(&accel_range)->default_value(2.0f))
        ("gyro_range", bpo::value<float>(&gyro_range)->default_value(250.0f))
        ("start_row", bpo::value<int>(&start_row)->default_value(0))
    ;

    bpo::variables_map vm;

    // Open the config file
    std::ifstream config_file("./resources/config.ini");
    if (!config_file) {
        BOOST_LOG_TRIVIAL(warning) << "Warning: config.ini not found, using defaults.";
    } else {
        bpo::store(bpo::parse_config_file(config_file, config, true), vm);
        bpo::notify(vm);
    }
    config_file.close(); 

    std::uint8_t imu_bank_0[0x2f] = {0};

    // Set GYRO_CONFIG0 and ACCEL_CONFIG0 registers
    configure_imu(accel_range, gyro_range, sampling_rate, imu_bank_0);

    // Open the CSV file
    std::string filename = "./resources/2023-01-16-15-33-09-imu.csv";
    std::ifstream csv_file(filename);
    if (!csv_file.is_open()) {
        BOOST_LOG_TRIVIAL(error) << "Failed to open CSV file: " << filename;
        return 1;
    }

    // Parse headers
    std::vector<std::string> headers;
    parse_headers(csv_file, headers);

    // Skip to start_row
    if (!skip_rows(csv_file, start_row)) {
        return 1;
    }

    // Open shared memory
    bip::managed_shared_memory segment(bip::open_only, "i2c_shm");

    bip::offset_ptr<std::uint8_t> i2c_mem = segment.find<uint8_t>("i2c_mem").first;

    // Find synchronization primitives and shared structures
    bip::interprocess_mutex* i2c_mtx = segment.find<bip::interprocess_mutex>("i2c_mtx").first;
    bip::interprocess_condition* start_cond = segment.find<bip::interprocess_condition>("start_cond").first;
    bip::interprocess_condition* ack_cond = segment.find<bip::interprocess_condition>("ack_cond").first;
    bip::interprocess_condition* transmit_cond = segment.find<bip::interprocess_condition>("transmit_cond").first;
    bip::interprocess_condition* stop_cond = segment.find<bip::interprocess_condition>("stop_cond").first;

    // Create scoped lock
    bip::scoped_lock<bip::interprocess_mutex> lock(*i2c_mtx);

    std::uint8_t device_addr = static_cast<std::uint8_t>(ICM_42670_P_ADDR);
    std::uint8_t reg_addr = 0x00u;
    std::uint8_t data, mode;

    imu_sample sample = {};

    while(true) {
        // Wait for the START signal
        start_cond->wait(lock);

        // Scan slave address
        if (i2c_scan_addr(i2c_mem.get(), device_addr)) {
            mode = i2c_check_rw(i2c_mem.get());

            // Send ACK signal
            ack_cond->notify_one();
            transmit_cond->wait(lock);

            // Receive register address
            reg_addr = i2c_receive_frame(i2c_mem.get());
            ack_cond->notify_one();

            switch(mode) {
                case WRITE:
                    // NOT IMPLEMENTED
                    break;

                case READ:
                    // Wait for ACK signal
                    ack_cond->wait(lock);

                    // Load IMU sample from CSV
                    if (!load_next_sample(csv_file, headers, sample)) {
                        BOOST_LOG_TRIVIAL(info) << "End of CSV file reached.";
                        break;
                    }

                    // Update IMU registers
                    update_imu_bank_0(accel_range, gyro_range, sample, imu_bank_0);

                    // Send data frame
                    data = imu_bank_0[reg_addr];
                    i2c_send_frame(i2c_mem.get(), data);
                    transmit_cond->notify_one();

                    stop_cond->wait(lock);
                    break;
            }
        }
    }

    csv_file.close();

    BOOST_LOG_TRIVIAL(info) << "IMU simulator finished.";

    return 0;
}