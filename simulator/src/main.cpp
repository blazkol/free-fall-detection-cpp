#include <cstddef>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "drivers/i2c_driver.h"
#include "drivers/imu_driver.h"

namespace bpo = boost::program_options;
namespace bip = boost::interprocess;

std::streampos parse_headers(std::ifstream &file, std::vector<std::string> &headers) {
    std::string line, col;

    std::getline(file, line);
    std::stringstream ss(line);

    while (std::getline(ss, col, ',')) {
        boost::algorithm::trim(col);
        headers.push_back(col);
    }

    return file.tellg();
}

void load_next_sample(std::ifstream &file,
                      std::vector<std::string> &headers, 
                      std::streampos &data_start_pos, 
                      imu_sample &sample) 
{
    std::vector<double> data_row;
    std::string line, cell;

    // Rewind to the beginning of the CSV file if its end has been reached
    if (!std::getline(file, line)) {
        file.clear(); // Clear EOF flag
        file.seekg(data_start_pos);
        std::getline(file, line);
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
}

void configure_imu(float &accel_range, float &gyro_range, std::uint8_t imu_registers[]) {
    uint8_t accel_fss;
    uint8_t gyro_fss;

    switch (static_cast<int>(accel_range)) {
        case 16: accel_fss = 0x00; break;
        case 8:  accel_fss = 0x01; break;
        case 4:  accel_fss = 0x02; break;
        case 2:  accel_fss = 0x03; break;
    }

    switch (static_cast<int>(gyro_range)) {
        case 2000: gyro_fss = 0x00; break;
        case 1000: gyro_fss = 0x01; break;
        case 500:  gyro_fss = 0x02; break;
        case 250:  gyro_fss = 0x03; break;
    }

    imu_registers[ACCEL_CONFIG0] = static_cast<std::uint8_t>(accel_fss << 5);
    imu_registers[GYRO_CONFIG0]  = static_cast<std::uint8_t>(gyro_fss << 5);
}

void update_imu_registers(float &accel_range, float &gyro_range, imu_sample &sample, std::uint8_t imu_registers[]) {
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

    imu_registers[ACCEL_DATA_X1] = static_cast<std::uint8_t>((ax_raw >> 8) & 0xFF);
    imu_registers[ACCEL_DATA_X0] = static_cast<std::uint8_t>(ax_raw & 0xFF);
    imu_registers[ACCEL_DATA_Y1] = static_cast<std::uint8_t>((ay_raw >> 8) & 0xFF);
    imu_registers[ACCEL_DATA_Y0] = static_cast<std::uint8_t>(ay_raw & 0xFF);
    imu_registers[ACCEL_DATA_Z1] = static_cast<std::uint8_t>((az_raw >> 8) & 0xFF);
    imu_registers[ACCEL_DATA_Z0] = static_cast<std::uint8_t>(az_raw & 0xFF);

    imu_registers[GYRO_DATA_X1] = static_cast<std::uint8_t>((gx_raw >> 8) & 0xFF);
    imu_registers[GYRO_DATA_X0] = static_cast<std::uint8_t>(gx_raw & 0xFF);
    imu_registers[GYRO_DATA_Y1] = static_cast<std::uint8_t>((gy_raw >> 8) & 0xFF);
    imu_registers[GYRO_DATA_Y0] = static_cast<std::uint8_t>(gy_raw & 0xFF);
    imu_registers[GYRO_DATA_Z1] = static_cast<std::uint8_t>((gz_raw >> 8) & 0xFF);
    imu_registers[GYRO_DATA_Z0] = static_cast<std::uint8_t>(gz_raw & 0xFF);
}
   
int main() {
    std::cout << "IMU simulator started." << std::endl;

    float accel_range, gyro_range;
    int sample_rate;

    // Create the configuration
    bpo::options_description config("configuration");
    config.add_options()
        ("sample_rate", bpo::value<int>(&sample_rate)->default_value(50))
        ("accel_range", bpo::value<float>(&accel_range)->default_value(2.0f))
        ("gyro_range", bpo::value<float>(&gyro_range)->default_value(250.0f))
    ;

    bpo::variables_map vm;

    // Open the config file
    std::ifstream config_file("./resources/config.ini");
    if (!config_file) {
        std::cerr << "Warning: config.ini not found, using defaults.\n";
    } else {
        bpo::store(bpo::parse_config_file(config_file, config, true), vm);
        bpo::notify(vm);
    }

    std::uint8_t imu_registers[34] = {0};

    // Set GYRO_CONFIG0 and ACCEL_CONFIG0 registers
    configure_imu(accel_range, gyro_range, imu_registers);

    // Open the CSV file
    std::string filename = "./resources/2023-01-16-15-33-09-imu.csv";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << filename << std::endl;
        return 1;
    }

    // Parse headers and record the position
    std::vector<std::string> headers;
    std::streampos data_start_pos = parse_headers(file, headers);

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
        // std::cout << "Slave (simulator): Waiting for START signal..." << std::endl;
        start_cond->wait(lock);
        // std::cout << "Slave (simulator): START signal received." << std::endl;

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
                    load_next_sample(file, headers, data_start_pos, sample);

                    // Update IMU registers
                    update_imu_registers(accel_range, gyro_range, sample, imu_registers);

                    // Send data frame
                    data = imu_registers[reg_addr];
                    i2c_send_frame(i2c_mem.get(), data);
                    transmit_cond->notify_one();

                    stop_cond->wait(lock);
                    // std::cout << "Slave (simulator): STOP signal received." << std::endl;
                    break;
            }
        }
    }

    file.close();

    std::cout << "IMU simulator finished." << std::endl;

    return 0;
}