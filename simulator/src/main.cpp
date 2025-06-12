#include <cstddef>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "drivers/i2c_driver.h"

namespace bip = boost::interprocess;

struct imu_sample {
    float ax, ay, az;
    float gx, gy, gz;
};

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
                      std::uint8_t imu_registers[]) 
{
    std::vector<float> data_row;
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
        data_row.push_back(std::stof(cell));
    }

    auto get_column = [&](const std::string &col_name) -> int16_t {
        for (std::size_t i = 0; i < headers.size(); ++i) {
            if (headers[i] == col_name) {
                return static_cast<int16_t>(data_row[i]);
            }
        }
        return 0.0f;
    };

    int16_t ax = get_column("ax");
    int16_t ay = get_column("ay");
    int16_t az = get_column("az");
    int16_t gx = get_column("gx");
    int16_t gy = get_column("gy");
    int16_t gz = get_column("gz");

    imu_registers[ACCEL_DATA_X1] = static_cast<uint8_t>((ax >> 8) & 0xFF);
    imu_registers[ACCEL_DATA_X0] = static_cast<uint8_t>(ax & 0xFF);
    imu_registers[ACCEL_DATA_Y1] = static_cast<uint8_t>((ay >> 8) & 0xFF);
    imu_registers[ACCEL_DATA_Y0] = static_cast<uint8_t>(ay & 0xFF);
    imu_registers[ACCEL_DATA_Z1] = static_cast<uint8_t>((az >> 8) & 0xFF);
    imu_registers[ACCEL_DATA_Z0] = static_cast<uint8_t>(az & 0xFF);

    imu_registers[GYRO_DATA_X1] = static_cast<uint8_t>((gx >> 8) & 0xFF);
    imu_registers[GYRO_DATA_X0] = static_cast<uint8_t>(gx & 0xFF);
    imu_registers[GYRO_DATA_Y1] = static_cast<uint8_t>((gy >> 8) & 0xFF);
    imu_registers[GYRO_DATA_Y0] = static_cast<uint8_t>(gy & 0xFF);
    imu_registers[GYRO_DATA_Z1] = static_cast<uint8_t>((gz >> 8) & 0xFF);
    imu_registers[GYRO_DATA_Z0] = static_cast<uint8_t>(gz & 0xFF);

}
   
int main() {
    std::cout << "IMU simulator started." << std::endl;

    std::uint8_t imu_registers[0x20] = {0};

    // Open the CSV file
    std::string filename = "./data/2023-01-16-15-33-09-imu.csv";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
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
    std::uint8_t data;
    std::uint8_t mode;

    while(true) {
        // Wait for the START signal
        std::cout << "Slave (simulator): Waiting for START signal..." << std::endl;
        start_cond->wait(lock);
        std::cout << "Slave (simulator): START signal received." << std::endl;

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
                    load_next_sample(file, headers, data_start_pos, imu_registers);

                    // Send data frame
                    data = imu_registers[reg_addr];
                    i2c_send_frame(i2c_mem.get(), data);
                    transmit_cond->notify_one();

                    stop_cond->wait(lock);
                    std::cout << "Slave (simulator): STOP signal received." << std::endl;
                    break;
            }
        }
    }

    file.close();

    std::cout << "IMU simulator finished." << std::endl;

    return 0;
}