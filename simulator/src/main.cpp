#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
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

std::string read_data(std::ifstream &file, std::vector<std::string> &headers, std::streampos &data_start_pos, const std::string &column_name) {
    // Find column index
    int column_index = -1;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == column_name) {
            column_index = static_cast<int>(i);
            break;
        }
    }

    if (column_index == -1) {
        std::cerr << "Column not found: " << column_name << std::endl;
        exit(1);
    }

    // Read next data line
    std::vector<std::string> data_row;
    std::string line, cell;

    if (!std::getline(file, line)) {
        file.clear(); // Clear EOF flag
        file.seekg(data_start_pos);
        std::getline(file, line);
    }
    std::stringstream ss(line);
    
    while (std::getline(ss, cell, ',')) {
        data_row.push_back(cell);
    }

    return data_row[column_index];
}
   

int main() {
    std::cout << "IMU simulator started." << std::endl;

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
    bip::shared_memory_object i2c_shm_obj(bip::open_only, "i2c_shm", bip::read_write);
    bip::mapped_region i2c_region(i2c_shm_obj, bip::read_write);
    std::uint8_t *i2c_mem = static_cast<std::uint8_t*>(i2c_region.get_address());

    // Open i2c mutex and start condition
    bip::named_mutex i2c_mtx(bip::open_only, "i2c_mtx");

    // Open named conditions
    bip::named_condition start_cond(bip::open_only, "start_cond");
    bip::named_condition ack_cond(bip::open_only, "ack_cond");
    bip::named_condition transmit_cond(bip::open_only, "transmit_cond");
    bip::named_condition stop_cond(bip::open_only, "stop_cond");

    bip::scoped_lock<bip::named_mutex> lock(i2c_mtx);

    std::uint8_t device_addr = static_cast<std::uint8_t>(ICM_42670_P_ADDR);
    std::uint8_t reg_addr = 0x00u;
    std::uint8_t data = 0x01u;
    std::uint8_t mode;

    while(true) {
        // Wait for the START signal
        std::cout << "Slave (simulator): Waiting for START signal..." << std::endl;
        start_cond.wait(lock);
        std::cout << "Slave (simulator): START signal received." << std::endl;

        // Scan slave address
        if (i2c_scan_addr(i2c_mem, device_addr)) {
            mode = i2c_check_rw(i2c_mem);

            std::cout << "Slave (simulator): Sending ACK signal." << std::endl;
            ack_cond.notify_one();
            transmit_cond.wait(lock);

            // Receive register address
            reg_addr = i2c_receive_frame(i2c_mem);
            
            std::cout << "Slave (simulator): Sending ACK signal." << std::endl;
            ack_cond.notify_one();

            switch(mode) {
                case WRITE:
                    std::cout << "Slave (simulator): WRITE mode." << std::endl;
                    // NOT IMPLEMENTED
                    break;

                case READ:
                    std::cout << "Slave (simulator): READ mode." << std::endl;

                    // Wait for the ACK signal
                    ack_cond.wait(lock);

                    // Send data frame
                    i2c_send_frame(i2c_mem, data);
                    transmit_cond.notify_one();

                    stop_cond.wait(lock);
                    std::cout << "Slave (simulator): STOP signal received." << std::endl;
                    break;
            }
        }
    }
    
    // Read data from CSV
    read_data(file, headers, data_start_pos, "ax");

    file.close();

    std::cout << "IMU simulator finished." << std::endl;

    return 0;
}