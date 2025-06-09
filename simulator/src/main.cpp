#include <iostream>
#include <fstream>
#include <sstream>
#include <cstddef>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#define REG_ACCEL_DATA_X1   0x0B    // Upper byte of Accel X-axis data
#define REG_ACCEL_DATA_X0   0x0C    // Lower byte of Accel X-axis data
#define REG_ACCEL_DATA_Y1   0x0D    // Upper byte of Accel Y-axis data
#define REG_ACCEL_DATA_Y0   0x0E    // Lower byte of Accel Y-axis data
#define REG_ACCEL_DATA_Z1   0x0F    // Upper byte of Accel Z-axis data
#define REG_ACCEL_DATA_Z0   0x10    // Lower byte of Accel Z-axis data

#define REG_GYRO_DATA_X1    0x11    // Upper byte of Gyro X-axis data
#define REG_GYRO_DATA_X0    0x12    // Lower byte of Gyro X-axis data
#define REG_GYRO_DATA_Y1    0x13    // Upper byte of Gyro Y-axis data
#define REG_GYRO_DATA_Y0    0x14    // Lower byte of Gyro Y-axis data
#define REG_GYRO_DATA_Z1    0x15    // Upper byte of Gyro Z-axis data
#define REG_GYRO_DATA_Z0    0x16    // Lower byte of Gyro Z-axis data

namespace balg = boost::algorithm;
namespace bip = boost::interprocess;

struct imu_sample {
    float ax, ay, az;
    float gx, gy, gz;
};

std::string read_data(std::ifstream &file, std::vector<std::string> &headers, std::streampos &data_start_pos, const std::string &column_name) {
    // Find column index
    int column_index = -1;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == column_name) {
            column_index = static_cast<int>(i);
            break;
        }
    }
    // if (column_index == -1) {
    //     std::cerr << "Column not found: " << column_name << std::endl;
    //     return;
    // }

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

    // Ensure clean slate
    bip::shared_memory_object::remove("imu_shm");
    bip::named_mutex::remove("imu_mtx");

    // Create shared memory
    bip::shared_memory_object imu_shm_obj(bip::create_only, "imu_shm", bip::read_write);
    imu_shm_obj.truncate(sizeof(imu_sample));
    bip::mapped_region imu_region(imu_shm_obj, bip::read_write);
    void* imu_addr = imu_region.get_address();
    std::memset(imu_addr, 0, sizeof(imu_sample));

    // Open the CSV file
    std::string filename = "./data/2023-01-16-15-33-09-imu.csv";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1;
    }
    
    // Parse headers
    std::vector<std::string> headers;
    std::string line, col;

    std::getline(file, line);
    std::stringstream ss(line);

    while (std::getline(ss, col, ',')) {
        balg::trim(col);
        headers.push_back(col);
    }

    std::streampos data_start_pos = file.tellg();

    imu_sample sample;
    sample.ax = std::stof(read_data(file, headers, data_start_pos, "ax"));
    sample.ay = std::stof(read_data(file, headers, data_start_pos, "ay"));
    sample.az = std::stof(read_data(file, headers, data_start_pos, "az"));
    sample.gx = std::stof(read_data(file, headers, data_start_pos, "gx"));
    sample.gy = std::stof(read_data(file, headers, data_start_pos, "gy"));
    sample.gz = std::stof(read_data(file, headers, data_start_pos, "gz"));

    file.close();
    
    // Create named mutex
    bip::named_mutex imu_mtx(bip::create_only, "imu_mtx");

    {
        bip::scoped_lock<bip::named_mutex> lock(imu_mtx);
        std::memcpy(imu_addr, &sample, sizeof(imu_sample));
        std::cout << "Simulator wrote IMU data.\n";
    }

    std::cout << "IMU simulator finished." << std::endl;

    return 0;
}