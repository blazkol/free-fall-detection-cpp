#include <cstdint>
#include <iostream>
#include <memory>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "drivers/i2c_driver.h"

namespace bip = boost::interprocess;
struct imu_sample {
    float ax, ay, az;
    float gx, gy, gz;
};

class free_fall_detector {
public:
    free_fall_detector(const std::string& shm_name, busy_flag &flag) : shm_name(shm_name), bf(flag) {
        // Clean up any previously existing shared memory
        bip::shared_memory_object::remove(shm_name.c_str());

        // Create shared memory segment
        segment = std::make_unique<bip::managed_shared_memory>(bip::create_only, shm_name.c_str(), 65536);

        // Allocate shared memory for I2C buffer
        i2c_mem = segment->construct<std::uint8_t>("i2c_mem")[1](); 

        // Construct synchronization primitives and shared structures
        i2c_mtx = segment->construct<bip::interprocess_mutex>("i2c_mtx")();
        start_cond = segment->construct<bip::interprocess_condition>("start_cond")();
        ack_cond = segment->construct<bip::interprocess_condition>("ack_cond")();
        transmit_cond = segment->construct<bip::interprocess_condition>("transmit_cond")();
        stop_cond = segment->construct<bip::interprocess_condition>("stop_cond")();
    }

    ~free_fall_detector() {
        // Clean up resources
        bip::shared_memory_object::remove(shm_name.c_str());
    }

    std::uint8_t i2c_read_byte(std::uint8_t slave_addr, std::uint8_t reg_addr, int max_retries = 10) {
        // Create scoped lock
        bip::scoped_lock<bip::interprocess_mutex> lock(*i2c_mtx);
        std::uint8_t data;
        
        while(bf != FREE);
        bf = BUSY;

        int retry_count = 0;
        while (retry_count < max_retries) {
            // Send the START signal
            std::cout << "Master (detector): Sending START signal." << std::endl;
            start_cond->notify_one();
            
            // Send address of the slave device with READ
            i2c_send_addr(i2c_mem.get(), slave_addr, READ);

            auto timeout = boost::posix_time::microsec_clock::universal_time() +
                           boost::posix_time::milliseconds(200);

            // Wait for the ACK signal
            bool ack_received = ack_cond->timed_wait(lock, timeout);

            if (ack_received) {
                std::cout << "Master (detector): ACK received after sending address with READ." << std::endl;
                break;
            }

            std::cerr << "Master (detector): Timeout waiting for ACK. Retrying..." << std::endl;
            retry_count++;
        }

        if (retry_count == max_retries) {
            std::cerr << "Master (detector): Failed to connect to device after " << max_retries << " attempts." << std::endl;
            return 1;
        }

        // Send address of the register we want to read from
        i2c_send_frame(i2c_mem.get(), reg_addr);
        transmit_cond->notify_one();
        
        // Wait for the ACK signal
        ack_cond->wait(lock);
        std::cout << "Master (detector): ACK received from slave device after sending register address." << std::endl;

        // Send the ACK signal
        std::cout << "Master (detector): Sending ACK signal." << std::endl;
        ack_cond->notify_one();

        // Wait for the data frame
        transmit_cond->wait(lock);
        data = i2c_receive_frame(i2c_mem.get());
        std::cout << "Master (detector): Data received from slave device." << std::endl;

        // Send the STOP signal
        std::cout << "Master (detector): Sending STOP signal." << std::endl;
        stop_cond->notify_one();

        // Set the busy flag to free
        bf = FREE;

        return data;
    }

    imu_sample read_sample() {
        imu_sample sample = {};

        // Read accelerometer values
        std::uint8_t ax_high = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_DATA_X1);
        std::uint8_t ax_low  = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_DATA_X0);
        std::uint8_t ay_high = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_DATA_Y1);
        std::uint8_t ay_low  = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_DATA_Y0);
        std::uint8_t az_high = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_DATA_Z1);
        std::uint8_t az_low  = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_DATA_Z0);

        // Read gyroscope values
        std::uint8_t gx_high = i2c_read_byte(ICM_42670_P_ADDR, GYRO_DATA_X1);
        std::uint8_t gx_low  = i2c_read_byte(ICM_42670_P_ADDR, GYRO_DATA_X0);
        std::uint8_t gy_high = i2c_read_byte(ICM_42670_P_ADDR, GYRO_DATA_Y1);
        std::uint8_t gy_low  = i2c_read_byte(ICM_42670_P_ADDR, GYRO_DATA_Y0);
        std::uint8_t gz_high = i2c_read_byte(ICM_42670_P_ADDR, GYRO_DATA_Z1);
        std::uint8_t gz_low  = i2c_read_byte(ICM_42670_P_ADDR, GYRO_DATA_Z0);

        // Combine high and low bytes into 16-bit signed values
        sample.ax = static_cast<int16_t>((ax_high << 8) | ax_low);
        sample.ay = static_cast<int16_t>((ay_high << 8) | ay_low);
        sample.az = static_cast<int16_t>((az_high << 8) | az_low);

        sample.gx = static_cast<int16_t>((gx_high << 8) | gx_low);
        sample.gy = static_cast<int16_t>((gy_high << 8) | gy_low);
        sample.gz = static_cast<int16_t>((gz_high << 8) | gz_low);

        return sample;
    }

private:
    std::string shm_name;
    std::unique_ptr<bip::managed_shared_memory> segment;

    bip::offset_ptr<std::uint8_t> i2c_mem;

    bip::interprocess_mutex* i2c_mtx;
    bip::interprocess_condition* start_cond;
    bip::interprocess_condition* ack_cond;
    bip::interprocess_condition* transmit_cond;
    bip::interprocess_condition* stop_cond;

    busy_flag bf;
};

int main() {
    std::cout << "Free fall detector started." << std::endl;

    // Create the busy flag
    busy_flag bf = FREE;

    // Create the detector
    free_fall_detector detector("i2c_shm", bf);

    // Read IMU sample
    detector.read_sample();
    
    std::cout << "Free fall detector finished." << std::endl;

    return 0;
}