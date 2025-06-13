#include <cstdint>
#include <iostream>
#include <memory>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/program_options.hpp>

#include "drivers/i2c_driver.h"
#include "drivers/imu_driver.h"

namespace bip = boost::interprocess;
namespace blog = boost::log;

void init_logging() {
    blog::add_console_log(std::clog, blog::keywords::format = "[%TimeStamp%] <%Severity%>: %Message%");
    blog::add_common_attributes();
}

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
            start_cond->notify_one();
            
            // Send address of the slave device with READ
            i2c_send_addr(i2c_mem.get(), slave_addr, READ);

            auto timeout = boost::posix_time::microsec_clock::universal_time() +
                           boost::posix_time::milliseconds(200);

            // Wait for the ACK signal
            bool ack_received = ack_cond->timed_wait(lock, timeout);

            if (ack_received) {
                break;
            }

            retry_count++;
        }

        if (retry_count == max_retries) {
            BOOST_LOG_TRIVIAL(error) << "Master (detector): Failed to connect to device after " << max_retries << " attempts.";
            return 1;
        }

        // Send address of the register we want to read from
        i2c_send_frame(i2c_mem.get(), reg_addr);
        transmit_cond->notify_one();
        
        // Wait for the ACK signal
        ack_cond->wait(lock);

        // Send the ACK signal
        ack_cond->notify_one();

        // Wait for the data frame
        transmit_cond->wait(lock);
        data = i2c_receive_frame(i2c_mem.get());

        // Send the STOP signal
        stop_cond->notify_one();

        // Set the busy flag to free
        bf = FREE;

        return data;
    }

    float get_gyro_scale_factor() {
        std::uint8_t gyro_config0 = i2c_read_byte(ICM_42670_P_ADDR, GYRO_CONFIG0);
        std::uint8_t gyro_fss = (gyro_config0 & 0x60) >> 5;
        switch (gyro_fss) {
            case 0x00: return 16.4f;   // ±2000 dps
            case 0x01: return 32.8f;   // ±1000 dps
            case 0x02: return 65.5f;   // ±500 dps
            case 0x03: return 131.0f;  // ±250 dps
        }
    }

    float get_accel_scale_factor() {
        std::uint8_t accel_config0 = i2c_read_byte(ICM_42670_P_ADDR, ACCEL_CONFIG0);
        std::uint8_t accel_fss = (accel_config0 & 0x60) >> 5;
        switch (accel_fss) {
            case 0x00: return 2048.0f;   // ±16g
            case 0x01: return 4096.0f;   // ±8g
            case 0x02: return 8192.0f;   // ±4g
            case 0x03: return 16384.0f;  // ±2g
        }
    }

    imu_sample read_sample() {
        imu_sample sample = {};

        float accel_scale = get_accel_scale_factor();
        float gyro_scale = get_gyro_scale_factor();

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

        // Combine high/low bytes
        std::int16_t ax_raw = static_cast<std::int16_t>((ax_high << 8) | ax_low);
        std::int16_t ay_raw = static_cast<std::int16_t>((ay_high << 8) | ay_low);
        std::int16_t az_raw = static_cast<std::int16_t>((az_high << 8) | az_low);
        std::int16_t gx_raw = static_cast<std::int16_t>((gx_high << 8) | gx_low);
        std::int16_t gy_raw = static_cast<std::int16_t>((gy_high << 8) | gy_low);
        std::int16_t gz_raw = static_cast<std::int16_t>((gz_high << 8) | gz_low);

        // Convert raw values to physical units
        sample.ax = static_cast<double>(ax_raw) / accel_scale; 
        sample.ay = static_cast<double>(ay_raw) / accel_scale;
        sample.az = static_cast<double>(az_raw) / accel_scale;
        sample.gx = static_cast<double>(gx_raw) / gyro_scale;
        sample.gy = static_cast<double>(gy_raw) / gyro_scale;
        sample.gz = static_cast<double>(gz_raw) / gyro_scale;

        BOOST_LOG_TRIVIAL(info) << "IMU Sample:";
        BOOST_LOG_TRIVIAL(info) << "  Accel: ax=" << sample.ax << ", ay=" << sample.ay << ", az=" << sample.az;
        BOOST_LOG_TRIVIAL(info) << "  Gyro : gx=" << sample.gx << ", gy=" << sample.gy << ", gz=" << sample.gz;

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
    init_logging();
    BOOST_LOG_TRIVIAL(info) << "Free fall detector started.";

    // Create the busy flag
    busy_flag bf = FREE;

    // Create the detector
    free_fall_detector detector("i2c_shm", bf);

    // Read IMU sample
    detector.read_sample();
    
    BOOST_LOG_TRIVIAL(info) << "Free fall detector finished.";

    return 0;
}