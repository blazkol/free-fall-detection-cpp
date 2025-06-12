#include <iostream>
#include <chrono>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "drivers/i2c_driver.h"

namespace bip = boost::interprocess;

struct imu_sample {
    float ax, ay, az;
    float gx, gy, gz;
};

int main() {
    std::cout << "Free fall detector started." << std::endl;

    // Ensure clean slate
    bip::shared_memory_object::remove("i2c_shm");
    bip::named_mutex::remove("i2c_mtx");
    bip::named_condition::remove("start_cond");
    bip::named_condition::remove("stop_cond");
    bip::named_condition::remove("ack_cond");
    bip::named_condition::remove("transmit_cond");

    // Create shared memory
    bip::shared_memory_object i2c_shm_obj(bip::create_only, "i2c_shm", bip::read_write);
    i2c_shm_obj.truncate(sizeof(std::uint8_t));
    bip::mapped_region i2c_region(i2c_shm_obj, bip::read_write);
    std::uint8_t *i2c_mem = static_cast<std::uint8_t*>(i2c_region.get_address());

    // Create i2c mutex
    bip::named_mutex i2c_mtx(bip::create_only, "i2c_mtx");

    // Create named conditions
    bip::named_condition start_cond(bip::create_only, "start_cond");
    bip::named_condition ack_cond(bip::create_only, "ack_cond");
    bip::named_condition transmit_cond(bip::create_only, "transmit_cond");
    bip::named_condition stop_cond(bip::create_only, "stop_cond");

    bip::scoped_lock<bip::named_mutex> lock(i2c_mtx);

    std::uint8_t slave_addr = static_cast<std::uint8_t>(ICM_42670_P_ADDR);
    std::uint8_t reg_addr = static_cast<std::uint8_t>(REG_ACCEL_DATA_X1);
    std::uint8_t data = 0x00u;

    // Create the busy flag
    busy_flag bf = BUSY;

    int max_retries = 10;
    int retry_count = 0;

    while (retry_count < max_retries) {
        // Send the START signal
        std::cout << "Master (detector): Sending START signal." << std::endl;
        start_cond.notify_one();
        
        // Send address of the slave device with READ
        i2c_send_addr(i2c_mem, slave_addr, READ);

        auto timeout = boost::posix_time::microsec_clock::universal_time() +
                       boost::posix_time::milliseconds(200);

        // Wait for the ACK signal
        bool ack_received = ack_cond.timed_wait(lock, timeout);

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
    i2c_send_frame(i2c_mem, reg_addr);
    transmit_cond.notify_one();
    
    // Wait for the ACK signal
    ack_cond.wait(lock);
    std::cout << "Master (detector): ACK received from slave device after sending register address." << std::endl;

    // Send the ACK signal
    std::cout << "Master (detector): Sending ACK signal." << std::endl;
    ack_cond.notify_one();

    // Wait for the data frame
    transmit_cond.wait(lock);
    data = i2c_receive_frame(i2c_mem);
    std::cout << "Master (detector): Data received from slave device." << std::endl;

    // Send the STOP signal
    std::cout << "Master (detector): Sending STOP signal." << std::endl;
    stop_cond.notify_one();

    // Set the busy flag to free
    bf = FREE;
    
    bip::shared_memory_object::remove("i2c_shm");
    bip::named_mutex::remove("i2c_mtx");
    bip::named_condition::remove("start_cond");
    bip::named_condition::remove("ack_cond");
    bip::named_condition::remove("transmit_cond");
    bip::named_condition::remove("stop_cond");
    
    std::cout << "Free fall detector finished." << std::endl;

    return 0;
}