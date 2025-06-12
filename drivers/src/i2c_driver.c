#include "drivers/i2c_driver.h"

void i2c_init(void) {
    // NOT IMPLEMENTED
}

void i2c_send_frame(uint8_t *i2c_addr, uint8_t frame) {
    *i2c_addr = frame;
}

void i2c_send_addr(uint8_t *i2c_addr, uint8_t slave_addr, rw_bit rw) {
    i2c_send_frame(i2c_addr, (slave_addr << 1) | rw);
}

uint8_t i2c_receive_frame(uint8_t *i2c_addr) {
    return *i2c_addr;
}

bool i2c_scan_addr(uint8_t *i2c_addr, uint8_t device_addr) {
    // Extract slave address
    uint8_t frame = *i2c_addr;
    uint8_t slave_addr = frame >> 1;

    // Check if the device address matches the slave address
    if (slave_addr == device_addr) {
        return true;
    }
    return false;
}

uint8_t i2c_check_rw(uint8_t *i2c_addr) {
    return *i2c_addr & 0x01;
}