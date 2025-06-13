#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FREE = 0x00,
    BUSY = 0x01
} busy_flag;

typedef enum {
    WRITE = 0x00,
    READ  = 0x01
} rw_bit;

typedef enum {
    ACK  = 0x00,
    NACK = 0x01
} ack_bit;

void i2c_init(void);
void i2c_send_frame(uint8_t *i2c_addr, uint8_t data);
void i2c_send_addr(uint8_t *i2c_addr, uint8_t slave_addr, rw_bit rw);
uint8_t i2c_receive_frame(uint8_t *i2c_addr);
bool i2c_scan_addr(uint8_t *i2c_addr, uint8_t device_addr);
uint8_t i2c_check_rw(uint8_t *i2c_addr);

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H