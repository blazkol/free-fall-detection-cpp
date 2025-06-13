#ifndef I2C_H_DRIVER
#define I2C_H_DRIVER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define ICM_42670_P_ADDR  0x68  // Slave address of ICM-42670-P

typedef enum {
    // Accelerometer data
    ACCEL_DATA_X1 = 0x0B,   // Upper byte of Accel X-axis data
    ACCEL_DATA_X0 = 0x0C,   // Lower byte of Accel X-axis data
    ACCEL_DATA_Y1 = 0x0D,   // Upper byte of Accel Y-axis data
    ACCEL_DATA_Y0 = 0x0E,   // Lower byte of Accel Y-axis data
    ACCEL_DATA_Z1 = 0x0F,   // Upper byte of Accel Z-axis data
    ACCEL_DATA_Z0 = 0x10,   // Lower byte of Accel Z-axis data

    // Gyroscope data
    GYRO_DATA_X1  = 0x11,   // Upper byte of Gyro X-axis data
    GYRO_DATA_X0  = 0x12,   // Lower byte of Gyro X-axis data
    GYRO_DATA_Y1  = 0x13,   // Upper byte of Gyro Y-axis data
    GYRO_DATA_Y0  = 0x14,   // Lower byte of Gyro Y-axis data
    GYRO_DATA_Z1  = 0x15,   // Upper byte of Gyro Z-axis data
    GYRO_DATA_Z0  = 0x16,   // Lower byte of Gyro Z-axis data

    // Gyroscope config
    GYRO_CONFIG0  = 0x20,

    // Accelerometer config
    ACCEL_CONFIG0 = 0x21
    
} imu_register;

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

#endif // I2C_H_DRIVER