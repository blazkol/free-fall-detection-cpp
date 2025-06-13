#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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

typedef struct  {
    double ax, ay, az;
    double gx, gy, gz;
} imu_sample;

#ifdef __cplusplus
}
#endif

#endif // IMU_DRIVER_H