// Author: Chongbing Yu <bd8ejk@foxmail.com>
// Date: 2025-06-23

#ifndef ICM42688_H
#define ICM42688_H

#define ICM42688_ADDRESS            0x69
#define ICM42688_WHO_AM_I           0x75
#define ICM42688_PWR_MGMT0          0x4E
#define ICM42688_ACCEL_CONFIG0      0x50
#define ICM42688_GYRO_CONFIG0       0x4F
#define ICM42688_TEMP_DATA1         0x1D
#define ICM42688_TEMP_DATA0         0x1E
#define ICM42688_ACCEL_DATA_X1      0x1F
#define ICM42688_ACCEL_DATA_X0      0x20
#define ICM42688_ACCEL_DATA_Y1      0x21
#define ICM42688_ACCEL_DATA_Y0      0x22
#define ICM42688_ACCEL_DATA_Z1      0x23
#define ICM42688_ACCEL_DATA_Z0      0x24
#define ICM42688_GYRO_DATA_X1       0x25
#define ICM42688_GYRO_DATA_X0       0x26
#define ICM42688_GYRO_DATA_Y1       0x27
#define ICM42688_GYRO_DATA_Y0       0x28
#define ICM42688_GYRO_DATA_Z1       0x29
#define ICM42688_GYRO_DATA_Z0       0x2A

void icm42688_init(void);
void icm42688_read_data(float* temp, float* acc, float* gyro);

#endif