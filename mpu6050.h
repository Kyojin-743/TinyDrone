/*
 * mpu6050.h
 *
 *  Created on: Feb 13, 2026
 *      Author: kyojin
 */

#ifndef MPU6050_H_
#define MPU6050_H_


/* INCLUDES */

//C-Std Lib.
#include <stdint.h>
#include <stdbool.h>

/* GLOBALS CONSTS AND MACROS */

//MPU6050 Default I2C Address
#define MPU6050_ADDR        0x68

//MPU6050 Register Map

#define MPU6050_SMPLRT_DIV_R          0x19
#define MPU6050_CONFIG_R              0x1A
#define MPU6050_GYRO_CONFIG_R         0x1B
#define MPU6050_ACCEL_CONFIG_R        0x1C

#define MPU6050_MOT_THR_R             0x1F

#define MPU6050_FIFO_EN_R             0x23

#define MPU6050_I2C_MST_CTRL_R        0x24

#define MPU6050_I2C_SLV0_ADDR_R       0x25
#define MPU6050_I2C_SLV0_REG_R        0x26
#define MPU6050_I2C_SLV0_CTRL_R       0x27

#define MPU6050_I2C_SLV1_ADDR_R       0x28
#define MPU6050_I2C_SLV1_REG_R        0x29
#define MPU6050_I2C_SLV1_CTRL_R       0x2A

#define MPU6050_I2C_SLV2_ADDR_R       0x2B
#define MPU6050_I2C_SLV2_REG_R        0x2C
#define MPU6050_I2C_SLV2_CTRL_R       0x2D

#define MPU6050_I2C_SLV3_ADDR_R       0x2E
#define MPU6050_I2C_SLV3_REG_R        0x2F
#define MPU6050_I2C_SLV3_CTRL_R       0x30

#define MPU6050_I2C_SLV4_ADDR_R       0x31
#define MPU6050_I2C_SLV4_REG_R        0x32
#define MPU6050_I2C_SLV4_CTRL_R       0x34

#define MPU6050_I2C_MST_STATUS_R      0x36

#define MPU6050_INT_PIN_CFG_R         0x37
#define MPU6050_INT_ENABLE_R          0x38
#define MPU6050_DMP_INT_STATUS_R      0x39
#define MPU6050_INT_STATUS_R          0x3A

#define MPU6050_ACCEL_XOUT_H_R        0x3B
#define MPU6050_ACCEL_XOUT_L_R        0x3C
#define MPU6050_ACCEL_YOUT_H_R        0x3D
#define MPU6050_ACCEL_YOUT_L_R        0x3E
#define MPU6050_ACCEL_ZOUT_H_R        0x3F
#define MPU6050_ACCEL_ZOUT_L_R        0x40

#define MPU6050_TEMP_OUT_H_R          0x41
#define MPU6050_TEMP_OUT_L_R          0x42

#define MPU6050_GYRO_XOUT_H_R         0x43
#define MPU6050_GYRO_XOUT_L_R         0x44
#define MPU6050_GYRO_YOUT_H_R         0x45
#define MPU6050_GYRO_YOUT_L_R         0x46
#define MPU6050_GYRO_ZOUT_H_R         0x47
#define MPU6050_GYRO_ZOUT_L_R         0x48

#define MPU6050_I2C_SLV0_DO_R         0x63
#define MPU6050_I2C_SLV1_DO_R         0x64
#define MPU6050_I2C_SLV2_DO_R         0x65
#define MPU6050_I2C_SLV3_DO_R         0x66
#define MPU6050_I2C_MST_DELAY_CTRL_R  0x67

#define MPU6050_SIGNAL_PATH_RESET_R   0x68
#define MPU6050_MOT_DETECT_CTRL_R     0x69
#define MPU6050_USER_CTRL_R           0x6A

#define MPU6050_PWR_MGMT_1_R          0x6B
#define MPU6050_PWR_MGMT_2_R          0x6C
#define MPU6050_BANK_SEL_R            0x6D
#define MPU6050_MEM_START_ADDR_R      0x6E
#define MPU6050_MEM_R_W_R             0x6F

#define MPU6050_DMP_CFG_1_R           0x70
#define MPU6050_DMP_CFG_2_R           0x71
#define MPU6050_FIFO_COUNTH_R         0x72
#define MPU6050_FIFO_COUNTL_R         0x73
#define MPU6050_FIFO_R_W_R            0x74

#define MPU6050_WHO_AM_I_R            0x75

/* Structs & Enums & Types*/

typedef void (*func)(void);

#ifndef VEC3F
typedef struct { float x, y, z; } Vec3f;
#define VEC3F
#endif
typedef struct { Vec3f a, g; } MpuData;

#ifndef nullptr
#define nullptr ((void*)0)
#endif
/* SUB ROUTINE PROTOTYPES */

bool mpu_init(void);
MpuData mpu_read(void);

#endif /* MPU6050_H_ */
