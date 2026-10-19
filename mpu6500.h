#ifndef MPU6500_H
#define MPU6500_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifndef MPU6500_I2C_ADDR
#define MPU6500_I2C_ADDR  (0x68 << 1)
#endif

typedef struct {
  int16_t ax;
  int16_t ay;
  int16_t az;
} MPU6500_RawAccel;

typedef struct {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} MPU6500_Motion;

int MPU6500_Init(I2C_HandleTypeDef *hi2c);
int MPU6500_ReadAccel(MPU6500_RawAccel *out);
int MPU6500_ReadMotion(MPU6500_Motion *out);

#endif