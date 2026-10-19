#include "mpu6500.h"

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43

static I2C_HandleTypeDef *g_hi2c = 0;

static HAL_StatusTypeDef wr_reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  return HAL_I2C_Master_Transmit(g_hi2c, MPU6500_I2C_ADDR, buf, 2, 100);
}

static HAL_StatusTypeDef rd_regs(uint8_t reg, uint8_t *buf, uint16_t n)
{
  if (HAL_I2C_Master_Transmit(g_hi2c, MPU6500_I2C_ADDR, &reg, 1, 100) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_I2C_Master_Receive(g_hi2c, MPU6500_I2C_ADDR, buf, n, 100);
}

int MPU6500_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t who = 0;
  g_hi2c = hi2c;

  if (HAL_I2C_IsDeviceReady(g_hi2c, MPU6500_I2C_ADDR, 3, 100) != HAL_OK) {
    return -1;
  }

  if (wr_reg(REG_PWR_MGMT_1, 0x00) != HAL_OK) {
    return -1;
  }
  HAL_Delay(50);

  wr_reg(REG_SMPLRT_DIV, 0x07);
  wr_reg(REG_CONFIG, 0x06);
  wr_reg(REG_GYRO_CONFIG, 0x00);
  wr_reg(REG_ACCEL_CONFIG, 0x00);

  if (rd_regs(REG_WHO_AM_I, &who, 1) != HAL_OK) {
    return -1;
  }

  if (who != 0x70 && who != 0x71 && who != 0x68) {
    return -2;
  }

  return 0;
}

int MPU6500_ReadAccel(MPU6500_RawAccel *out)
{
  uint8_t raw[6];
  if (!g_hi2c || !out) return -1;
  if (rd_regs(REG_ACCEL_XOUT_H, raw, 6) != HAL_OK) {
    return -1;
  }
  out->ax = (int16_t)((raw[0] << 8) | raw[1]);
  out->ay = (int16_t)((raw[2] << 8) | raw[3]);
  out->az = (int16_t)((raw[4] << 8) | raw[5]);
  return 0;
}

int MPU6500_ReadMotion(MPU6500_Motion *out)
{
  uint8_t raw[14];
  if (!g_hi2c || !out) return -1;
  if (rd_regs(REG_ACCEL_XOUT_H, raw, 14) != HAL_OK) {
    return -1;
  }
  out->ax = (int16_t)((raw[0] << 8) | raw[1]);
  out->ay = (int16_t)((raw[2] << 8) | raw[3]);
  out->az = (int16_t)((raw[4] << 8) | raw[5]);
  out->gx = (int16_t)((raw[8] << 8) | raw[9]);
  out->gy = (int16_t)((raw[10] << 8) | raw[11]);
  out->gz = (int16_t)((raw[12] << 8) | raw[13]);
  return 0;
}