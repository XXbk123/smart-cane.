#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 7-bit address 0x3C -> HAL expects left-shifted 8-bit form */
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR  (0x3C << 1)
#endif

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64

/* return 0 = OK, -1 = device not answering */
int SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t on);
void SSD1306_DrawChar(uint8_t x, uint8_t y, char c);
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *s);

#endif