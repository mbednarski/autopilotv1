/**
  ******************************************************************************
  * @file           : ssd1306.h
  * @brief          : Header for SSD1306 OLED Display Driver (128x32)
  ******************************************************************************
  */

#ifndef __SSD1306_H
#define __SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

/* SSD1306 I2C Configuration */
#define SSD1306_I2C_ADDR        0x78    // 0x3C << 1 (shifted for HAL)
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          32

/* SSD1306 Commands */
#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6
#define SSD1306_CMD_INVERSE_DISPLAY     0xA7
#define SSD1306_CMD_SET_MUX_RATIO       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_MEMORY_MODE         0x20
#define SSD1306_CMD_COLUMN_ADDR         0x21
#define SSD1306_CMD_PAGE_ADDR           0x22
#define SSD1306_CMD_COM_SCAN_INC        0xC0
#define SSD1306_CMD_COM_SCAN_DEC        0xC8
#define SSD1306_CMD_SEG_REMAP           0xA0
#define SSD1306_CMD_SEG_REMAP_INV       0xA1
#define SSD1306_CMD_SET_COM_PINS        0xDA
#define SSD1306_CMD_CHARGE_PUMP         0x8D
#define SSD1306_CMD_SET_PRECHARGE       0xD9
#define SSD1306_CMD_SET_VCOMH           0xDB
#define SSD1306_CMD_DEACTIVATE_SCROLL   0x2E

/* Public Functions */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateScreen(void);
void SSD1306_GotoXY(uint8_t x, uint8_t y);
void SSD1306_Putc(char ch);
void SSD1306_Puts(char* str);
void SSD1306_Clear(void);
void SSD1306_SetCursor(uint8_t x, uint8_t line);  // line = 0,1,2,3 for 32px height

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H */
