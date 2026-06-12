#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"
#include "i2c.h"

void OLED_Init(void);
void OLED_WriteCmd(uint8_t cmd);
void OLED_WriteData(uint8_t data);

void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_ShowStr(uint8_t x, uint8_t y, char *str);
void OLED_WriteDataBlock(uint8_t *data, uint8_t size);

void OLED_DrawChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_Printf(uint8_t x, uint8_t y, const char *format, ...);
#endif
