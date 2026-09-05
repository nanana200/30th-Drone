#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_WIDTH   128U
#define OLED_HEIGHT  64U

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_Print(uint8_t row, uint8_t col, const char *str);
void OLED_Printf(uint8_t row, uint8_t col, const char *fmt, ...);
uint8_t OLED_IsBusy(void);

#endif
