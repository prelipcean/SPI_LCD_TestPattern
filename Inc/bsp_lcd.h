/*
 * bsp_lcd.h
 *
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_

#include "main.h"
#include "reg_util.h"
#include "SysTick_Delay.h"

// Color definition for LCD with ST7735 controller
#define COLOR_WHITE      	0xFFFF
#define COLOR_BLACK      	0x0000
#define COLOR_BLUE       	0x001F
#define COLOR_BRED        	0XF81F
#define COLOR_GRED 			0XFFE0
#define COLOR_GBLUE			0X07FF
#define COLOR_RED         	0xF800
#define COLOR_MAGENTA     	0xF81F
#define COLOR_GREEN       	0x07E0
#define COLOR_CYAN        	0x7FFF
#define COLOR_YELLOW      	0xFFE0
#define COLOR_BROWN 		0XBC40
#define COLOR_BRRED 		0XFC07
#define COLOR_GRAY  		0X8430
#define COLOR_GRAY0       	0xEF7D
#define COLOR_GRAY1       	0x8410
#define COLOR_GRAY2       	0x4208
#define COLOR_DARKBLUE      0X01CF
#define COLOR_LIGHTBLUE     0X7D7C
#define COLOR_GRAYBLUE      0X5458
#define COLOR_LIGHTGREEN    0X841F
#define COLOR_LIGHTGRAY     0XEF5B
#define COLOR_LGRAY 		0XC618
#define COLOR_LGRAYBLUE     0XA651
#define COLOR_LBBLUE        0X2B12

/* PIN DEFINES */
#define LCD_LED_PIN // will be connected to power supply directly

#define LCD_SCK_PIN 	GPIO_Pin_5 // D/CX_SCL PA5
#define LCD_SCK_PORT	GPIOA

#define LCD_SDA_PIN 	GPIO_Pin_7 // SDI/SDA PA7
#define LCD_SDA_PORT	GPIOA

#define LCD_A0_PIN 		GPIO_Pin_9// WRX_D/CX PA9
#define LCD_DCX_PORT 	GPIOA

#define LCD_RESET_PIN 	GPIO_Pin_7// RESX PC7
#define LCD_RESX_PORT 	GPIOC

#define LCD_CS_PIN 		GPIO_Pin_6 // CSX PB6
#define LCD_CSX_PORT 	GPIOB

/* MACRO definitions to set CS, RES, DC pins HIGH or LOW */
#define LCD_RESX_HIGH()				REG_SET_BIT(LCD_RESX_PORT->ODR, GPIO_ODR_OD7_Pos)
#define LCD_RESX_LOW()				REG_CLR_BIT(LCD_RESX_PORT->ODR, GPIO_ODR_OD7_Pos)

#define LCD_CSX_HIGH()				REG_SET_BIT(LCD_CSX_PORT->ODR, GPIO_ODR_OD6_Pos)
#define LCD_CSX_LOW()				REG_CLR_BIT(LCD_CSX_PORT->ODR, GPIO_ODR_OD6_Pos)

#define LCD_DCX_HIGH()				REG_SET_BIT(LCD_DCX_PORT->ODR, GPIO_ODR_OD9_Pos)
#define LCD_DCX_LOW()				REG_CLR_BIT(LCD_DCX_PORT->ODR, GPIO_ODR_OD9_Pos)

// CS pin macros
#define CS_L() LCD_CSX_LOW()
#define CS_H() LCD_CSX_HIGH()
// A0 pin macros
#define A0_L() LCD_DCX_LOW()
#define A0_H() LCD_DCX_HIGH()
// RESET pin macros
#define RST_L() LCD_RESX_LOW()
#define RST_H() LCD_RESX_HIGH()

// Use SPI1 and Half-Duplex Mode because we will only send data to the TFT
#define LCD_SPI	SPI1

/* Screen resolution */
#define LCD_SCREEN_W         128
#define LCD_SCREEN_H         160


// Global function
void BSP_LCD_Init(void);
void LCD_Write_Cmd(uint8_t cmd);
void LCD_Write_Data(uint8_t *buffer, uint32_t len);

#endif /* BSP_LCD_H_ */
