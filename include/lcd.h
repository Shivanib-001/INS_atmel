/*
 * lcd.h
 *
 * Created: 08-03-2024 11:54:37
 *  Author: sumit.chatterjee
 */ 


#ifndef LCD_H_
#define LCD_H_

#ifdef __cplusplus
extern "C" {
	#endif
	
	#if (__GNUC__ * 100 + __GNUC_MINOR__) < 303
	#error "This library requires AVR-GCC 3.3 or later, update to newer AVR-GCC compiler !"
	#endif
	
	#include <inttypes.h>
	#include <avr/pgmspace.h>
	#include "i2c.h"
	

	#define LCD_I2C_ADR         (0x78 >> 1)    // 7 bit slave-adress without r/w-bit
	// r/w-bit are set/unset by library
	// e.g. 8 bit slave-adress:
	// 0x78 = adress 0x3C with cleared r/w-bit (write-mode)


	#define NORMALSIZE 1
	#define DOUBLESIZE 2
	
	#define LCD_DISP_OFF        0xAE
	#define LCD_DISP_ON        0xAF
	
	#define WHITE            0x01
	#define BLACK            0x00
	
	#define DISPLAY_WIDTH        128
	#define DISPLAY_HEIGHT        64
	/***************************************************************************************************
 Macros to find the mod of a number
 ****************************************************************************************************/
#define util_GetMod8(dividend, divisor) (uint8_t)(dividend - (divisor * (uint8_t)(dividend / divisor)))
#define util_GetMod16(dividend, divisor) (uint16_t)(dividend - (divisor * (uint16_t)(dividend / divisor)))
#define util_GetMod32(dividend, divisor) (uint32_t)(dividend - (divisor * (uint32_t)(dividend / divisor)))
/***************************************************************************************************/
	
	/******************************************************************************
 standard defs
 ******************************************************************************/

typedef signed char sint8_t;
typedef unsigned char uint8_t;
typedef signed int sint16_t;
typedef signed long int sint32_t;

#define C_BINARY_U8 2u
#define C_DECIMAL_U8 10u
#define C_HEX_U8 16u

#define C_SINT8_MAX 0x7F
#define C_SINT8_MIN -128

#define C_UINT8_MAX 0xFFu
#define C_UINT8_MIN 0x00u

#define C_SINT16_MAX 32767
#define C_SINT16_MIN -32768

#define C_UINT16_MAX 0xFFFFu
#define C_UINT16_MIN 0x00u

#define C_SINT32_MAX 2147483647
#define C_SINT32_MIN -2147483648

#define C_UINT32_MAX 0xFFFFFFFFu
#define C_UINT32_MIN 0x00u

typedef enum
{
    E_BINARY = 2,
    E_DECIMAL = 10,
    E_HEX = 16
} NumericSystem_et;
	
	#define util_Dec2Ascii(Dec) ((Dec) + 0x30)
	#define util_Ascii2Dec(Asc) ((Asc)-0x30)
	#define util_Hex2Ascii(Hex) (((Hex) > 0x09) ? ((Hex) + 0x37) : ((Hex) + 0x30))
	#define util_Ascii2Hex(Asc) (((Asc) > 0x39) ? ((Asc)-0x37) : ((Asc)-0x30))
	#define util_GetBitStatus(x, bit) (((x) & (util_GetBitMask(bit))) != 0u)
	#define util_GetBitMask(bit) (1 << (bit))
	
	
	/******************************************************************************
 SSD1306 ID and Command List
 ******************************************************************************/
#define SSD1306_ADDRESS 0x3C

#define SSD1306_COMMAND 0x00
#define SSD1306_DATA 0xC0
#define SSD1306_DATA_CONTINUE 0x40

#define SSD1306_SET_CONTRAST_CONTROL 0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_DISPLAY_ALL_ON 0xA5
#define SSD1306_NORMAL_DISPLAY 0xA6
#define SSD1306_INVERT_DISPLAY 0xA7
#define SSD1306_DISPLAY_OFF 0xAE
#define SSD1306_DISPLAY_ON 0xAF
#define SSD1306_NOP 0xE3

#define SSD1306_HORIZONTAL_SCROLL_RIGHT 0x26
#define SSD1306_HORIZONTAL_SCROLL_LEFT 0x27
#define SSD1306_HORIZONTAL_SCROLL_VERTICAL_AND_RIGHT 0x29
#define SSD1306_HORIZONTAL_SCROLL_VERTICAL_AND_LEFT 0x2A
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3

#define SSD1306_SET_LOWER_COLUMN 0x00
#define SSD1306_SET_HIGHER_COLUMN 0x10
#define SSD1306_MEMORY_ADDR_MODE 0x20
#define SSD1306_SET_COLUMN_ADDR 0x21
#define SSD1306_SET_PAGE_ADDR 0x22

#define SSD1306_SET_START_LINE 0x40
#define SSD1306_SET_SEGMENT_REMAP 0xA0
#define SSD1306_SET_MULTIPLEX_RATIO 0xA8
#define SSD1306_COM_SCAN_DIR_INC 0xC0
#define SSD1306_COM_SCAN_DIR_DEC 0xC8
#define SSD1306_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_SET_COM_PINS 0xDA
#define SSD1306_CHARGE_PUMP 0x8D

#define SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO 0xD5
#define SSD1306_SET_PRECHARGE_PERIOD 0xD9
#define SSD1306_SET_VCOM_DESELECT 0xDB

uint8_t OledLineNum, OledCursorPos;
/****************************************************************************/
	/* Commonly used LCD macros/Constants
	 ***************************************************************************************************/
	 #define C_DefaultDigits 10

	 #define C_OledFirstLine_U8 0x00u
	 #define C_OledLastLine_U8 0x07u

	 #define C_FirstLineAddress_U8 0xB8
	 #define C_LastLineAddress_U8 0xBF

	 #define C_DisplayDefaultDigits_U8 0xffu                              // Will display the exact digits in the number
	 #define C_MaxDigitsToDisplay_U8 10u                                  // Max decimal/hexadecimal digits to be displayed
	 #define C_NumOfBinDigitsToDisplay_U8 16u                             // Max bits of a binary number to be displayed
	 #define C_MaxDigitsToDisplayUsingPrintf_U8 C_DisplayDefaultDigits_U8 /* Max dec/hexadecimal digits to be displayed using printf */

	 #define C_MaxBarGraphs_U8 5
	void OLED_Clear();
	void oledSendCommand(uint8_t cmd);
	void oledSendByte(uint8_t cmd);
	void OLED_SetCursor(uint8_t lineNumber,uint8_t cursorPosition);
	void OLED_DisplayChar(uint8_t ch);
	void OLED_GoToNextLine();
	void OLED_GoToLine(uint8_t lineNumber);
	void OLED_DisplayString(uint8_t *ptr);
	void OLED_Init(void);
	void OLED_ScrollMessage(uint8_t lineNum, char *strptr);
	//void OLED_SH1106(void);
	#ifdef __cplusplus
}
#endif





#endif /* LCD_H_ */