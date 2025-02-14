/*
 * lcd.c
 *
 * Created: 08-03-2024 11:54:21
 *  Author: sumit.chatterjee
 */ 
#include "lcd.h"
#include "i2c.h"
#include <string.h>
#include <stdlib.h>


#define FONT_SIZE 5
const unsigned char OledFontTable[][FONT_SIZE] = {
	
	0x00, 0x00, 0x00, 0x00, 0x00,   // space
	0x00, 0x00, 0x2f, 0x00, 0x00,   // !
	0x00, 0x07, 0x00, 0x07, 0x00,   // "
	0x14, 0x7f, 0x14, 0x7f, 0x14,   // #
	0x24, 0x2a, 0x7f, 0x2a, 0x12,   // $
	0x23, 0x13, 0x08, 0x64, 0x62,   // %
	0x36, 0x49, 0x55, 0x22, 0x50,   // &
	0x00, 0x05, 0x03, 0x00, 0x00,   // '
	0x00, 0x1c, 0x22, 0x41, 0x00,   // (
	0x00, 0x41, 0x22, 0x1c, 0x00,   // )
	0x14, 0x08, 0x3E, 0x08, 0x14,   // *
	0x08, 0x08, 0x3E, 0x08, 0x08,   // +
	0x00, 0x00, 0xA0, 0x60, 0x00,   // ,
	0x08, 0x08, 0x08, 0x08, 0x08,   // -
	0x00, 0x60, 0x60, 0x00, 0x00,   // .
	0x20, 0x10, 0x08, 0x04, 0x02,   // /
	
	0x3E, 0x51, 0x49, 0x45, 0x3E,   // 0
	0x00, 0x42, 0x7F, 0x40, 0x00,   // 1
	0x42, 0x61, 0x51, 0x49, 0x46,   // 2
	0x21, 0x41, 0x45, 0x4B, 0x31,   // 3
	0x18, 0x14, 0x12, 0x7F, 0x10,   // 4
	0x27, 0x45, 0x45, 0x45, 0x39,   // 5
	0x3C, 0x4A, 0x49, 0x49, 0x30,   // 6
	0x01, 0x71, 0x09, 0x05, 0x03,   // 7
	0x36, 0x49, 0x49, 0x49, 0x36,   // 8
	0x06, 0x49, 0x49, 0x29, 0x1E,   // 9
	
	0x00, 0x36, 0x36, 0x00, 0x00,   // :
	0x00, 0x56, 0x36, 0x00, 0x00,   // ;
	0x08, 0x14, 0x22, 0x41, 0x00,   // <
	0x14, 0x14, 0x14, 0x14, 0x14,   // =
	0x00, 0x41, 0x22, 0x14, 0x08,   // >
	0x02, 0x01, 0x51, 0x09, 0x06,   // ?
	0x32, 0x49, 0x59, 0x51, 0x3E,   // @
	
	0x7C, 0x12, 0x11, 0x12, 0x7C,   // A
	0x7F, 0x49, 0x49, 0x49, 0x36,   // B
	0x3E, 0x41, 0x41, 0x41, 0x22,   // C
	0x7F, 0x41, 0x41, 0x22, 0x1C,   // D
	0x7F, 0x49, 0x49, 0x49, 0x41,   // E
	0x7F, 0x09, 0x09, 0x09, 0x01,   // F
	0x3E, 0x41, 0x49, 0x49, 0x7A,   // G
	0x7F, 0x08, 0x08, 0x08, 0x7F,   // H
	0x00, 0x41, 0x7F, 0x41, 0x00,   // I
	0x20, 0x40, 0x41, 0x3F, 0x01,   // J
	0x7F, 0x08, 0x14, 0x22, 0x41,   // K
	0x7F, 0x40, 0x40, 0x40, 0x40,   // L
	0x7F, 0x02, 0x0C, 0x02, 0x7F,   // M
	0x7F, 0x04, 0x08, 0x10, 0x7F,   // N
	0x3E, 0x41, 0x41, 0x41, 0x3E,   // O
	0x7F, 0x09, 0x09, 0x09, 0x06,   // P
	0x3E, 0x41, 0x51, 0x21, 0x5E,   // Q
	0x7F, 0x09, 0x19, 0x29, 0x46,   // R
	0x46, 0x49, 0x49, 0x49, 0x31,   // S
	0x01, 0x01, 0x7F, 0x01, 0x01,   // T
	0x3F, 0x40, 0x40, 0x40, 0x3F,   // U
	0x1F, 0x20, 0x40, 0x20, 0x1F,   // V
	0x3F, 0x40, 0x38, 0x40, 0x3F,   // W
	0x63, 0x14, 0x08, 0x14, 0x63,   // X
	0x07, 0x08, 0x70, 0x08, 0x07,   // Y
	0x61, 0x51, 0x49, 0x45, 0x43,   // Z
	
	0x00, 0x7F, 0x41, 0x41, 0x00,   // [
	0x55, 0xAA, 0x55, 0xAA, 0x55,   // Backslash (Checker pattern)
	0x00, 0x41, 0x41, 0x7F, 0x00,   // ]
	0x04, 0x02, 0x01, 0x02, 0x04,   // ^
	0x40, 0x40, 0x40, 0x40, 0x40,   // _
	0x00, 0x03, 0x05, 0x00, 0x00,   // `
	
	0x20, 0x54, 0x54, 0x54, 0x78,   // a
	0x7F, 0x48, 0x44, 0x44, 0x38,   // b
	0x38, 0x44, 0x44, 0x44, 0x20,   // c
	0x38, 0x44, 0x44, 0x48, 0x7F,   // d
	0x38, 0x54, 0x54, 0x54, 0x18,   // e
	0x08, 0x7E, 0x09, 0x01, 0x02,   // f
	0x18, 0xA4, 0xA4, 0xA4, 0x7C,   // g
	0x7F, 0x08, 0x04, 0x04, 0x78,   // h
	0x00, 0x44, 0x7D, 0x40, 0x00,   // i
	0x40, 0x80, 0x84, 0x7D, 0x00,   // j
	0x7F, 0x10, 0x28, 0x44, 0x00,   // k
	0x00, 0x41, 0x7F, 0x40, 0x00,   // l
	0x7C, 0x04, 0x18, 0x04, 0x78,   // m
	0x7C, 0x08, 0x04, 0x04, 0x78,   // n
	0x38, 0x44, 0x44, 0x44, 0x38,   // o
	0xFC, 0x24, 0x24, 0x24, 0x18,   // p
	0x18, 0x24, 0x24, 0x18, 0xFC,   // q
	0x7C, 0x08, 0x04, 0x04, 0x08,   // r
	0x48, 0x54, 0x54, 0x54, 0x20,   // s
	0x04, 0x3F, 0x44, 0x40, 0x20,   // t
	0x3C, 0x40, 0x40, 0x20, 0x7C,   // u
	0x1C, 0x20, 0x40, 0x20, 0x1C,   // v
	0x3C, 0x40, 0x30, 0x40, 0x3C,   // w
	0x44, 0x28, 0x10, 0x28, 0x44,   // x
	0x1C, 0xA0, 0xA0, 0xA0, 0x7C,   // y
	0x44, 0x64, 0x54, 0x4C, 0x44,   // z
	
	0x00, 0x10, 0x7C, 0x82, 0x00,   // {
	0x00, 0x00, 0xFF, 0x00, 0x00,   // |
	0x00, 0x82, 0x7C, 0x10, 0x00,   // }
	0x00, 0x06, 0x09, 0x09, 0x06    // ~ (Degrees)
};






/*********************************************************************************************************************************************************************************************/

void oledSendCommand(uint8_t cmd){
	tw_start();
	twMSLA_Init(LCD_I2C_ADR,write_mode);
	twMaster_Write(0x00);
	twMaster_Write(cmd);
	tw_stop();
}


void oledSendByte(uint8_t cmd){
	tw_start();
	twMSLA_Init(LCD_I2C_ADR,write_mode);
	twMaster_Write(0x40);
	twMaster_Write(cmd);
	tw_stop();
}

void OLED_Clear()
{
	int i;
	
	oledSendCommand(SSD1306_SET_COLUMN_ADDR);
	oledSendCommand(0);
	oledSendCommand(127);
	
	oledSendCommand(SSD1306_SET_PAGE_ADDR);
	oledSendCommand(0);
	oledSendCommand(7);
	tw_start();
	twMSLA_Init(LCD_I2C_ADR,write_mode);
	twMaster_Write(0x40);
	
	for (i=0; i<1024; i++)      // Write Zeros to clear the display
	{
		twMaster_Write(0);
	}
	tw_stop();
	
	oledSendCommand(SSD1306_SET_COLUMN_ADDR);
	oledSendCommand(0);
	oledSendCommand(127);
	
	oledSendCommand(SSD1306_SET_PAGE_ADDR);
	oledSendCommand(0);
	oledSendCommand(7);
	
	tw_start();
	twMSLA_Init(LCD_I2C_ADR,write_mode);
    twMaster_Write(0x40);
	tw_stop();
}


void OLED_SetCursor(uint8_t lineNumber,uint8_t cursorPosition)
{
	
	/* Move the Cursor to specified position only if it is in range */
	if((lineNumber <= 7) && (cursorPosition <= 127))
	{
		
		OledLineNum=lineNumber;   /* Save the specified line number */
		OledCursorPos=cursorPosition; /* Save the specified cursor position */
		
		oledSendCommand(SSD1306_SET_COLUMN_ADDR);
		oledSendCommand(cursorPosition);
		oledSendCommand(127);
		
		oledSendCommand(SSD1306_SET_PAGE_ADDR);		//oledSendCommand(0x00+((2+lineNumber) & (0x0f)));
		//oledSendCommand(0x10+(((2+lineNumber) & (0xf0)) >> 4));
		oledSendCommand(lineNumber);
		oledSendCommand(7);
		
		
		
			tw_start();
			twMSLA_Init(LCD_I2C_ADR,write_mode);
			twMaster_Write(0x40);
			tw_stop();
	}
}


void OLED_DisplayChar(uint8_t ch)
{
    uint8_t dat,i=0;
    
    if(((OledCursorPos+FONT_SIZE)>=128) || (ch=='\n'))
    {
        /* If the cursor has reached to end of line on page1
         OR NewLine command is issued Then Move the cursor to next line */
        OLED_GoToNextLine();
    }
    if(ch!='\n') /* TODO */
    {
        ch = ch-0x20; // As the lookup table starts from Space(0x20)
        
        while(1)
        {
            dat= OledFontTable[ch][i]; /* Get the data to be displayed for LookUptable*/
            
            
            oledSendByte(dat); /* Display the data and keep track of cursor */
            OledCursorPos++;
            
            i++;
            
            if(i==FONT_SIZE) /* Exit the loop if End of char is encountered */
            {
                oledSendByte(0x00); /* Display the data and keep track of cursor */
                OledCursorPos++;
                break;
            }
        }
    }
}

void  OLED_GoToNextLine()
{
    /*Increment the current line number.
     In case it exceeds the limit, rool it back to first line */
    OledLineNum++;
    OledLineNum = OledLineNum&0x07;
    OLED_SetCursor(OledLineNum,0); /* Finally move it to next line */
}


void  OLED_GoToLine(uint8_t lineNumber)
{
    if(lineNumber<8)
    {   /* If the line number is within range
         then move it to specified line and keep track*/
        OledLineNum = lineNumber;
        OLED_SetCursor(OledLineNum,0);
    }
}


void OLED_DisplayString(uint8_t *ptr)
{
	while(*ptr)
	OLED_DisplayChar(*ptr++);
}


void OLED_Init(void)
{
	tw_init(TW_FREQ_100K);
	
	oledSendCommand(SSD1306_DISPLAY_OFF);
	oledSendCommand(SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO);
	oledSendCommand(0x80);
	oledSendCommand(SSD1306_SET_MULTIPLEX_RATIO);
	oledSendCommand(0x3F);
	oledSendCommand(SSD1306_SET_DISPLAY_OFFSET);
	oledSendCommand(0x0);
	oledSendCommand(SSD1306_SET_START_LINE | 0x0);
	oledSendCommand(SSD1306_CHARGE_PUMP);
	oledSendCommand(0x14);
	oledSendCommand(SSD1306_MEMORY_ADDR_MODE);
	oledSendCommand(0x00);
	oledSendCommand(SSD1306_SET_SEGMENT_REMAP | 0x1);
	oledSendCommand(SSD1306_COM_SCAN_DIR_DEC);
	oledSendCommand(SSD1306_SET_COM_PINS);
	oledSendCommand(0x12);
	oledSendCommand(SSD1306_SET_CONTRAST_CONTROL);
	oledSendCommand(0xCF);
	oledSendCommand(SSD1306_SET_PRECHARGE_PERIOD);
	oledSendCommand(0xF1);
	oledSendCommand(SSD1306_SET_VCOM_DESELECT);
	oledSendCommand(0x40);
	oledSendCommand(SSD1306_DISPLAY_ALL_ON_RESUME);
	oledSendCommand(SSD1306_NORMAL_DISPLAY);
	oledSendCommand(SSD1306_DISPLAY_ON);
	
	OLED_Clear();  /* Clear the complete LCD during init */
}

void OLED_ScrollMessage(uint8_t lineNum, char *strptr)
{
    unsigned char i,j,k,l,cursor,ch;
    
    if(lineNum > 7)
        lineNum = 0; // Select first line if the lineNumberToStartDisplay is out of range
    
    for(i=0;strptr[i];i++)
    {
        /* Loop to display the complete string,    each time 16 chars are displayed and
         pointer is incremented to point to next char */
        
        for(k=0;k<6;k++)
        {
            OLED_SetCursor(lineNum,6-k);     //Move the Cursor to first line
            cursor = 6-k;
            
            for(j=0;(strptr[i+j] && (cursor<128));j++)
            {
                ch = strptr[i+j]-0x20;
                for(l=0;(l<5) && (cursor<128);l++)//Display first 16 Chars or till Null char is reached
                {
                    oledSendByte(OledFontTable[ch][l]);
                    cursor++;
                }
                
                oledSendByte(0);
                _delay_us(10);
                cursor++;
            }
            _delay_ms(20);
        }
    }
}


