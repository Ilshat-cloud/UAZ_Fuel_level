
#include "stm32f1xx_hal.h"
#include "fonts.h"
#ifndef sh1106
#define sh1106


#define SSD1306_I2C_PORT        hi2c1
#define CHARGEPUMP 			0x8D
#define COLUMNADDR 			0x21
#define COMSCANDEC 			0xC8
#define COMSCANINC 			0xC0
#define DISPLAYALLON 		0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 			0xAE
#define DISPLAYON 			0xAF
#define EXTERNALVCC 		0x1
#define INVERTDISPLAY 		0xA7
#define MEMORYMODE 			0x20
#define NORMALDISPLAY 		0xA6
#define PAGEADDR 			0x22
#define SEGREMAP 			0xA0
#define SETCOMPINS 			0xDA
#define SETCONTRAST 		0x81
#define SETDISPLAYCLOCKDIV 	0xD5
#define SETDISPLAYOFFSET 	0xD3
#define SETHIGHCOLUMN 		0x10
#define SETLOWCOLUMN 		0x02
#define SETMULTIPLEX 		0xA8
#define SETPRECHARGE 		0xD9
#define SETSEGMENTREMAP 	0xA1
#define SETSTARTLINE		0x40
#define SETVCOMDETECT 		0xDB
#define SWITCHCAPVCC 		0x2

/* I2C address */
//#define USE_DMA					// uncomment if used I2C DMA mode
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR         0x78
//#define SSD1306_I2C_ADDR       0x7A
#endif

/* SSD1306 settings */
/* SSD1306 width in pixels */
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif
/* SSD1306 LCD height in pixels */
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT           64
#endif

   
   typedef enum 
   {
     Black=0x00,
     White=0x01
   } SSD1306_COLOR;
   
   typedef struct{
     uint16_t CurrentX;
     uint16_t CurrentY;
     uint8_t Inverted;
     uint8_t Initialized;
   } SSD1306_t;
extern I2C_HandleTypeDef SSD1306_I2C_PORT;

void startScreen(void);
uint8_t ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
int8_t ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y,SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_Draw_dot_colum_line(uint8_t x, uint8_t y);
#endif
