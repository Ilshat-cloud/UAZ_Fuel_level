
#include "sh1106.h"
#include "cmsis_os2.h"
/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
/* Private variable */
static SSD1306_t SSD1306;

static int8_t ssd1306_WriteCommand(uint8_t command);
static int8_t ssd1306_WriteData(uint8_t* data, uint16_t size);




static const uint8_t dota[128]={
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x80,
0x00,0x00,0x1F,0xC0,
0x00,0x00,0x3F,0xC0,
0x00,0x00,0x3F,0xC0,
0x00,0x00,0x3F,0xE0,
0x00,0x00,0x7F,0xE0,
0x00,0x00,0x7F,0x70,
0x00,0x00,0x7E,0x20,
0x00,0x00,0x7F,0x00,
0x00,0x00,0x7F,0x00,
0x00,0x00,0xFF,0x00,
0x00,0x01,0xFF,0x00,
0x00,0x03,0xFF,0xB0,
0x00,0x3f,0xFF,0xF0,
0x01,0xff,0xFF,0xF0,
0x03,0xff,0xFF,0xF0,
0x03,0xff,0xFD,0x90,
0x07,0xff,0xF9,0xB0,
0x07,0xff,0xF1,0xA0,
0x07,0xff,0x80,0x20,
0x07,0xff,0x00,0x00,
0x06,0xEE,0x00,0x00,
0x00,0xCC,0x00,0x00,
0x00,0xD8,0x00,0x00,
0x00,0xFC,0x80,0x00,
0x00,0x77,0xC0,0x00,
0x00,0x19,0xC0,0x00,
0x00,0x18,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00};

uint8_t ssd1306_Init(void) {
        
	if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 3, 100) != HAL_OK)
	{
		SSD1306.Initialized = 0;
		/* Return false */
		return 0;
	}
        
        osDelay(100);
	
	/* Init LCD */
	 ssd1306_WriteCommand(0xAE); //display off
         ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
         ssd1306_WriteCommand(0x80); 
         ssd1306_WriteCommand(0xA8);
         ssd1306_WriteCommand(0x3F);
         ssd1306_WriteCommand(0xD3);
         ssd1306_WriteCommand(0x00);
         ssd1306_WriteCommand(0x40);
         ssd1306_WriteCommand(0x8D);
         ssd1306_WriteCommand(0x10);
         ssd1306_WriteCommand(0x20);
         ssd1306_WriteCommand(0x00);
         ssd1306_WriteCommand(0xA1);
         ssd1306_WriteCommand(0xC8);
         ssd1306_WriteCommand(0xDA);
         ssd1306_WriteCommand(0x12);
         ssd1306_WriteCommand(0x81);
         ssd1306_WriteCommand(0xFF);
         ssd1306_WriteCommand(0xD9);
         ssd1306_WriteCommand(0x22);
         ssd1306_WriteCommand(0xDB);
         ssd1306_WriteCommand(0x40);
         ssd1306_WriteCommand(0xA4);
         ssd1306_WriteCommand(0xA6);
         ssd1306_WriteCommand(0xAF);
         
         
         
         
	 ssd1306_Fill(White);
	
	/* Update screen */
	ssd1306_UpdateScreen();
	
	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	
	/* Initialized OK */
	SSD1306.Initialized = 1;
	
	/* Return OK */
	return 1;
}


void ssd1306_Fill(SSD1306_COLOR color) 
{
 
        uint16_t i;
	/* Set memory */
        uint8_t colot_t=(color==Black)?0x00:0xFF;
	for (i=0; i<sizeof(SSD1306_Buffer);i++)
        {
          SSD1306_Buffer[i]=colot_t;
        }
}


int8_t ssd1306_UpdateScreen(void) 
{
 
        uint8_t i;
        if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 3, 100) != HAL_OK)
	{
		SSD1306.Initialized = 0;
		/* Return false */
		return -1;
	}

        for (i=0; i<8; i++)
        {
          ssd1306_WriteCommand(0xB0+i);
          ssd1306_WriteCommand(SETLOWCOLUMN);
          ssd1306_WriteCommand(SETHIGHCOLUMN);
          
          ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
          
        }
        return 0;
        
}



void ssd1306_DrawPixel(uint8_t x, uint8_t y,SSD1306_COLOR color) {
	if (x >= SSD1306_WIDTH ||y >= SSD1306_HEIGHT) 
        {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (SSD1306_COLOR)!color;
	}
	
	/* Set color */
	if (color == White) 
        {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}


void ssd1306_Draw_dot_colum_line(uint8_t x, uint8_t y) {
	if (x >= SSD1306_WIDTH ||y >= SSD1306_HEIGHT) 
    {
		/* Error */
		return;
	}
        uint16_t i;
	for (i=(x+(y / 8) * SSD1306_WIDTH); i<(127-x+((y / 8) * SSD1306_WIDTH)); i++)
		{
			SSD1306_Buffer[i+((y / 8) * SSD1306_WIDTH)] |= 1 << (y % 8);;
		}

	
}





static int8_t ssd1306_WriteCommand(uint8_t command)
{
#ifdef USE_DMA
  while(HAL_I2C_GetState(&SSD1306_I2C_PORT) != HAL_I2C_STATE_READY);
  HAL_I2C_Mem_Write_DMA(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &command, 1);
#else
  if (HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 10)==HAL_OK){
    return 0;
  }
  return -1;
#endif
}

static int8_t ssd1306_WriteData(uint8_t* data, uint16_t size)
{
#ifdef USE_DMA
  while(HAL_I2C_GetState(&SSD1306_I2C_PORT) != HAL_I2C_STATE_READY);
  HAL_I2C_Mem_Write_DMA(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, data, size);
#else
  
  if (HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, data, size, 100)==HAL_OK){
    return 0;
  }
  return -1;
#endif
}
#ifdef USE_DMA
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == SSD1306_I2C_PORT.Instance)
	{
		//TODO:
	}
}
#endif

///////////***********************////////////////////

char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;
	
	// Check remaining space on current line
	if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}
	
	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	
	// The current space is now taken
	SSD1306.CurrentX += Font.FontWidth;
	
	// Return written char for validation
	return ch;
}

//------------------------startScreen-------------------
void startScreen() {
  uint8_t i;
        uint8_t j;
	for (i = 0; i < 128; i++)
	{       
                uint8_t i2=i/4;
		for (j = 0; j < 8; j++)
		{
			if ((dota[i])&(0x01<<(7-j)))
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j+(i%4)*8, (SSD1306.CurrentY + i2),White);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j+(i%4)*8, (SSD1306.CurrentY + i2), Black);
			}
                        
		}
	}
  ssd1306_UpdateScreen();
  osDelay(200);
}
//
//  Write full string to screenbuffer
//
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
	// Write until null-byte
	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
		{
			// Char could not be written
			return *str;
		}
		
		// Next char
		str++;
	}
	
	// Everything ok
	return *str;
}

//
//	Position the cursor
//
void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

