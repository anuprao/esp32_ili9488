/**
 * @file ili9488.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9488.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9488_send_cmd(uint8_t cmd);
static void ili9488_send_data(void * data, uint16_t length);
static void ili9488_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9488_init(void)
{
	lcd_init_cmd_t ili_init_cmds[]={
		
		//CABC Control 9
		//{0xCF, {0x00, 0x83, 0X30}, 3},
		
		//{0xED, {0x64, 0x03, 0X12, 0X81}, 4},
		
		//{0xE8, {0x85, 0x01, 0x79}, 3},
		
		//CABC Control 5
		//{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
		
		//Adjust Control 3
		//{0xF7, {0x20}, 1},
		
		//{0xEA, {0x00, 0x00}, 2},
		
		//Positive Gamma Control
		//??
		
		// Negative Gamma Control
		//??
		
		// Positive Gamma Control
		//{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
		{0xE0, {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15},
		
		// Negative Gamma Control
		//{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
		{0XE1, {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15},
		
		// Power Control 1
		{0xC0, {0x17, 0x15}, 2},			
		
		// Power Control 2
		{0xC1, {0x41}, 1},		
		
		// VCOM Control
		{0xC5, {0x00, 0x12, 0x80}, 3},	
		
		// VCOM Control
		//{0xC7, {0xBE}, 1},			
		
		// Memory Access Control
		//{0x36, {0x28}, 1},
		{0x36, {0x48}, 1},			
		
		// Pixel Interface Format
		// Original code for ILI9341
		//{0x3A, {0x55}, 1},			
		{0x3A, {0x66}, 1},
		
		// Interface Mode Control
		{0xB0, {0x00}, 1},
		
		// Frame Rate Control
		{0xB1, {0xA0}, 1},
		
		//??
		//{0xF2, {0x08}, 1},
		
		//??
		//{0x26, {0x01}, 1},
		
		// Display Inversion Control
		{0xB4, {0x02}, 1},
		
		// Display Function Control
		// Original code for ILI9341
		//{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
		{0xB6, {0x02, 0x02, 0x3B}, 3},
		
		// Entry Mode Set
		{0xB7, {0xC6}, 1},
		
		//Adjust Control 3
		{0xF7, {0xA9, 0x51, 0x2C, 0x82}, 4},
		
		// Set_column_address 4 parameters
		//{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4}, // 0xEF = 240
		
		// Set_page_address 4 parameters
		//{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4}, // 0x013F = 320
		
		// Write_memory_start  
		// ?? 
		//{0x2C, {0}, 0},
		
		//Exit Sleep
		{0x11, {0}, 0x80},
		
		//Display on
		{0x29, {0}, 0x80},
		
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
	gpio_set_direction(ILI9488_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(ILI9488_RST, GPIO_MODE_OUTPUT);
	gpio_set_direction(ILI9488_BCKL, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(ILI9488_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(ILI9488_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);


	printf("ILI9488 initialization.\n");


	//Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes!=0xff) {
		ili9488_send_cmd(ili_init_cmds[cmd].cmd);
		ili9488_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	///Enable backlight
	printf("Enable backlight.\n");
	gpio_set_level(ILI9488_BCKL, 1);
}
/*
const uint16_t arrSample_16[] = 
	
	//GREEN	1st pixel - 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	//RED 1st pixel - 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	//BLUE 1st pixel - 0x0000, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	
	//BLUE 1st and Red 2nd pixel - 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	//RED 2nd pixel - 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	
	//BLUE 2nd and Red 2nd pixel - 0x0000, 0xFF00, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	//BLUE 2nd and Green 2nd pixel - 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
	{ 0x0000, 0x0000, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
*/

const uint8_t arrSample[] = 
	{ 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	
// Used in unbuffered mode
void ili9488_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)
{
	uint8_t data[4];

	/*Column addresses*/
	ili9488_send_cmd(0x2A);
	data[0] = (x1 >> 8) & 0xFF;
	data[1] = x1 & 0xFF;
	data[2] = (x2 >> 8) & 0xFF;
	data[3] = x2 & 0xFF;
	ili9488_send_data(data, 4);

	/*Page addresses*/
	ili9488_send_cmd(0x2B);
	data[0] = (y1 >> 8) & 0xFF;
	data[1] = y1 & 0xFF;
	data[2] = (y2 >> 8) & 0xFF;
	data[3] = y2 & 0xFF;
	ili9488_send_data(data, 4);

	/*Memory write*/
	ili9488_send_cmd(0x2C);

	uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);
	uint16_t buf[LV_HOR_RES];

	uint32_t i;
	if(size < LV_HOR_RES) {
		for(i = 0; i < size; i++) buf[i] = color.full;

	} else {
		for(i = 0; i < LV_HOR_RES; i++) buf[i] = color.full;
	}

	while(size > LV_HOR_RES) {
		ili9488_send_color(buf, LV_HOR_RES * 2);
		size -= LV_HOR_RES;
	}
	
	//Original code
	//ili9488_send_color(buf, size * 2);	/*Send the remaining data*/
	
	//ili9488_send_color(buf, size * 2);
	ili9488_send_color(arrSample, 32);
	
	printf("size_fill = %d\r\n", size);
}

uint8_t buffConv[320*3] = {};

// Used in buffered mode
void ili9488_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map)
{
	uint8_t data[4];
	
	uint8_t* pbuf;
	uint16_t i, len, chunk;
	//uint8_t r,g,b;
	
	lv_color16_t s;
	lv_color16_t* pColor = color_map;

	/*Column addresses*/
	ili9488_send_cmd(0x2A);
	data[0] = (x1 >> 8) & 0xFF;
	data[1] = x1 & 0xFF;
	data[2] = (x2 >> 8) & 0xFF;
	data[3] = x2 & 0xFF;
	ili9488_send_data(data, 4);

	/*Page addresses*/
	ili9488_send_cmd(0x2B);
	data[0] = (y1 >> 8) & 0xFF;
	data[1] = y1 & 0xFF;
	data[2] = (y2 >> 8) & 0xFF;
	data[3] = y2 & 0xFF;
	ili9488_send_data(data, 4);

	/*Memory write*/
	ili9488_send_cmd(0x2C);
	
	uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);

	//Original code
	//ili9488_send_color((void*)color_map, size * 2);	/*Send the remaining data*/
	
	//ili9488_send_color((void*)color_map, size * 2);
	//ili9488_send_color(arrSample, 32);
	
	chunk = 0;
	len = size;
	while(len > 0)
	{
		chunk = (320 < len)? 320: len;
		
		pbuf = buffConv;
		for(i = 0; i<chunk; i++)
		{
			s = *pColor;
			
			*pbuf = s.red << 3;
			pbuf++;
			
			*pbuf = (s.green_h << 3 | s.green_l) << 2;
			pbuf++;
			
			*pbuf = s.blue << 3;
			pbuf++;
			
			pColor++;
		}
		
		ili9488_send_color(buffConv, chunk*3);
		
		len = len - chunk;
	}
	
	printf("size_flush = %d\r\n", size);
	
	//lv_flush_ready();

}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ili9488_send_cmd(uint8_t cmd)
{
	gpio_set_level(ILI9488_DC, 0);	 /*Command mode*/
	disp_spi_send_data(&cmd, 1);
}

static void ili9488_send_data(void * data, uint16_t length)
{
	gpio_set_level(ILI9488_DC, 1);	 /*Data mode*/
	disp_spi_send_data(data, length);
}

static void ili9488_send_color(void * data, uint16_t length)
{
    gpio_set_level(ILI9488_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}
