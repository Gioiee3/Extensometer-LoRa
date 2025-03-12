#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include <esp_log.h>
#include "HD44780.h"
#include "sx1278.h"
#include "button.h"

// LCD module defines
#define LCD_LINEONE             0x00        // start of line 1
#define LCD_LINETWO             0x40        // start of line 2
#define LCD_LINETHREE           0x14        // start of line 3
#define LCD_LINEFOUR            0x54        // start of line 4

#define LCD_BACKLIGHT           0x08        // backlight on
#define LCD_ENABLE              0x04               
#define LCD_COMMAND             0x00
#define LCD_WRITE               0x01

#define LCD_SET_DDRAM_ADDR      0x80
#define LCD_READ_BF             0x40
#define SET_CGRAM_ADDRESS	 	0x40

// LCD instructions
#define LCD_CLEAR               0x01        // replace all characters with ASCII 'space'
#define LCD_HOME                0x02        // return cursor to first position on first line
#define LCD_ENTRY_MODE          0x06        // auto di chuyển con trỏ đến vị trí tiếp theo khi xuất ra LCD 1 kí tự
#define LCD_DISPLAY_OFF         0x08        // Tắt hiển thị, tắt con trỏ
#define LCD_DISPLAY_ON          0x0C        // Bật hiển thị và tắt con trỏ
#define LCD_FUNCTION_RESET      0x30        // reset the LCD
#define LCD_FUNCTION_SET_4BIT   0x28        // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR          0x80        // set cursor position

// Pin mappings
// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> Backlight
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

uint8_t char_battery_gate[] = {0x0e, 0x0e, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00};
uint8_t char_battery_node[] = {0x0e, 0x0a, 0x1b, 0x11, 0x11, 0x11, 0x1f, 0x00};
uint8_t state_backlight = 0x08;

static char tag[] = "LCD Driver";
static uint8_t LCD_addr;
static uint8_t SDA_pin;
static uint8_t SCL_pin;
static uint8_t LCD_cols;
static uint8_t LCD_rows;

static void LCD_writeNibble(uint8_t nibble, uint8_t mode);
static void LCD_pulseEnable(uint8_t nibble);
extern int rssi_LCD;
extern unsigned int number_news; 

static esp_err_t I2C_init(void)
{
  i2c_config_t config = {};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = GPIO_NUM_21;
  config.scl_io_num = GPIO_NUM_22;
  config.master.clk_speed = 100000;

	i2c_param_config(I2C_NUM_0, &config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}

void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows)
{
    LCD_addr = addr;
    SDA_pin = dataPin;
    SCL_pin = clockPin;
    LCD_cols = cols;
    LCD_rows = rows;
    I2C_init();
    vTaskDelay(100 / portTICK_PERIOD_MS );                                 // Initial 40 mSec delay

    // Reset the LCD controller
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // First part of reset sequence
    vTaskDelay(10 /portTICK_PERIOD_MS );                                  // 4.1 mS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // second part of reset sequence
    ets_delay_us(200);                                                  // 100 uS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // Third time's a charm
    LCD_writeNibble(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                // Activate 4-bit mode
    ets_delay_us(80);                                                   // 40 uS delay (min)

    // --- Busy flag now available ---
    // Function Set instruction
    LCD_writeByte(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                  // Set mode, lines, and font
    ets_delay_us(80); 

    // Clear Display instruction
    // LCD_writeByte(LCD_CLEAR, LCD_COMMAND);                              // clear display RAM
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // Clearing memory takes a bit longer
    
    // Entry Mode Set instruction
    LCD_writeByte(LCD_ENTRY_MODE, LCD_COMMAND);                         // Set desired shift characteristics
    ets_delay_us(80); 

    LCD_writeByte(LCD_DISPLAY_ON, LCD_COMMAND);                         // Ensure LCD is set to on
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
    if (row > LCD_rows - 1) {
        // ESP_LOGE(tag, "Cannot write to row %d. Please select a row in the range (0, %d)", row, LCD_rows-1);
        row = LCD_rows - 1;
    }
    uint8_t row_offsets[] = {LCD_LINEONE, LCD_LINETWO, LCD_LINETHREE, LCD_LINEFOUR};
    LCD_writeByte(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]), LCD_COMMAND);
}

void LCD_writeChar(char c)
{
    LCD_writeByte(c, LCD_WRITE);                                        // Write data to DDRAM
}

void LCD_writeStr(char* str)
{
    while (*str) {
        LCD_writeChar(*str++);
    }
}

void LCD_home(void)
{
    LCD_writeByte(LCD_HOME, LCD_COMMAND);
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // This command takes a while to complete
}

void lcd_clear (void)
{
    LCD_writeByte(0x80, LCD_COMMAND);    
	for (int i=0; i<80; i++)
	{
		LCD_writeByte(' ', LCD_WRITE);
	}
}
static void LCD_writeNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | state_backlight;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);   

    LCD_pulseEnable(data);                                              // Clock data into LCD
}

void LCD_setBacklight(bool state)
{
    uint8_t data = state ? LCD_BACKLIGHT : 0x00; // Bật hoặc tắt backlight
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1)); // Địa chỉ LCD
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1)); // Gửi dữ liệu điều khiển đèn
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS)); // Thực thi lệnh I2C
    
    i2c_cmd_link_delete(cmd);
}

void LCD_writeByte(uint8_t data, uint8_t mode)
{
    LCD_writeNibble(data & 0xF0, mode);
    LCD_writeNibble((data << 4) & 0xF0, mode);
}

static void LCD_pulseEnable(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data | LCD_ENABLE, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);  
    ets_delay_us(1);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (data & ~LCD_ENABLE), 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(500);
}

void Lcd_clear_data(void)
{
    LCD_setCursor(5, 0); // vi tri cua battery node
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);

    LCD_setCursor(9, 1); // vi tri cua range
    LCD_writeByte(' ', LCD_WRITE); // xoa 7 o lien tuc
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    
    LCD_setCursor(9, 3); // vi tri cua speed
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
}

void LCD_clear_rssi(void)
{
    LCD_setCursor(0, 3);
    for (int p = 0; p < 20; p++)
        LCD_writeByte(' ', 1); // write_data ' '
}

void LCD_clear_row_3(void)
{
    LCD_setCursor(0, 2);
    for (int p = 0; p < 20; p++)
        LCD_writeByte(' ', 1); // write_data ' '
}

void Lcd_display_rssi(void)
{
    LCD_clear_rssi();
    LCD_setCursor(0, 3);
    LCD_writeStr("Rssi: ");
    lcd_number(rssi_LCD);
}

void LCD_Display_IP_Wifi(void)
{
    char ipChar[20];
    sprintf(ipChar, "IP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    LCD_clear_rssi();
    LCD_setCursor(0, 3);
    LCD_writeStr(ipChar);
}

void Lcd_display_data(void)
{
    if (Node_Data[0].battery <= 0) Node_Data[0].battery = 0;
    else if (Node_Data[0].battery >= 100) Node_Data[0].battery = 100;
    LCD_setCursor(5, 0);           //display battery
    lcd_number(Node_Data[0].battery);
    LCD_writeStr("%");

    // if (Node_Data[0].battery <= BATTERY_THRESHOLD) 
    // {
    //     LCD_setCursor(11, 0);
    //     LCD_writeStr("pin yeu");
    // }
    // else
    // {
    //     // xoa ki tu
    //     LCD_setCursor(11, 0);
    //     for (int p = 0; p <strlen("pin yeu"); p++)
    //         LCD_writeByte(' ', 1); // write_data ' '
    // }            

    LCD_setCursor(9, 1);
    lcd_number_float(Node_Data[0].range);
    // if (Node_Data[0].range >= Node_Data[0].range_threshold)
    // {
    //     LCD_setCursor(2, 3);
    //     LCD_writeStr("Canh bao sat lo!");
    // }
    if (number_news > 1)
    {
        LCD_setCursor(9, 3);
        lcd_number_speed(Node_Data[0].speed);
    }
}

void Lcd_clear_threshold(void)
{
    LCD_setCursor(9, 2); // vi tri cua range threshold
    LCD_writeByte(' ', LCD_WRITE); // xoa 4 o lien tuc
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
    LCD_writeByte(' ', LCD_WRITE);
}

void lcd_modify_threshold(void)
{
    Lcd_clear_threshold();
    LCD_setCursor(9, 2);
    lcd_number(Node_Data[0].range_threshold);
}

void LCD_clear_wifi(void)
{
    LCD_setCursor(12, 0);     
	for (int i=0; i< strlen("wifi:off"); i++)
	{
		LCD_writeByte(' ', LCD_WRITE);
	}
}

void Lcd_write_custom_char(uint8_t col, uint8_t row, uint8_t location, uint8_t* data_bytes)
{
	uint8_t i;
	// We only have 8 locations 0-7 for custom chars
	location &= 0x07; 
	// Set CGRAM address
	LCD_writeByte(SET_CGRAM_ADDRESS | (location << 3), LCD_COMMAND);
	
	// Write 8 bytes custom char pattern
	for (i = 0; i < 8; i++) 
	{
		LCD_writeByte(data_bytes[i], LCD_WRITE);
	}
	LCD_setCursor(col, row);
	LCD_writeByte(location, LCD_WRITE);
}

void LCD_Display_Form(void)
{
    lcd_clear();
    // Lcd_write_custom_char(15, 0, 0, char_battery_gate);
    // Lcd_write_custom_char(0, 0, 1, char_battery_node);
	LCD_setCursor(0, 0);
	LCD_writeStr("Pin: ");

	LCD_setCursor(0, 1);
    LCD_writeStr("Do Gian: ");
    LCD_setCursor(16, 1);
	LCD_writeStr("mm");

    LCD_setCursor(0, 2);
	LCD_writeStr("NGUONG: ");
    LCD_setCursor(9, 2);
    lcd_number(Node_Data[0].range_threshold);
    LCD_setCursor(16, 2);
	LCD_writeStr("mm");

    LCD_setCursor(0, 3);
	LCD_writeStr("Toc do: ");
    LCD_setCursor(16, 3);
	LCD_writeStr("mm/p");

	// if (wifi_status == WIFI_CONNECTED) 
	// {
	// 	LCD_setCursor(0, 3);
	// 	LCD_writeStr("WIFI: Connected");
	// }
	// else if (wifi_status == WIFI_DISCONNECT) 
	// 	{
	// 		LCD_setCursor(0, 3);
	// 		LCD_writeStr("WIFI: No connect");
	// 	}
    // wifi_status_pre = wifi_status;
}

void lcd_number(int x)
{
	char num[8];
	sprintf(num,"%d",x);
	LCD_writeStr(num);
}

void lcd_number_float(float x)
{
	char num[8];
	sprintf(num,"%.1f",x);
	LCD_writeStr(num);
}

void lcd_number_speed(float y)
{
	char num[8];
	sprintf(num,"%.3f",y);
	LCD_writeStr(num);
}