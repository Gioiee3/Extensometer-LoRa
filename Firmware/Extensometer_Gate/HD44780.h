#ifndef _LCD_H_
#define _LCD_H_

extern uint8_t state_backlight;
//define ki tu

void LCD_writeByte(uint8_t data, uint8_t mode);
void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows);
void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_home(void);
void LCD_writeChar(char c);
void LCD_writeStr(char* str); 
void lcd_clear (void);
void Lcd_write_custom_char(uint8_t col, uint8_t row, uint8_t location, uint8_t* data_bytes);
void LCD_Display_Form(void);
void Lcd_clear_data(void);
void Lcd_display_data(void);
void Lcd_clear_threshold(void);
void lcd_modify_threshold(void);
void LCD_clear_wifi(void);
void LCD_clear_row_3(void);
void lcd_number(int x);
void lcd_number_float(float x);
void LCD_setBacklight(bool state);
void LCD_clear_rssi(void);
void Lcd_display_rssi(void);
void LCD_Display_IP_Wifi(void);
void lcd_number_speed(float y);
#endif