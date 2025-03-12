/**
 * @file button.c
 * @author Dinh Gioi- K65 -HUST
 * @brief 
 * @date 2024-12-09
 */
#include <Arduino.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include "esp_http_client.h"
#include "esp_attr.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "driver/adc.h"
#include "soc/soc_caps.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"
#include "esp_adc_cal.h"

#include "button.h"
#include "sx1278.h"
#include "HD44780.h"

// char ssid[50] = {0};
// char pwd[50] = {0};

static const char *TAG = "BUTTON";

extern uint8_t wifi_status;
static uint8_t wifi_status_pre;
static esp_adc_cal_characteristics_t adc1_chars;

uint32_t current_Time = 0, previous_Time = 0;
int count = 0, flag_LCD_rssi = 0;

void button_init(void)
{
    // esp_rom_gpio_pad_select_gpio(LED_TFT_PIN);
    // gpio_reset_pin(LED_TFT_PIN);
    // gpio_set_direction(LED_TFT_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(Light_Warning);
    gpio_reset_pin(Light_Warning);
    gpio_set_direction(Light_Warning, GPIO_MODE_OUTPUT);    

    gpio_config_t io1_config = {
        .pin_bit_mask = BIT64(SW1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io1_config);

    gpio_config_t io2_config = {
        .pin_bit_mask = BIT64(SW_LED),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io2_config);

    gpio_config_t io3_config = {
        .pin_bit_mask = BIT64(SW3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io3_config);
}

void Baterry_gate_init_adc(void)
{
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, static_cast<adc_bits_width_t>(ADC_WIDTH_BIT_DEFAULT), 0, &adc1_chars);
    adc1_config_width((adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
}

void Display_battery_gate(void)
{
	uint32_t adc_value = 0;
    float battery_gate = 0;
    int battery_phantram = 0;
    for (int id = 0; id <15; id ++)
    {
        adc_value +=adc1_get_raw(ADC1_CHANNEL_6);
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
    
    adc_value = adc_value/15;
    battery_gate = (float)(adc_value * 3.3 * 4 / 4095.0 + 0.88);

    Serial.printf("[%s] ADC Value: %.2f\n", TAG, battery_gate);

    battery_phantram = (int)(100 * (float)(battery_gate - BATTERY_MIN)/(BATTERY_MAX - BATTERY_MIN));
    if (battery_phantram <= 0) battery_phantram = 0;
    else if (battery_phantram >= 100) battery_phantram = 100;

    LCD_clear_rssi();
    LCD_setCursor(0, 3);
    LCD_writeStr("Gate: ");
    lcd_number_float(battery_gate);
    // LCD_writeStr("%");
}

void button_task(void *param)
{
    button_t button1 = {
        .time_down = 0,
        .pin = SW1,
        .time_up = 0,
        .deltaT = 0,
        .click_cnt = 0,
        .time_stamp = 0,
    };
    button_t button_led = {
        .time_down = 0,
        .pin = SW_LED,
        .time_up = 0,
        .deltaT = 0,
        .click_cnt = 0,
        .time_stamp = 0,
    };
    button_t button3 = {
        .time_down = 0,
        .pin = SW3,
        .time_up = 0,
        .deltaT = 0,
        .click_cnt = 0,
        .time_stamp = 0,
    };
    uint8_t state_buttons = 0;

    button_init();
    Baterry_gate_init_adc();
    // LCD_init(0x27,SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
	// lcd_clear();
	// LCD_setCursor(0, 1);
	// LCD_writeStr("----EXTENSOMETER----");

	// wifi_init();
    // sprintf(ssid, "Laptop"); //"TP-Link_5AB6"
    // sprintf(pwd, "888888888"); //"11173793"
    // wifi_config_t wifi_config;
    // bzero(&wifi_config, sizeof(wifi_config_t));
    // memcpy(wifi_config.sta.ssid, ssid, strlen(ssid));
    // memcpy(wifi_config.sta.password, pwd, strlen(pwd));    
    // wifi_sta(wifi_config, WIFI_MODE_STA);

    // vTaskDelay(2000 / portTICK_PERIOD_MS);  
    LCD_Display_Form();

    LCD_clear_wifi();
    if (wifi_status ==  WIFI_CONNECTED) 
    {
        LCD_setCursor(12, 0);    
        LCD_writeStr("WIFI:bat");
    }
    else
    {
        LCD_setCursor(12, 0);
        LCD_writeStr("WIFI:tat");
    }
    wifi_status_pre = wifi_status;   
    while (1)
    {   
        current_Time = millis();
        if ((current_Time - previous_Time > 180000) && (flag_LCD_rssi == 3))
        {
            Display_battery_gate();
            previous_Time = current_Time;
        }

        if (flag_LCD == 1)
        {
            Lcd_clear_data();
            Lcd_display_data();
        
            if (flag_LCD_rssi == 1) Lcd_display_rssi();
            flag_LCD = 0;
        }

        // // check wifi
        if (wifi_status != wifi_status_pre)
        {
            LCD_clear_wifi();
            if (wifi_status ==  WIFI_CONNECTED) 
            {
                LCD_setCursor(12, 0);    
                LCD_writeStr("WIFI:bat");
            }
            else
            {
                LCD_setCursor(12, 0);
                LCD_writeStr("WIFI:tat");
            }
            wifi_status_pre = wifi_status;        
        }

        // bat tat light, reset
        if (gpio_get_level(SW1) == BUTTON_TRIGGER)
        {
            if (button1.time_up == 0)    button1.time_up = (uint32_t)(millis());
            else      
            {
                button1.deltaT = (uint32_t)(millis()) - button1.time_up;
                if ((button1.deltaT > TIME_HOLD) && (button1.deltaT < TIME_RESTART))
                {
                    gpio_set_level(Light_Warning, Light_OFF);
                    // LCD_setCursor(2, 3);
                    // for (int p = 0; p <strlen("Canh bao sat lo!"); p++)
                    //     LCD_writeByte(' ', 1);
                    Serial.printf("[%s] Turn off light warning\n", TAG);
                }
                else if (button1.deltaT >= TIME_RESTART)
                    {
                        lcd_clear();
            			LCD_setCursor(0, 1);
		            	LCD_writeStr("------RESTART-------");
                        vTaskDelay(TIME_DISPLAY_RESTART / portTICK_PERIOD_MS);
                        esp_restart();
                    }
            }
        }
        else if (gpio_get_level(SW1) == BUTTON_NOT_TRIGGER && button1.time_up != 0 && button1.deltaT > TIME_CLICK_MIN)
            {
                // ESP_LOGI(TAG, "DeltaT: %.0ld", button1.deltaT);
                // neu nhieu hon 9s thi bat rssi
                if (button1.deltaT >= TIME_ON_RSSI)
                {
                    flag_LCD_rssi ++;
                    if (flag_LCD_rssi > 3) flag_LCD_rssi = 0;
                    if (flag_LCD_rssi == 1) Lcd_display_rssi();
                    else if (flag_LCD_rssi == 2) LCD_Display_IP_Wifi();
                         else if (flag_LCD_rssi == 3) Display_battery_gate();
                              else LCD_clear_rssi();
                    Serial.printf("[%s] Button RSSI\n", TAG);
                }                
                else if ((button1.deltaT > TIME_HOLD) && (button1.deltaT < TIME_ON_RSSI))
                    {
                        gpio_set_level(Light_Warning, Light_OFF);
                        Serial.printf("[%s] Turn off light warning\n", TAG);
                    }
                    else if (button1.deltaT > TIME_CLICK_MIN && button1.deltaT < TIME_CLICK_MAX)
                        {
                            Serial.printf("[%s] Button modify threshold !\n", TAG);
                            state_backlight = 8 - state_backlight;
                            LCD_setBacklight(state_backlight);
                            state_buttons = 1 - state_buttons;
                        }                                
                button1.time_up = 0;
                button1.time_down = 0;
                button1.deltaT = 0;            
            }
            else // if (gpio_get_level(SW1) == BUTTON_NOT_TRIGGER && (uint32_t)(xTaskGetTickCount() / portTICK_PERIOD_MS) - button.time_stamp > TIME_RESET && button.time_stamp != 0)
            {
                button1.time_up = 0;
                button1.time_down = 0;
                button1.deltaT = 0;
                button1.click_cnt = 0;
                button1.time_stamp = 0;
            }
        // nguong giam
        if (gpio_get_level(SW3) == BUTTON_TRIGGER)
        {
            if (button3.time_up == 0)   button3.time_up = (uint32_t)(millis());
            else
            {                       
                button3.deltaT = (uint32_t)(millis()) - button3.time_up;
                if (button3.deltaT > TIME_BUTTON_REPEAT)
                {
                    if (Node_Data[0].range_threshold > 0) Node_Data[0].range_threshold--;
                    else Node_Data[0].range_threshold = RANGE_THRES_MAX;
                    // ESP_LOGI(TAG, "click down repeat!!!");
                    Serial.printf("[%s] click down repeat!!!\n", TAG);
                    lcd_modify_threshold();
                    // vTaskDelay(3 / portTICK_PERIOD_MS);
                }
            }
        }
        else if (gpio_get_level(SW3) == BUTTON_NOT_TRIGGER && button3.time_up != 0 && button3.deltaT > TIME_CLICK_MIN)
            {
                // nut nhan giam
                if (Node_Data[0].range_threshold > 0) Node_Data[0].range_threshold--;
                else Node_Data[0].range_threshold = RANGE_THRES_MAX;
                lcd_modify_threshold();
                Serial.printf("[%s] Theshold: %d\n", TAG, Node_Data[0].range_threshold);
                // ESP_LOGI(TAG, "Theshold: %d",Node_Data[0].range_threshold);
                Serial.printf("[%s] click down\n", TAG);
                // ESP_LOGI(TAG, "click down");
                Serial.printf("[%s] DeltaT: %.0ld\n", TAG, button3.deltaT);
                // ESP_LOGI(TAG, "DeltaT: %.0ld", button3.deltaT);                
                button3.time_up = 0;
                button3.time_down = 0;
                button3.deltaT = 0;            
            }
            else // if (gpio_get_level(SW1) == BUTTON_NOT_TRIGGER && (uint32_t)(xTaskGetTickCount() / portTICK_PERIOD_MS) - button.time_stamp > TIME_RESET && button.time_stamp != 0)
            {
                button3.time_up = 0;
                button3.time_down = 0;
                button3.deltaT = 0;
                button3.click_cnt = 0;
                button3.time_stamp = 0;
            }
        // nguong tang
        if (gpio_get_level(SW_LED) == BUTTON_TRIGGER)
        {
            if (button_led.time_up == 0)   button_led.time_up = (uint32_t)(millis());
            else
            {
                button_led.deltaT = (uint32_t)(millis()) - button_led.time_up;
                if (button_led.deltaT > TIME_BUTTON_REPEAT)
                {
                    if (Node_Data[0].range_threshold < RANGE_THRES_MAX) Node_Data[0].range_threshold ++;
                    else Node_Data[0].range_threshold = 0;
                    // ESP_LOGI(TAG, "click up repeat!!!");
                    Serial.printf("[%s] click up repeat!!!\n", TAG);
                    lcd_modify_threshold();
                    // vTaskDelay(4 / portTICK_PERIOD_MS);
                }
            }
        }
        else if (gpio_get_level(SW_LED) == BUTTON_NOT_TRIGGER && button_led.time_up != 0 && button_led.deltaT > TIME_CLICK_MIN)
            {
                // nut nhan tang
                if (Node_Data[0].range_threshold < RANGE_THRES_MAX) Node_Data[0].range_threshold ++;
                else Node_Data[0].range_threshold = 0;
                lcd_modify_threshold();
                Serial.printf("[%s] Theshold: %d\n", TAG, Node_Data[0].range_threshold);
                Serial.printf("[%s] click up\n", TAG);
                Serial.printf("[%s] DeltaT: %.0ld\n", TAG, button_led.deltaT);
                // ESP_LOGI(TAG, "Theshold: %d",Node_Data[0].range_threshold);
                // ESP_LOGI(TAG, "click up");
                // ESP_LOGI(TAG, "DeltaT: %.0ld", button_led.deltaT);
                button_led.time_up = 0;
                button_led.time_down = 0;
                button_led.deltaT = 0;            
            }
            else 
           {
                button_led.time_up = 0;
                button_led.time_down = 0;
                button_led.deltaT = 0;
                button_led.click_cnt = 0;
                button_led.time_stamp = 0;
            }
        vTaskDelay(9 / portTICK_PERIOD_MS);
    }
}

// void button_task(void *param)
// {
//     button_t button = {
//         .pin = BUTTON_CONFIG_PIN,
//         .time_down = 0,
//         .time_up = 0,
//         .deltaT = 0,
//         .click_cnt = 0,
//         .time_stamp = 0,
//     };

//     gpio_config_t config_io;
//     config_io.intr_type = GPIO_INTR_DISABLE;
//     config_io.mode = GPIO_MODE_INPUT;
//     config_io.pull_up_en = GPIO_PULLUP_ONLY;
//     config_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
//     config_io.pin_bit_mask = (1ULL << button.pin);
//     gpio_config(&config_io);

//     while (1)
//     {
//         if (gpio_get_level(button.pin) == BUTTON_TRIGGER)
//         {
//             if (button.time_up == 0)
//                 button.time_up = (uint32_t)(xTaskGetTickCount() / portTICK_PERIOD_MS);
//             else
//             {
//                 button.deltaT = (uint32_t)(xTaskGetTickCount() / portTICK_PERIOD_MS) - button.time_up;
//             }
//         }
//         else if (gpio_get_level(button.pin) == BUTTON_NOT_TRIGGER && button.time_up != 0 && button.deltaT > TIME_CLICK_MIN)
//             {
//                 button.time_down = (uint32_t)(xTaskGetTickCount() / portTICK_PERIOD_MS);
//                 button.deltaT = button.time_down - button.time_up;
//                 ESP_LOGI(TAG, "DeltaT: %d", button.deltaT);
//                 if (button.deltaT > TIME_HOLD)
//                 {
//                     ESP_LOGI(TAG, "Trigger Smartconfig");
//                     gateway_mode_flag = SMARTCONFIG_MODE;
//                     esp_restart();
//                 }
//                 else if (button.deltaT > TIME_CLICK_MIN && button.deltaT < TIME_CLICK_MAX)
//                     {
//                         button.click_cnt++;
//                         ESP_LOGI(TAG, "Button counter: %d", button.click_cnt);
//                         if (button.click_cnt == 5)
//                         {
//                             ESP_LOGI(TAG, "Trigger SoftAP");
//                             gateway_mode_flag = WIFI_SOFTAP_MODE;
//                             esp_restart();
//                         }
//                         else
//                         {
//                             button.time_stamp = button.time_up;
//                             button.time_up = 0;
//                             button.time_down = 0;
//                             button.deltaT = 0;
//                         }
//                     }
//             }
//             else if (gpio_get_level(button.pin) == BUTTON_NOT_TRIGGER && (uint32_t)(xTaskGetTickCount() / portTICK_PERIOD_MS) - button.time_stamp > TIME_RESET && button.time_stamp != 0)
//                 {
//                     button.time_up = 0;
//                     button.time_down = 0;
//                     button.deltaT = 0;
//                     button.click_cnt = 0;
//                     button.time_stamp = 0;
//                 }
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }
