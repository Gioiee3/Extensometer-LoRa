/**
 * @file button.c
 * @author Dinh Gioi- K65 -HUST
 * @brief 
 * @date 2024-12-09
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#define SW1             GPIO_NUM_36
#define SW_LED          GPIO_NUM_39
#define SW3             GPIO_NUM_35
#define LED_TFT_PIN     GPIO_NUM_27
#define l               GPIO_NUM_27
#define Light_Warning   GPIO_NUM_12
#define Battery         GPIO_NUM_34

#define Light_ON        1
#define Light_OFF       0

// #define BUTTON_CONFIG_PIN GPIO_NUM_23
// #define TIME_HOLD (3000 / portTICK_PERIOD_MS)
#define TIME_HOLD       3000           // 3s
#define TIME_RESTART    10000           // 10s
// #define TIME_CLICK_MIN (5 / portTICK_PERIOD_MS)
// #define TIME_CLICK_MAX (1000 / portTICK_PERIOD_MS)
#define TIME_CLICK_MIN  30        // 30 ms
#define TIME_CLICK_MAX  1000
#define TIME_RESET      (1000 / portTICK_PERIOD_MS)
#define TIME_BUTTON_REPEAT      600   // 600 ms 
#define TIME_DISPLAY_RESTART    2000    // 2s
#define TIME_ON_RSSI            8000

#define BUTTON_MODIFY_THRESHOLD    1
#define BUTTON_NO_MODIFY_THRESHOLD 0
//define Battery
#define BATTERY_MIN             7.2
#define BATTERY_MAX             8.4
#define BATTERY_THRESHOLD       20

#define BUTTON_TRIGGER          0
#define BUTTON_NOT_TRIGGER      1

#define RANGE_THRES_MAX 1000

#define WIFI_CONNECTED 3
void button_task(void *param);
void button_init(void);
void l_task(void *param);

typedef struct
{
    uint32_t time_down;
    uint8_t pin;
    uint32_t time_up;
    uint32_t deltaT;
    uint8_t click_cnt;
    uint32_t time_stamp;
} button_t;

#endif

