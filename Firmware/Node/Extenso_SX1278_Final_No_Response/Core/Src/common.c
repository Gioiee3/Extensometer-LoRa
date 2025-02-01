/**
 * @file common.c
 * @author Kyuubi0323 (nguyenvankhoi8d@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"
//#include "cmsis_os.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
//#include "cJSON.h"
#include "sx1278.h"
#include "common.h"

static const char *TAG = "COMMON";

void LOG(const char *TAG, char *data)
{
	char data_log[100] = {0};
	sprintf(data_log, "%s: %s\n", TAG, data);
	HAL_UART_Transmit(&huart2, (uint8_t*)data_log, strlen(data_log), 100);
}



//esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj)
//{
//    cJSON *root = cJSON_Parse(mqtt_data);
//    if (root == NULL)
//        return ESP_FAIL;
//    cJSON *cur_elem = NULL;
//    cJSON_ArrayForEach(cur_elem, root)
//    {
//        if (cur_elem->string)
//        {
//            const char *cur_str = cur_elem->string;
//            if (strcmp(cur_str, "action") == 0)
//                memcpy(mqtt_obj->action, cur_elem->valuestring, strlen(cur_elem->valuestring) + 1);
//        }
//    }
//    cJSON_Delete(root);
//    return ESP_OK;
//}
