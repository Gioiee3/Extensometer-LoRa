/**
 * @file common.h
 * @author Kyuubi0323 (nguyenvankhoi8d@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define NORMAL_MOD 11
#define SMARTCONFIG_MODE 20
#define WIFI_SOFTAP_MODE 31

typedef enum
{
    LOCAL,
    NORMAL,
    CONFIG,
} gateway_mode_t;

typedef enum
{
    NOT_STATE,
    SMARTCONFIG,
    FOTA,
    WIFI_SOFTAP,
} gateway_cfg_mode_t;

typedef struct
{
    char action[15];
} mqtt_obj_t;

void LOG(const char *TAG, char *data);
//esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj);
#endif /* INC_COMMON_H_ */
