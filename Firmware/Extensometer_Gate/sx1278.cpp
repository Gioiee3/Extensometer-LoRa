/**
 * @file sx1278.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-01-29
 *
 * @copyright Copyright (c) 2023
 * 
 *  Modify: Nov 30, 2024 
 *  Author: Dinh Gioi - HUST - K65
 *
 */
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include "esp_attr.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "sx1278.h"
#include "stdio.h"
#include "string.h"
#include "Queue.h"
#include "button.h"

static const char *TAG = "LORA GATEWAY";
static spi_device_handle_t spi_handle;

EventGroupHandle_t sx1278_evt_group;
queue_t xQueue1;
// extern esp_mqtt_client_handle_t client;

sx1278_node_slot_t Node_Data[3];
double val_sen, val_lm, val_bat, val_lv, Node0_range = 0;
int flag_MQTT = 0, flag_LCD = 0;

unsigned int number_news = 0; 
int rssi_LCD = -1;
uint32_t num = 0, num_par = 0;
uint64_t time_recv_pre = 0;

//dao nguoc chuoi
void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    if (x == 0)
        str[i++] = '0';

    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}
//convert float to string and select number of digits after the comma
void ftoa(double n, char *res, int afterpoint)
{
    int ipart = (int)n;
    double fpart = n - (double)ipart;
    int i = intToStr(ipart, res, 0);
    if (afterpoint != 0)
    {
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t pin = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pin == SX1278_DIO0_PIN) xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO0_BIT, &xHigherPriorityTaskWoken);
    
    // if (((pin == SW1) || (pin == SW3) || (pin == SW_LED)) && ((current_Time - previous_Time) > 30) ) 
    // {
    //     xQueueSendFromISR(interputQueue, &pin, NULL);
    //     previous_Time = current_Time;
    // }
}

void sx1278_gpio_init(void)
{
    gpio_pad_select_gpio(SX1278_TXEN_PIN);
    gpio_reset_pin(SX1278_TXEN_PIN);
    gpio_set_direction(SX1278_TXEN_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(SX1278_RXEN_PIN);
    gpio_reset_pin(SX1278_RXEN_PIN);
    gpio_set_direction(SX1278_RXEN_PIN, GPIO_MODE_OUTPUT);    

    gpio_set_level(SX1278_TXEN_PIN, 0);
    gpio_set_level(SX1278_RXEN_PIN, 0);

    gpio_config_t io0_config = {
        .pin_bit_mask = BIT64(SX1278_DIO0_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io0_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SX1278_DIO0_PIN, gpio_isr_handler, (void *)SX1278_DIO0_PIN);
}

void sx1278_spi_init(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = SX1278_MOSI_PIN,
        .miso_io_num = SX1278_MISO_PIN,
        .sclk_io_num = SX1278_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0, //kich thuoc toi da cua du lieu duoc truyen
    };

    assert(spi_bus_initialize(VSPI_HOST, &bus_config, 0) == ESP_OK);

    spi_device_interface_config_t device_config = {
        .mode = 0,
        .clock_speed_hz = 4000000,
        .spics_io_num = SX1278_NSS_PIN,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
    };
    assert(spi_bus_add_device(VSPI_HOST, &device_config, &spi_handle) == ESP_OK);
}

uint8_t sx1278_read_reg(uint8_t reg)
{
    uint8_t data_send[2] = {0x00 | reg, 0xFF}; // A wnr bit, which is 1 for write and 0 for read access
    uint8_t data_recv[2] = {0};
    spi_transaction_t sx1278_recv = {
        .flags = 0,
        .length = 8 * sizeof(data_send),
        .tx_buffer = (void *)data_send,
        .rx_buffer = (void *)data_recv,
    };
    gpio_set_level(SX1278_NSS_PIN, 0);
    spi_device_transmit(spi_handle, &sx1278_recv);
    gpio_set_level(SX1278_NSS_PIN, 1);
    return data_recv[1];
}

void sx1278_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data_send[2] = {0x80 | reg, val}; // A wnr bit, which is 1 for write and 0 for read access
    uint8_t data_recv[2] = {0};
    spi_transaction_t sx1278_send = {
        .flags = 0,
        .length = 8 * sizeof(data_send),
        .tx_buffer = (void *)data_send,
        .rx_buffer = (void *)data_recv,
    };
    gpio_set_level(SX1278_NSS_PIN, 0);
    spi_device_transmit(spi_handle, &sx1278_send);
    gpio_set_level(SX1278_NSS_PIN, 1);
}

void sx1278_reset(void)
{
    gpio_set_level(SX1278_RST_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(SX1278_RST_PIN, 1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void sx1278_sleep(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void sx1278_standby(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sx1278_rx_contiuous(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void sx1278_rx_single(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
}

void sx1278_tx(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void sx1278_set_irq(uint8_t val)
{
    sx1278_write_reg(REG_DIO_MAPPING_1, val);
}

void sx1278_cad(void)
{
    sx1278_set_irq(0x80);
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void sx1278_set_tx_power(uint8_t output_power)
{
    // if (output_power > 15)
    // {
    //     ESP_LOGE(TAG, "Invalid output power");
    //     return;
    // }
    // PA output pin: PA_BOOST pin
    sx1278_write_reg(REG_PA_CONFIG, PA_BOOST | output_power);
}

void sx1278_set_LNA_gain(uint8_t gain)
{
    if (gain > 6)
    {
        // ESP_LOGW(TAG, "Invalid gain");
        return;
    }

    if (gain == 0)
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x04);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x00);
        sx1278_write_reg(REG_LNA, sx1278_read_reg(REG_LNA) | (gain << 5));
    }
}

void sx1278_set_freq(uint64_t freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    sx1278_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1278_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1278_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void sx1278_set_bandwidth(long band)
{
    int bw;
    if (band <= 7.8E3)
        bw = 0;
    else if (band <= 10.4E3)
        bw = 1;
    else if (band <= 15.6E3)
        bw = 2;
    else if (band <= 20.8E3)
        bw = 3;
    else if (band <= 31.25E3)
        bw = 4;
    else if (band <= 41.7E3)
        bw = 5;
    else if (band <= 62.5E3)
        bw = 6;
    else if (band <= 125E3)
        bw = 7;
    else if (band <= 250E3)
        bw = 8;
    else
        bw = 9;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void sx1278_set_sf(uint8_t sf)
{
    if (sf < 6 || sf > 12)
    {
        // ESP_LOGE(TAG, "Invalid spreading factor");
        Serial.printf("[%s] Invalid spreading factor\n", TAG);
        return;
    }

    if (sf == 6)
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
    }
    sx1278_write_reg(REG_MODEM_CONFIG_2, (sx1278_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void sx1278_set_cr(uint8_t cr)
{
    if (cr < 5 || cr > 8)
    {
        // ESP_LOGE(TAG, "Invalid coding rate");
        Serial.printf("[%s] Invalid coding rate\n", TAG);
        return;
    }

    cr = cr - 4;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void sx1278_set_header(bool en, uint32_t size)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) | 0x01);
        sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    }
}

void sx1278_set_crc(bool en)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) | 0x04);
    else
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void sx1278_set_preamble(int len)
{
    sx1278_write_reg(REG_PREAMBLE_MSB, (uint8_t)(len >> 8));
    sx1278_write_reg(REG_PREAMBLE_LSB, (uint8_t)(len >> 0));
}

int sx1278_get_rssi(void)
{
    return (sx1278_read_reg(REG_PKT_RSSI_VALUE) - 164);
}

float sx1278_get_snr(void)
{
    return ((int8_t)sx1278_read_reg(REG_PKT_SNR_VALUE) * 0.25);
}

void sx1278_init(void)
{
    sx1278_reset();
    uint8_t ver = sx1278_read_reg(REG_VERSION);
    // ESP_LOGI(TAG, "SX1278 version: 0x%02x", ver);
    Serial.printf("[%s] SX1278 version: 0x%02x\n", TAG, ver);
    sx1278_sleep();
    sx1278_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    sx1278_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1278_set_LNA_gain(0);
    sx1278_set_tx_power(13); // 13dBm
    sx1278_set_freq(433E6);
    sx1278_set_bandwidth(125E3); //250
    sx1278_set_sf(11U);
    sx1278_set_cr(5U);
    sx1278_set_preamble(12);
    sx1278_set_header(true, 0);
    sx1278_set_crc(true);
    // sx1278_set_irq(0x00);
    sx1278_standby();
}

void sx1278_send_data(uint8_t *data_send, int size)
{
    sx1278_standby();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, 0);
    sx1278_write_reg(REG_PAYLOAD_LENGTH, 0);
    for (int index = 0; index < size; index++)
    {
        sx1278_write_reg(REG_FIFO, data_send[index]);
    }
    sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    gpio_set_level(SX1278_RXEN_PIN, 0);
    gpio_set_level(SX1278_TXEN_PIN, 1);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    // Start transmission and wait for conclusion
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    while (!(sx1278_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK))
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);
    // sx1278_sleep();
    // gpio_set_level(SX1278_TXEN_PIN, 0);
    // gpio_set_level(SX1278_RXEN_PIN, 0);
}

void sx1278_start_recv_data(void)
{
    sx1278_standby();    
    sx1278_set_irq(0x00);
    gpio_set_level(SX1278_TXEN_PIN, 0);
    gpio_set_level(SX1278_RXEN_PIN, 1);
    // sx1278_write_reg(REG_IRQ_FLAGS, sx1278_read_reg(REG_IRQ_FLAGS));
    sx1278_rx_contiuous();
}

sx1278_err_t sx1278_recv_data(uint8_t *data_recv, uint32_t *len, int *rssi, float *snr, bool isStayinRX)
{
    memset((char *)data_recv, '\0', strlen((char *)data_recv));
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);

    if (!(irq & IRQ_RX_DONE_MASK))
    {
        // ESP_LOGI(TAG, "Invalid RxDone Interrupt");
        Serial.printf("[%s] Invalid RxDone Interrupt\n", TAG);
        return SX1278_INVALID_RX_DONE;
    }

    if (!(irq & IRQ_VALID_HEADER_MASK))
    {
        // ESP_LOGI(TAG, "Invalid Header Interrupt");
        Serial.printf("[%s] Invalid Header Interrupt\n", TAG);
        return SX1278_INVALID_HEADER;
    }

    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
    {
        // ESP_LOGI(TAG, "Payload Crc Error Interrupt");
        Serial.printf("[%s] Payload Crc Error Interrupt\n", TAG);
        return SX1278_PAYLOAD_CRC_ERROR;
    }

    *len = sx1278_read_reg(REG_RX_NB_BYTES);
    *rssi = sx1278_get_rssi();
    *snr = sx1278_get_snr();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, sx1278_read_reg(REG_FIFO_RX_CURRENT_ADDR));
    for (int index = 0; index < *len; index++)
    {
        data_recv[index] = sx1278_read_reg(REG_FIFO);
    }
    if (isStayinRX == false)
        sx1278_standby();
    return SX1278_OK;
}

int get_random_value(int min, int max)
{
    int random;
    if (max <= min)
    {
        // ESP_LOGE(TAG, "Range error");
        return 0;
    }
    return random = min + rand() % (max + 1 - min);
}

uint8_t get_crc_value(uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

bool listen_before_talk(void)
{
    sx1278_standby();
    EventBits_t evt_bits;
    uint8_t irq;
    uint32_t timeout = millis();
    // is_LoRa_processing = false;
    while ((millis()*10 - timeout) <= 4000)
    {
        xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT);
        sx1278_cad();
        evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, 1000);
        if ((evt_bits & SX1278_DIO0_BIT) != 0U)
        {
            // logPC("LoRa - CAD Done\tTook: %i ms\t", HAL_GetTick() - timeout);
            irq = sx1278_read_reg(REG_IRQ_FLAGS);
            sx1278_write_reg(REG_IRQ_FLAGS, irq);
            if ((irq & 0x01) != 0U)
            {
                // logPC("LoRa - CAD Detected\t");
                // HAL_Delay(get_random_value(0, 50));
            }
            else
            {
                // logPC("LoRa - CAD Clear\t");
                sx1278_set_irq(0x00);
                sx1278_standby();
                return true;
            }
        }
    }
    irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);
    sx1278_set_irq(0x00);
    sx1278_standby();
    // logPC("Listen FAIL...\t");
    return false;
}

bool is_Node_ID_inNetwork(uint16_t Node_ID)
{
    Node_Table[0].Link.Node_ID = NODE_ID_1;
    Node_Table[1].Link.Node_ID = NODE_ID_2;
    Node_Table[2].Link.Node_ID = NODE_ID_3;
    for (int i = 0; i < MAX_NODE; i++)
    {
        if (Node_Table[i].Link.Node_ID == Node_ID)
            return true;
    }
    return false;
}

uint8_t sizeof_node()
{
  for(uint8_t i=0 ; i< MAX_NODE; i++)
  {
    if( Node_Table[i].Link.Node_ID == 0)
    {
        return (i);
    }
  }
  return MAX_NODE;
}

void LoRa_Packet_Parser(const uint8_t *data, uint32_t len)
{
    switch ((uint16_t)((data[0] << 8) & 0xFF00) | (uint16_t)(data[1] & 0x00FF))
    {
        // ban tin ket noi
        case PACKET_ID_0:
        {
            if (len != sizeof(Link_Packet_t)) Serial.printf("[%s] Error\n", TAG);
            // ESP_LOGI(TAG, "Receive Link Packet!!");
            Serial.printf("[%s] Receive Link Packet!!\n", TAG);
            number_news++;
            // ESP_LOGI(TAG,"Number: %d \twith time: %.1f", number_news, xTaskGetTickCount()/100.0);
            Serial.printf("[%s] Number: %d \twith time: %.1lf\n", TAG, number_news, millis()/1000.0);

            Link_Struct_t link_payload;
            memcpy((uint8_t *)&link_payload, (uint8_t *)&data[2], sizeof(Link_Struct_t)); // copy all data for link_payload
            // ESP_LOGI(TAG,"Node ID: %02X",link_payload.Node_ID);
            Serial.printf("[%s] Node ID: %02X\n", TAG, link_payload.Node_ID);
            
            if (is_Node_ID_inNetwork(link_payload.Node_ID) == true)
            {
                ResponsePacket_t resp = {PACKET_ID_2, link_payload.Node_ID, NORMAL_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK)};
                if (listen_before_talk() == false)
                {
                    sx1278_start_recv_data();
                    break;
                }
                sx1278_send_data((uint8_t*)&resp, sizeof(ResponsePacket_t));
                // ESP_LOGI(TAG, "Sent respone!\n");
                Serial.printf("[%s] Send respone!\n", TAG);
            }
            else if ( Check_Queue(&xQueue1, link_payload.Node_ID) == true )
                {
                        // Node_Table[sizeof_node()].Link.Node_ID = link_payload.Node_ID;
                        // Pop_Queue(&xQueue1);
                        // ResponsePacket_t resp = {PACKET_ID_2, link_payload.Node_ID, NORMAL_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK)};
                        // if (listen_before_talk() == false)
                        // {
                        //     sx1278_start_recv_data();
                        //     break;
                        // }
                        // sx1278_send_data(&resp, sizeof(ResponsePacket_t));
                        // ESP_LOGI(TAG, "Done2!\n");
                }
                else
                {
                    // Push_Queue(&xQueue1, static_cast<EventState>(1), link_payload.Node_ID, 0);
                    // char data3[40];
                    // if (link_payload.Node_ID == NODE_ID_1)
                    // {
                    //     sprintf(data3,"{\"NODE 1 \":Connecting}");
                    //     esp_mqtt_client_publish(client, TOPIC , data3, 0, 1, 0);
                    // }
                    // else if (link_payload.Node_ID == NODE_ID_2)
                    //     {
                    //         sprintf(data3,"{\"NODE 2 \":Connecting}");
                    //         esp_mqtt_client_publish(client, TOPIC , data3, 0, 1, 0);
                    //     }
                    //     else if (link_payload.Node_ID == NODE_ID_3)
                    //         {
                    //             sprintf(data3,"{\"NODE 3 \":Connecting}");
                    //             esp_mqtt_client_publish(client, TOPIC , data3, 0, 1, 0);
                    //         }
                    //         else if (link_payload.Node_ID == 0xCAAF)
                    //             {
                    //                 sprintf(data3,"{\"node new \":0xCAAF}");
                    //                 esp_mqtt_client_publish(client, TOPIC , data3, 0, 1, 0);
                    //             }
                }
            sx1278_start_recv_data();
            break;
        }

        // ban tin du lieu    
        case PACKET_ID_1:
        {
            if (len != sizeof(Data_Packet_t)) break;
            // ESP_LOGI(TAG, "Receive Data Packet!\t");
            Serial.printf("[%s] Receive Data Packet!!\n", TAG);
            number_news++;
            Serial.printf("[%s] Number: %d \twith time: %.1lf\n", TAG, number_news, millis()/1000.0);
            // ESP_LOGI(TAG,"Number: %d \twith time: %.1f s", number_news, xTaskGetTickCount()/100.0);

            Data_Struct_t data_recv;
            memcpy((uint8_t *)&data_recv, (uint8_t *)&data[2], sizeof(Data_Struct_t));

            if (is_Node_ID_inNetwork(data_recv.Link.Node_ID) == true)
            {             
                if (data_recv.Link.Node_ID == NODE_ID_1)
                {
                    Node_Data[0].node_id = (int)data_recv.Link.Node_ID;
                    // ESP_LOGI(TAG,"Node_1 ID: %02X",data_recv.Link.Node_ID);
                    Serial.printf("[%s] Node_1 ID: %02X\n", TAG, data_recv.Link.Node_ID);
                    
                    Node_Data[0].period  = (int)data_recv.Link.Node_Period;
                    // ESP_LOGI(TAG,"Period: %d",data_recv.Link.Node_Period);

                    if (data_recv.Link.Node_Status == 1) Serial.printf("[%s] Status: WAKEUP MODE!\n", TAG);
                    else if (data_recv.Link.Node_Status == 2) Serial.printf("[%s] Status: LINK MODE!\n", TAG);
                         else if (data_recv.Link.Node_Status == 4) Serial.printf("[%s] Status: NORMAL MODE!\n", TAG);
                              else if (data_recv.Link.Node_Status == 8) Serial.printf("[%s] Status: RETRY MODE!\n", TAG);
                                   else if (data_recv.Link.Node_Status == 16) Serial.printf("[%s] Status: SHUTDOWN MODE!\n", TAG);

                    val_bat = (double)(((double)(data_recv.Link.Node_Battery_Voltage))/100.0);
                    val_lm = (double)(((double)(data_recv.Node_LM35_Temperature))/10.0);
                    // caculation battery
                    // ESP_LOGI(TAG,"Battery: %.2f", val_bat);
                    Serial.printf("[%s] Battery: %.2f\n", TAG, val_bat);
                    val_bat = (int)(100 * (float)(val_bat - BATTERY_MIN)/(BATTERY_MAX - BATTERY_MIN));
                    Serial.printf("[%s] Battery phan tram: %d\n", TAG, int(val_bat));
                    Node_Data[0].battery = (int) val_bat;

                    // ESP_LOGI(TAG,"LM35: %.1f", val_lm);
                    Serial.printf("[%s] LM35: %.1f\n",TAG, val_lm);
                    sprintf(Node_Data[0].temp_lm35, "%.1f", val_lm);

                    Node_Data[0].range = (float)((float)data_recv.Node_Range_Extenso / 10.0);
                    Serial.printf("[%s] Range Extenso: %.1f mm\n", TAG, Node_Data[0].range);
                    // ESP_LOGI(TAG,"Range Extenso: %.1f mm", Node_Data[0].range);
                    if (Node_Data[0].range >= Node_Data[0].range_threshold) gpio_set_level(Light_Warning, Light_ON);

                    if (number_news == 1)
                    {
                        Node0_range = Node_Data[0].range;
                        time_recv_pre = (uint64_t)millis();
                    }
                    else
                    {
                        if (millis() > time_recv_pre)
                        {
                            Node_Data[0].speed = (double)(Node_Data[0].range - Node0_range) * 60 / ((double)(millis() - time_recv_pre)/1000.0);
                            // if (Node_Data[0].speed < (-0.5)) gpio_set_level(Light_Warning, Light_ON);
                            Node0_range = Node_Data[0].range;
                            time_recv_pre = (uint64_t)millis();
                        }
                    }
                }

                else if (data_recv.Link.Node_ID == NODE_ID_2)
                {
                    Node_Data[1].node_id = (int)data_recv.Link.Node_ID;
                    Serial.printf("[%s] Node_2 ID: %02X\n", TAG, data_recv.Link.Node_ID);
                    Node_Data[1].period  = (int)data_recv.Link.Node_Period;

                    if (data_recv.Link.Node_Status == 1) Serial.printf("[%s] Status: WAKEUP MODE!\n", TAG);
                    else if (data_recv.Link.Node_Status == 2) Serial.printf("[%s] Status: LINK MODE!\n", TAG);
                         else if (data_recv.Link.Node_Status == 4) Serial.printf("[%s] Status: NORMAL MODE!\n", TAG);
                              else if (data_recv.Link.Node_Status == 8) Serial.printf("[%s] Status: RETRY MODE!\n", TAG);
                                   else if (data_recv.Link.Node_Status == 16) Serial.printf("[%s] Status: SHUTDOWN MODE!\n", TAG);

                    val_bat = (double)(((double)(data_recv.Link.Node_Battery_Voltage))/100.0);
                    val_lm = (double)(((double)(data_recv.Node_LM35_Temperature))/10.0);
                    // caculation battery
                    Serial.printf("[%s] Battery: %.2f\n", TAG, val_bat);
                    val_bat = (int)(100 * (float)(val_bat - BATTERY_MIN)/(BATTERY_MAX - BATTERY_MIN));
                    Serial.printf("[%s] Battery phan tram: %d\n", TAG, int(val_bat));
                    Node_Data[1].battery = (int) val_bat;

                    Serial.printf("[%s] LM35: %.1f\n",TAG, val_lm);
                    sprintf(Node_Data[1].temp_lm35, "%.1f", val_lm);

                    Node_Data[1].range = (float)((float)data_recv.Node_Range_Extenso / 10.0);
                    Serial.printf("[%s] Range Extenso: %.1f mm\n", TAG, Node_Data[1].range);
                    if (Node_Data[1].range >= Node_Data[0].range_threshold) gpio_set_level(Light_Warning, Light_ON);  
    
                    // sprintf(mqtt_data,"{\"NODE 2 Battery\": %d,\"NODE 2 Threshold\": %d,\"NODE 2 Range\": %.1f}", Node_Data[1].battery, Node_Data[1].range_threshold, Node_Data[1].range);
                    // esp_mqtt_client_publish(client, TOPIC , mqtt_data, 0, 1, 0);  
                }
                flag_LCD = 1;
                flag_MQTT = 1;

                // ResponsePacket_t resp = {PACKET_ID_2, data_recv.Link.Node_ID, NORMAL_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_CARRYON << 8) & 0xFF00) | LINK_ACK)};
                // // ESP_LOGI(TAG, "Response Ready...\t");
                
                // if (listen_before_talk() == false)
                // {
                //     sx1278_start_recv_data();
                //     break;
                // }
                // else
                // {
                //     ESP_LOGI(TAG, "Sending packet response!\t");
                //     sx1278_send_data(&resp, sizeof(ResponsePacket_t));
                // }
            }
            else
            {
                // ESP_LOGI(TAG, "NOT IN NETWORK");
                ResponsePacket_t resp = {PACKET_ID_2, data_recv.Link.Node_ID, LINK_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_DISMISS << 8) & 0xFF00) | LINK_ACK)};
                if (listen_before_talk() == false)
                {   
                    sx1278_start_recv_data();
                    break;
                }
                else
                {
                    // ESP_LOGI(TAG, "Sending packet...\t");
                    sx1278_send_data((uint8_t*)(&resp), sizeof(ResponsePacket_t));
                }
                // ESP_LOGI(TAG, "Done!\n");
                sx1278_start_recv_data();
            }
            sx1278_start_recv_data();
            break;
        }
    }
}

void sx1278_TASK(void *paramet)
{
	sx1278_gpio_init();
    sx1278_spi_init();
    sx1278_init();
  
    uint8_t data_re[100] = {0};
    uint32_t nByteRx = 0;
    int rssi = -1;
    float snr = -1;
    EventBits_t event_bits;
    uint8_t stateee = 0;
    sx1278_evt_group = xEventGroupCreate();
    sx1278_start_recv_data();
    while (1)
    {
        event_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS);
        if (event_bits & SX1278_DIO0_BIT)
        {
            // stat = 1 - stat;
            // gpio_set_level(LED_TFT_PIN, stat);
            if (sx1278_recv_data((uint8_t *)data_re, &nByteRx, &rssi, &snr, true) == SX1278_OK)
            {
                stateee = 1 - stateee;
                rssi_LCD = rssi;
                // gpio_set_level(LED_TFT_PIN, stateee);
                Serial.printf("\n[%s] LoRa Received %d bytes\trssi: %d\tsnr: %2.1f\n",TAG, nByteRx, rssi, snr);
                // ESP_LOGW(TAG, "\nLoRa Received %d bytes\trssi: %d\tsnr: %2.1f", nByteRx, rssi, snr);
                LoRa_Packet_Parser((const uint8_t *)data_re, nByteRx);
            }
        }
    }
}
