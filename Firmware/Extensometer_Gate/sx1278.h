/**
 * @file sx1278.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 *  Modify: Nov 30, 2024 
 *  Author: Dinh Gioi - HUST - K65
 */


#ifndef _SX1278_H_
#define _SX1278_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define SX1278_MOSI_PIN GPIO_NUM_23
#define SX1278_MISO_PIN GPIO_NUM_19
#define SX1278_NSS_PIN  GPIO_NUM_5
#define SX1278_SCK_PIN  GPIO_NUM_18
#define SX1278_DIO0_PIN GPIO_NUM_32
// #define SX1278_DIO4_PIN GPIO_NUM_35
// #define SX1278_DIO3_PIN GPIO_NUM_34
#define SX1278_RST_PIN  GPIO_NUM_33
#define SX1278_TXEN_PIN GPIO_NUM_26
#define SX1278_RXEN_PIN GPIO_NUM_25


#define SX1278_DIO0_BIT BIT0
// #define SX1278_DIO3_BIT BIT1
// #define SX1278_DIO4_BIT BIT2

// #define NODE_ID     (0xCAAEU)
#define NODE_ID_3     (0xCAADU)
#define NODE_ID_2     (0xCAACU)
#define NODE_ID_1     (0xCAABU)

// Register definitions
#define REG_FIFO                        0x00
#define REG_OP_MODE                     0x01
#define REG_FRF_MSB                     0x06
#define REG_FRF_MID                     0x07
#define REG_FRF_LSB                     0x08
#define REG_PA_CONFIG                   0x09
#define REG_LNA                         0x0c
#define REG_FIFO_ADDR_PTR               0x0d
#define REG_FIFO_TX_BASE_ADDR           0x0e
#define REG_FIFO_RX_BASE_ADDR           0x0f
#define REG_FIFO_RX_CURRENT_ADDR        0x10
#define REG_IRQ_FLAGS                   0x12
#define REG_RX_NB_BYTES                 0x13
#define REG_PKT_SNR_VALUE               0x19
#define REG_PKT_RSSI_VALUE              0x1a
#define REG_MODEM_CONFIG_1              0x1d
#define REG_MODEM_CONFIG_2              0x1e
#define REG_PREAMBLE_MSB                0x20
#define REG_PREAMBLE_LSB                0x21
#define REG_PAYLOAD_LENGTH              0x22
#define REG_MODEM_CONFIG_3              0x26
#define REG_RSSI_WIDEBAND               0x2c
#define REG_DETECTION_OPTIMIZE          0x31
#define REG_DETECTION_THRESHOLD         0x37
#define REG_SYNC_WORD                   0x39
#define REG_DIO_MAPPING_1               0x40
#define REG_VERSION                     0x42

// Transceiver modes
#define MODE_LONG_RANGE_MODE            0x80
#define MODE_SLEEP                      0x00
#define MODE_STDBY                      0x01
#define MODE_TX                         0x03
#define MODE_RX_CONTINUOUS              0x05
#define MODE_RX_SINGLE                  0x06
#define MODE_CAD                        0x07

// PA configuration
#define PA_BOOST                        0x80

// IRQ masks
#define IRQ_TX_DONE_MASK                0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK      0x20
#define IRQ_RX_DONE_MASK                0x40
#define IRQ_VALID_HEADER_MASK           0x10
#define IRQ_CAD_DETECTED_MASK			      0x01
#define IRQ_CAD_DONE_MASK				        0x04

#define PA_OUTPUT_RFO_PIN               0
#define PA_OUTPUT_PA_BOOST_PIN          1

#define TIMEOUT_RESET                   100

#define ACK 0x06
#define NACK 0x15

#define UPLINK_TX_REQUEST_OPCODE 10
#define DOWNLINK_RX_DATA_OPCODE 20

#define NW_DEFAULT_TOTAL_SLOTS 10
#define NW_DEFAULT_PERIOD 5
#define NW_DEFAULT_THRESHOLD 25.0

#define PACKET_ID_0   (0xBBBBU)
#define PACKET_ID_1   (0x4A4AU)
#define PACKET_ID_2   (0x4444U)

#define LINK_ACCEPT   (0xAAU)
#define LINK_REJECT   (0x55U)

#define LINK_CARRYON  (0x00U)
#define LINK_DISMISS  (0xFFU)

#define LINK_ACK      (0xABU)
#define LINK_NACK     (0xBAU)

#define DEFAULT_PERIOD_SEC  (300U)

// DO NOT CHANGE
// #define EVT_LORA      (0x04U)
// #define EVT_TOCUH     (0x02U)

// DO NOT CHANGE
#define MAX_NODE      (6U)

typedef struct LoRa_Link_Struct
{
  uint16_t Node_ID;
  uint16_t Node_Status; 
  uint16_t Node_Battery_Voltage;
  uint16_t Node_Period;
} Link_Struct_t;

typedef struct LoRa_Data_Struct
{
  Link_Struct_t Link;
  uint16_t Node_LM35_Temperature;
  uint16_t Node_Range_Extenso;
  // uint16_t Node_AHT_Temperature;
  // uint16_t Node_AHT_Humidity;
  // uint16_t Node_wind_speed;
  // uint16_t Node_wind_direction;
  // uint16_t Node_wind_level;
} Data_Struct_t;

typedef struct LoRa_Packet_ID_0
{
  uint16_t Packet_ID;
  Link_Struct_t Payload;
} Link_Packet_t;

typedef struct LoRa_Packet_ID_1
{
  uint16_t Packet_ID;
  Data_Struct_t Payload;
} Data_Packet_t;

typedef struct LoRa_Packet_ID_2
{
  uint16_t Packet_ID;
  uint16_t Target_Node_ID;
  uint16_t Target_Node_Status; 
  uint16_t Target_Node_Period;
  uint16_t Target_Node_Response;
} ResponsePacket_t;

typedef enum NodeStatus
{
  WAKEUP_MODE = 1U,
  LINK_MODE = 2U,
  NORMAL_MODE = 4U,
  RETRY_MODE = 8U,
  SHUTDOWN_MODE = 16U
} NodeStatus_t;

typedef int sx1278_opcode_type_t;

typedef enum
{
    SX1278_OK,
    SX1278_NOT_OK,
    SX1278_INVALID_RX_DONE,
    SX1278_PAYLOAD_CRC_ERROR,
    SX1278_INVALID_HEADER
} sx1278_err_t;

typedef struct Node_Slot
{
    int node_id;
    int slot_id;
    int period;

    char temp_lm35[7];
    
    int battery;
    char temp_aht[10];
    char humid_aht[7];

    float range;
    double speed;
    int range_threshold;
} sx1278_node_slot_t;

typedef struct 
{
    char opcode[5];
    char node_id[5];
    char gate_id[5];
    char battery[7];
    char period[5];

    char temp_lm35[7];
    char threshold[7];
    
    char temp_aht[7];
    char humid_aht[7];
    char wind_speed[7];
    char wind_level[7];
    char wind_direction[7];
    uint8_t crc;
} sx1278_packet_t;

typedef struct 
{
    float threshold;
    // int level;
    int period;
} sx1278_attr_cfg_t;

typedef struct 
{
    bool network_run;
} sx1278_flag_t;

typedef struct 
{
    int total_slots;
    int gate_id;
    sx1278_node_slot_t node_slots[NW_DEFAULT_TOTAL_SLOTS];
    sx1278_flag_t flags;
} sx1278_network_t;

extern sx1278_node_slot_t Node_Data[3];
extern int flag_LCD;

static Data_Struct_t Node_Table[MAX_NODE] = {0};
void gpio_isr_handler(void *arg);

void sx1278_TASK(void *paramet);
void sx1278_task(void *param);

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(double n, char *res, int afterpoint);
// void IRAM_ATTR sx1278_intr_handler(void *arg);
void sx1278_gpio_init(void);
void sx1278_spi_init(void);
uint8_t sx1278_read_reg(uint8_t reg);
void sx1278_write_reg(uint8_t reg, uint8_t val);
void sx1278_reset(void);
void sx1278_sleep(void);
void sx1278_standby(void);
void sx1278_rx_contiuous(void);
void sx1278_rx_single(void);
void sx1278_tx(void);
void sx1278_set_irq(uint8_t val);
void sx1278_cad(void);
void sx1278_set_tx_power(uint8_t output_power);
void sx1278_set_LNA_gain(uint8_t gain);
void sx1278_set_freq(uint64_t freq);
void sx1278_set_bandwidth(long band);
void sx1278_set_sf(uint8_t sf);
void sx1278_set_cr(uint8_t cr);
void sx1278_set_header(bool en, uint32_t size);
void sx1278_set_crc(bool en);
void sx1278_set_preamble(int len);
int sx1278_get_rssi(void);
float sx1278_get_snr(void);
void sx1278_init(void);
void sx1278_send_data(uint8_t *data_send, int size);
void sx1278_start_recv_data(void);
sx1278_err_t sx1278_recv_data(uint8_t *data_recv, uint32_t *len, int *rssi, float *snr, bool isStayinRX);
int get_random_value(int min, int max);
uint8_t get_crc_value(uint8_t *data, int len);
bool listen_before_talk(void);
bool is_Node_ID_inNetwork(uint16_t Node_ID);
uint8_t sizeof_node();
void LoRa_Packet_Parser(const uint8_t *data, uint32_t len);

//sx1278_err_t parse_packet(uint8_t *packet_data, sx1278_node_slot_t *node_slot);
#endif