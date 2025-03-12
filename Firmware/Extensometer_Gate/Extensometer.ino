#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <PubSubClient.h>
#include "EEPROM.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_log.h"
// #include "esp_smartconfig.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include "esp_attr.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_sntp.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/adc.h"

#include "sx1278.h"
#include "smartcfg.h"
#include "button.h"
#include "HD44780.h"

#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 20
#define LCD_ROWS 4

#define LENGTH(x) (strlen(x) + 1)
#define EEPROM_SIZE 200

static const char *TAG = "Main";

// extern EventGroupHandle_t sx1278_evt_group;
extern int flag_MQTT;
extern uint8_t state;
// extern sx1278_node_slot_t Node_Data[3];
int count_past = 0;
int connect_time = 0;
uint8_t wifi_status;

// const char* ssid = "Laptop";
// const char* password = "888888888";

//variable to store ssid and password
String ssid;
String pss;

//Khai báo Thingsboard
#define MQTT_SERVER   "demo.thingsboard.io"
#define MQTT_PORT     1883
#define MQTT_USERNAME "Mandevices"     
#define MQTT_PASSWORD "MANDevices"       
#define MQTT_NAME     "Extenso"
#define TOPIC         "v1/devices/me/telemetry"
WiFiClient client;
PubSubClient mqtt(client);

AsyncWebServer server(80);
int LED = 2;

// reconnect wifi
void connect_WiFi(void)
{
  int retry = 0;
  while ((WiFi.status() != WL_CONNECTED) && (retry < 14)) 
  {
    delay(500);
    retry++;
    Serial.print(".");
  }
}

void writeStringToFlash(const char* toStore, int startAddr) {
  int i = 0;
  for (; i < LENGTH(toStore); i++) {
    EEPROM.write(startAddr + i, toStore[i]);
  }
  EEPROM.write(startAddr + i, '\0');
  EEPROM.commit();
}


String readStringFromFlash(int startAddr) {
  char in[128]; // char array of size 128 for reading the stored data 
  int i = 0;
  for (; i < 128; i++) {
    in[i] = EEPROM.read(startAddr + i);
  }
  return String(in);
}

void setup(void) {
  Serial.begin(115200);
  // pinMode(LED, OUTPUT);
  Serial.print("Current CPU Frequency: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");

  Node_Data[0].range_threshold = 900;
  Node_Data[1].range_threshold = 900;
  Node_Data[2].range_threshold = 900;
  
  LCD_init(0x27, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
	lcd_clear();
	LCD_setCursor(0, 1);
	LCD_writeStr("----EXTENSOMETER----");
  LCD_setBacklight(state_backlight);  

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  if (!EEPROM.begin(EEPROM_SIZE)) 
  {
    Serial.println("failed to init EEPROM");
    delay(1000);
  }
  else
  {
    //Read SSID stored at address 0
    ssid = readStringFromFlash(0);
    Serial.print("SSID = ");
    Serial.println(ssid);
    // Read Password stored at address 40
    pss = readStringFromFlash(40);
    Serial.print("psss = ");
    Serial.println(pss);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pss.c_str());
  connect_WiFi();

  if (WiFi.status() != WL_CONNECTED)
  {
    //Init WiFi as Station, start SmartConfig
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    //Wait for SmartConfig packet from mobile
    Serial.println("Waiting for SmartConfig.");

  	lcd_clear();
	  LCD_setCursor(4, 1);
	  LCD_writeStr("Quet wifi...");
    int retry_smartconfig = 120;
    while ((!WiFi.smartConfigDone()) && (retry_smartconfig >= 0)) 
    {
      if (retry_smartconfig % 2 == 0)
      {
        LCD_clear_row_3();
        LCD_setCursor(9, 2);
        lcd_number(retry_smartconfig/2);
      }
      delay(500);
      retry_smartconfig--;
    }

    if (WiFi.smartConfigDone())
    {
      Serial.println("");
      Serial.println("SmartConfig received.");
      //Wait for WiFi to connect to AP
      connect_WiFi();
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.println("WiFi Connected.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        //read the connected WiFi SSID and password
        ssid = WiFi.SSID();
        pss = WiFi.psk();

        Serial.print("SSID:");
        Serial.println(ssid);
        Serial.print("PASS:");
        Serial.println(pss);
        Serial.println("Storing SSID & PASSWORD in EEPROM");

        //store the ssid at address 0
        writeStringToFlash(ssid.c_str(), 0);
        //store the password at address 40
        writeStringToFlash(pss.c_str(), 40);
        WiFi.stopSmartConfig();
      }
      else 
      {
        WiFi.stopSmartConfig();
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pss.c_str());
      }
    }
    else 
    {
      WiFi.stopSmartConfig();
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid.c_str(), pss.c_str());
    }
  }
  
  //Khai báo MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);

	xTaskCreate(&button_task, "button_task", 1024*4, NULL, 5, NULL);
	xTaskCreate(&sx1278_TASK, "sx1278_task", 1024*8, NULL, 8, NULL);
  xTaskCreate(&mqtt_TASK, "mqtt_task", 1024*4, NULL, 7, NULL);
  delay(5000);
  // OTA
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Connected to ");
    Serial.println(ssid.c_str());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am MANDev.");
    });

    server.begin();
    Serial.println("HTTP server started");

    ElegantOTA.begin(&server);    // Start ElegantOTA
  }
  // setCpuFrequencyMhz(160);
}

unsigned int time_ota_start = millis();
// IP/update
void loop(void) {
  ElegantOTA.loop(); // Sketch > Export Compiled Binary
  delay(500);
  if (millis() - time_ota_start >= 150000)
  {
    server.end();
    Serial.println("OTA Disabled!");
    while(1) delay(1000);
  }
  // digitalWrite(LED, LOW);
  // delay(500);
  // digitalWrite(LED, HIGH);
}

void mqtt_TASK(void *par)
{
  // unsigned int time_mqtt = millis();
  mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD);
  char mqtt_data[100];
  while (1)
  {
    wifi_status = WiFi.status();
    // while ((WiFi.status() != WL_CONNECTED) && (millis() - time_mqtt < 180000)) 
    // {
    //   delay(500);
    //   Serial.print(".");
    // }

    mqtt.loop();
    if (flag_MQTT == 1)
    { 
      if (mqtt.connected() == false)
      {
        if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD))
        {
            Serial.println("mqtt connected");
        } 
        else Serial.println("mqtt connect failed");
      }

      if (mqtt.connected() == true)
      {
          sprintf(mqtt_data,"{\"NODE 1 Battery\": %d,\"NODE 1 Threshold\": %d,\"NODE 1 Range\": %.1f, \"NODE 1 Speed\": %.3f}", Node_Data[0].battery, Node_Data[0].range_threshold, Node_Data[0].range, Node_Data[0].speed);
          mqtt.publish(TOPIC, mqtt_data);
      }
      flag_MQTT = 0;
    }
    delay(5);
  }
}