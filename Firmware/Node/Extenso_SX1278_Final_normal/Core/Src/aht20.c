/*
 * aht20.c
 *
 *  Created on: Oct 23, 2023
 *      Author: baris
 */


#include "aht20.h"

#define AHT20_ADDR		0x70

void AHT20_init(){
	uint8_t readBufer;
	HAL_Delay(40);
	HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, &readBufer, 1, 100);
	if((readBufer & 0x08) == 0x00){
		uint8_t sendBuffer[3]={0xBE, 0x08, 0x00};
		HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, sendBuffer, 3, 100);
	}
}

void AHT20_read(float *temp, float *humid){
	uint8_t sendBuffer[3]={0xAC, 0x33, 0x00};
	uint8_t readBuffer[6];
	HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, sendBuffer, 3, 100);
	HAL_Delay(80);
	HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, readBuffer, 6, 100);
	if((readBuffer[0] & 0x80)==0x00){
		uint32_t data=0;
		data = ((uint32_t)readBuffer[3]>>4) + ((uint32_t)readBuffer[2]<<4) + ((uint32_t)readBuffer[1]<<12);
		*humid = data*100.0f/(1<<20);

		data = (((uint32_t)readBuffer[3] & 0x0F) <<16) + ((uint32_t)readBuffer[4]<<8) + ((uint32_t)readBuffer[5]);
		*temp = data*200.0f/(1<<20)-50;
	}
}
