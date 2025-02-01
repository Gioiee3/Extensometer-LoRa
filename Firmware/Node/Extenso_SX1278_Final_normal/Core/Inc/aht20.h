/*
 * aht20.h
 *
 *  Created on: Oct 23, 2023
 *      Author: baris
 */

#ifndef INC_AHT20_H_
#define INC_AHT20_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void AHT20_init();
void AHT20_read(float *temp, float *humid);

#endif /* INC_AHT20_H_ */
