/*
 * user_utilities.h
 *
 *  Created on: Jun 20, 2021
 *      Author: jsantana
 */

#ifndef USER_UTILITIES_H_
#define USER_UTILITIES_H_

#include <stddef.h>
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include <stdlib.h>
#include <string.h>
#include "fsl_lpi2c.h"

//#define EXAMPLE_I2C_MASTER_BASE (LPI2C1_BASE)//The PCF85063 RTC is connected to LPI2C1
//#define EXAMPLE_I2C_MASTER ((LPI2C_Type *)EXAMPLE_I2C_MASTER_BASE)
//
//#define LPI2C_DATA_LENGTH            255U


void SysTick_Handler(void);

void SysTick_DelayTicks(uint32_t n);

//status_t I2C_SendFunc(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff);

//status_t I2C_ReceiveFunc(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint8_t *rxBuff, uint8_t rxBuffSize);

#endif /* USER_UTILITIES_H_ */
