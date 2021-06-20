/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MIMXRT1021_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1021.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

#include "fsl_lpi2c.h"
#include "user_utilities.h"

/* TODO: insert other definitions and declarations here. */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define EXAMPLE_I2C_MASTER_BASE (LPI2C4_BASE)//The FXO accelerometer is connected to 4
#define EXAMPLE_I2C_MASTER_BASE (LPI2C1_BASE)//The PCF85063 RTC is connected to LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define LPI2C_MASTER_CLOCK_FREQUENCY LPI2C_CLOCK_FREQUENCY
#define WAIT_TIME                    10U


#define EXAMPLE_I2C_MASTER ((LPI2C_Type *)EXAMPLE_I2C_MASTER_BASE)


#define ADDRESS_RTC 0x51U //Address for the RTC PCF85063
#define ADDRESS_BME680 0x77U //Address for the BME680 sensor page 38 datasheet
#define LPI2C_BAUDRATE               100000U
#define LPI2C_DATA_LENGTH            18U



#define EXAMPLE_LED_GPIO     BOARD_USER_LED_GPIO
#define EXAMPLE_LED_GPIO_PIN BOARD_USER_LED_PIN
/*******************************************************************************
 * Variables
 ******************************************************************************/

/* The PIN status */
volatile bool g_pinSet = false;

uint8_t g_master_txBuff[LPI2C_DATA_LENGTH];
uint8_t g_master_rxBuff[LPI2C_DATA_LENGTH];

uint16_t temp;

status_t I2C_SendFunc(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff)
{
    status_t reVal        = kStatus_Fail;
    size_t txCount        = 0xFFU;


	if (kStatus_Success == LPI2C_MasterStart(EXAMPLE_I2C_MASTER, deviceAddress, kLPI2C_Write))
	    {
	        /* Check master tx FIFO empty or not */
	        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
	        while (txCount)
	        {
	            LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
	        }
	        /* Check communicate with slave successful or not */
	        if (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
	        {
	            return kStatus_LPI2C_Nak;
	        }


	        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &subAddress, 1);
	        if (reVal != kStatus_Success)
	        {
	            if (reVal == kStatus_LPI2C_Nak)
	            {
	                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
	            }
	            return -1;
	        }

	        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &txBuff, LPI2C_DATA_LENGTH);
	        if (reVal != kStatus_Success)
	        {
	            if (reVal == kStatus_LPI2C_Nak)
	            {
	                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
	            }
	            return -1;
	        }

	        reVal = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
	        if (reVal != kStatus_Success)
	        {
	            return -1;
	        }
	    }

	return 0;
}

status_t I2C_ReceiveFunc(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    /* Receive blocking data from slave */
    status_t reVal        = kStatus_Fail;
    size_t txCount        = 0xFFU;

    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
    if (kStatus_Success == LPI2C_MasterStart(EXAMPLE_I2C_MASTER, deviceAddress, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
         LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
         while (txCount)
         {
             LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txCount);
         }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER) & kLPI2C_MasterNackDetectFlag)
        {
            return kStatus_LPI2C_Nak;
        }

        reVal = LPI2C_MasterSend(EXAMPLE_I2C_MASTER, &subAddress, 1);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }

        reVal = LPI2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, deviceAddress, kLPI2C_Read);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = LPI2C_MasterReceive(EXAMPLE_I2C_MASTER, rxBuff, rxBuffSize - 1);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
            }
            return -1;
        }

        reVal = LPI2C_MasterStop(EXAMPLE_I2C_MASTER);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
    }

    return 0;
}


/*
 * @brief   Application entry point.
 */
int main(void) {

	lpi2c_master_config_t masterConfig;
	//uint8_t reg_addr = 0x08U;
	g_master_txBuff[0]= 0x08;


    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }

    PRINTF("Reading RTC PCF85063 and BME680\n");
    /*Clock setting for LPI2C*/
        CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
        CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);
        /*
         * masterConfig.debugEnable = false;
         * masterConfig.ignoreAck = false;
         * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
         * masterConfig.baudRate_Hz = 100000U;
         * masterConfig.busIdleTimeout_ns = 0;
         * masterConfig.pinLowTimeout_ns = 0;
         * masterConfig.sdaGlitchFilterWidth_ns = 0;
         * masterConfig.sclGlitchFilterWidth_ns = 0;
         */
        LPI2C_MasterGetDefaultConfig(&masterConfig);

        /* Change the default baudrate configuration */
        masterConfig.baudRate_Hz = LPI2C_BAUDRATE;

        /* Initialize the LPI2C master peripheral */
        LPI2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, LPI2C_MASTER_CLOCK_FREQUENCY);



       I2C_ReceiveFunc(ADDRESS_RTC, 0x08, 1, &g_master_rxBuff[0],10);
//        LPI2C_MasterStart(EXAMPLE_I2C_MASTER, ADDRESS_RTC, kLPI2C_Write);
//
//        LPI2C_MasterSend(EXAMPLE_I2C_MASTER, g_master_txBuff, 1);
//
//        LPI2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, ADDRESS_RTC, kLPI2C_Read);
//
//        LPI2C_MasterReceive(EXAMPLE_I2C_MASTER, g_master_rxBuff, 10);
//
//        LPI2C_MasterStop(EXAMPLE_I2C_MASTER);



        for(int i=0;i<10;i++){
        	PRINTF("RTC register 0x08 data read %18d \n", g_master_rxBuff[i]);
        }

        g_master_rxBuff[0] = 0x0;

        I2C_ReceiveFunc(ADDRESS_BME680, 0xD0, 1, &g_master_rxBuff[0],2);
        for(int i=0;i<2;i++){
        	PRINTF("BME680 register 0xD0 Chip ID %18d \n", g_master_rxBuff[i]);
        }

        I2C_SendFunc(ADDRESS_BME680, 0x74, 1, 0x59);
        I2C_SendFunc(ADDRESS_BME680, 0x75, 1, 0x00);
        I2C_SendFunc(ADDRESS_BME680, 0x72, 1, 0x01);

        I2C_SendFunc(ADDRESS_BME680, 0x74, 1, 0x59);

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        /* Delay 1000 ms */
        SysTick_DelayTicks(1000U);
        if (g_pinSet)
        {
            GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 0U);
            g_pinSet = false;
        }
        else
        {
            GPIO_PinWrite(EXAMPLE_LED_GPIO, EXAMPLE_LED_GPIO_PIN, 1U);
            g_pinSet = true;
        }

        I2C_SendFunc(ADDRESS_BME680, 0x74, 1, 0x59);
        I2C_ReceiveFunc(ADDRESS_BME680, 0x22, 1, &g_master_rxBuff[0],2);
        //temp = g_master_rxBuff[0] | g_master_rxBuff[1];

        for(int i=0;i<2;i++){
        	PRINTF("BME680 register 0x22 temperature %18d \n", g_master_rxBuff[i]);
        }
        PRINTF("BME680 register 0x22 y 0x23 temperature %d \n", temp);

        __asm volatile ("nop");
    }
    return 0 ;
}
