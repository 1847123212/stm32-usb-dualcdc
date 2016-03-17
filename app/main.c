/**
 * Main C file
 *
 * This file is part of stm32-usb-dualcdc, an implementation of Dual VCP ports
 * over USB for STM32F4xx controllers.
 * This project is available at
 * <https://github.com/jisszacharia/stm32-usb-dualcdc>
 *
 * stm32-usb-dualcdc is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or any
 * later version (at your option).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * GNU Lesser General Public License is available at
 * <http://www.gnu.org/licenses/>
 *
 * This project uses STM32F4xx HAL library and STM32 USB Device Library
 * ST's license agreement is available at
 * <http://www.st.com/software_license_agreement_liberty_v2>
 *
 * Copyright (c) 2016 JZJ <jiss.joseph@gmail.com>
**/

/* Includes */
#include "main.h"
#include "dualcdc.h"

/* Extern */
extern USBD_ClassTypeDef DCDC_cbs;
extern DCDC_ItfTypeDef DCDC_fops;

/* Global */
USBD_HandleTypeDef   USBDevice;

/* Function prototypes */
void PlatformInit(void);

int main()
{
    // Init board
    PlatformInit();

    // Init USBD library
	USBD_Init(&USBDevice, &USBD_Desc, 0);
    // Register Class driver
	USBD_RegisterClass(&USBDevice, &DCDC_cbs);
    // Register class callbacks
    DCDC_RegisterInterface(&USBDevice, &DCDC_fops);

    // Start USB
    USBD_Start(&USBDevice);

    while(1)
    {
        CDC_Itf_ProcessData();
    }
}

void PlatformInit(void)
{
    // Init the ST HAL first
    HAL_Init();

    // GPIO A, B and C
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // HS select
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    // LED
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    __BKPT(1);
}
#endif