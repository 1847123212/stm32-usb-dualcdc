/**
 * Dual CDC Header file
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

/* Multiple inclusion */
#ifndef __DUALCDC_H
#define __DUALCDC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "usbd_ioreq.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

/* Macros */
// DCDC status
#define DCDC_OK (0)
#define DCDC_BUSY (1)
#define DCDC_FAIL (2)

// Rx and Tx buffer sizes
#define DCDC_RXBUF_SIZE  (512)
#define DCDC_TXBUF_SIZE  (512)

// Ports
#define DCDC_PORT1 (0x01)
#define DCDC_PORT2 (0x02)

// Endpoints for both ports
#define DCDC_P1_INTRIN_EP  (0x81)  // Port 1 EP for CDC commands
#define DCDC_P1_BULKIN_EP  (0x82)  // Port 1 EP for data IN
#define DCDC_P1_BULKOUT_EP (0x01)  // Port 1 EP for data OUT
#define DCDC_P2_INTRIN_EP  (0x83)  // Port 2 EP for CDC commands
#define DCDC_P2_BULKIN_EP  (0x84)  // Port 2 EP for data IN
#define DCDC_P2_BULKOUT_EP (0x03)  // Port 2 EP for data OUT

// Endpoint parameters
#ifdef USE_USB_HS
#define DCDC_DATA_PACKET_SIZE   (0x0200)   // High Speed data packet size
#define DCDC_CMD_PACKET_SIZE    (0x40)    // High speed command packet size
#else
#define DCDC_DATA_PACKET_SIZE   (0x0040)   // High Speed data packet size
#define DCDC_CMD_PACKET_SIZE    (0x08)     // Full speed command packet size
#endif

#define DCDC_DATA_HS_IN_PACKET_SIZE  (DCDC_DATA_PACKET_SIZE)
#define DCDC_DATA_HS_OUT_PACKET_SIZE (DCDC_DATA_PACKET_SIZE)
#define DCDC_DATA_FS_IN_PACKET_SIZE  (DCDC_DATA_PACKET_SIZE)
#define DCDC_DATA_FS_OUT_PACKET_SIZE (DCDC_DATA_PACKET_SIZE)

// Config descriptor size
#define DCDC_CONFIG_DESC_SIZE_LB (0x8D)
#define DCDC_CONFIG_DESC_SIZE_HB (0x00)

// Dual CDC interfaces
typedef struct {
    USBD_CDC_ItfTypeDef *CDC1;
    USBD_CDC_ItfTypeDef *CDC2;
} DCDC_ItfTypeDef;

// Dual CDC handles
typedef struct {
    USBD_CDC_HandleTypeDef hcdc1;
    USBD_CDC_HandleTypeDef hcdc2;
} DCDC_HandleTypeDef;

extern USBD_ClassTypeDef  DCDC;

uint8_t DCDC_RegisterInterface (USBD_HandleTypeDef *pdev, DCDC_ItfTypeDef *fops);
uint8_t DCDC_TransmitData(uint8_t com_port, uint8_t *tx_buf, uint16_t tx_len);

#ifdef __cplusplus
}
#endif

#endif  /* __DUALCDC_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
