/**
******************************************************************************
* @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
* @author  MCD Application Team
* @version V1.0.0
* @date    17-December-2014
* @brief   Source file for USBD CDC interface
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "dualcdc.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
* @{
*/

/** @defgroup USBD_CDC
* @brief usbd core module
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding_P1 =
{
    9600,   /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
};

USBD_CDC_LineCodingTypeDef LineCoding_P2 =
{
    9600,   /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* USB handler declaration */
extern USBD_HandleTypeDef  USBDevice;

// Application buffers for demo
char CDC1_Data[512];
char CDC2_Data[512];
uint32_t CDC1_DataLen = 0;
uint32_t CDC2_DataLen = 0;


/* Private function prototypes -----------------------------------------------*/
static int8_t CDC1_Itf_Init     (void);
static int8_t CDC1_Itf_DeInit   (void);
static int8_t CDC1_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC1_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);
static int8_t CDC2_Itf_Init     (void);
static int8_t CDC2_Itf_DeInit   (void);
static int8_t CDC2_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC2_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef CDC1_fops =
{
    // CDC 1 interfaces
    CDC1_Itf_Init,
    CDC1_Itf_DeInit,
    CDC1_Itf_Control,
    CDC1_Itf_Receive
};

USBD_CDC_ItfTypeDef CDC2_fops =
{
    // CDC 2 interfaces
    CDC2_Itf_Init,
    CDC2_Itf_DeInit,
    CDC2_Itf_Control,
    CDC2_Itf_Receive
};

DCDC_ItfTypeDef DCDC_fops =
{
    &CDC1_fops,
    &CDC2_fops
};

/* Public functions ----------------------------------------------------------*/
void CDC_Itf_ProcessData(void)
{
    if(CDC1_DataLen)
    {
        DCDC_TransmitData(DCDC_PORT2, (uint8_t*)CDC1_Data, CDC1_DataLen);
        CDC1_DataLen = 0;
    }
    if(CDC2_DataLen)
    {
        DCDC_TransmitData(DCDC_PORT1, (uint8_t*)CDC2_Data, CDC2_DataLen);
        CDC2_DataLen = 0;
    }
}

/* Private functions ---------------------------------------------------------*/
/**
* @brief  CDC1_Itf_Init
*         Initializes the CDC1 media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC1_Itf_Init(void)
{
    return (USBD_OK);
}

/**
* @brief  CDC1_Itf_DeInit
*         DeInitializes the CDC media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC1_Itf_DeInit(void)
{
    return (USBD_OK);
}

/**
* @brief  CDC1_Itf_Control
*         Manage the CDC class requests
* @param  Cmd: Command code
* @param  Buf: Buffer containing command data (request parameters)
* @param  Len: Number of data to be sent (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC1_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch (cmd)
    {
        case CDC_SEND_ENCAPSULATED_COMMAND:
        /* Add your code here */
        break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
        /* Add your code here */
        break;

        case CDC_SET_COMM_FEATURE:
        /* Add your code here */
        break;

        case CDC_GET_COMM_FEATURE:
        /* Add your code here */
        break;

        case CDC_CLEAR_COMM_FEATURE:
        /* Add your code here */
        break;

        case CDC_SET_LINE_CODING:
            LineCoding_P1.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                                  (pbuf[2] << 16) | (pbuf[3] << 24));
            LineCoding_P1.format     = pbuf[4];
            LineCoding_P1.paritytype = pbuf[5];
            LineCoding_P1.datatype   = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(LineCoding_P1.bitrate);
            pbuf[1] = (uint8_t)(LineCoding_P1.bitrate >> 8);
            pbuf[2] = (uint8_t)(LineCoding_P1.bitrate >> 16);
            pbuf[3] = (uint8_t)(LineCoding_P1.bitrate >> 24);
            pbuf[4] = LineCoding_P1.format;
            pbuf[5] = LineCoding_P1.paritytype;
            pbuf[6] = LineCoding_P1.datatype;
            break;

        case CDC_SET_CONTROL_LINE_STATE:
        /* Add your code here */
        break;

        case CDC_SEND_BREAK:
        /* Add your code here */
        break;

        default:
        break;
    }

    return (USBD_OK);
}

/**
* @brief  CDC1_Itf_Receive
*         Data received over USB OUT endpoint are sent over CDC interface
*         through this function.
* @param  Buf: Buffer of data to be transmitted
* @param  Len: Number of data received (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC1_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
    CDC1_DataLen = *Len;
    strncpy(CDC1_Data, (const char*) Buf, CDC1_DataLen);
    return (USBD_OK);
}

/**
* @brief  CDC2_Itf_Init
*         Initializes the CDC2 media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC2_Itf_Init(void)
{
    return (USBD_OK);
}

/**
* @brief  CDC2_Itf_DeInit
*         DeInitializes the CDC media low layer
* @param  None
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC2_Itf_DeInit(void)
{
    return (USBD_OK);
}

/**
* @brief  CDC2_Itf_Control
*         Manage the CDC class requests
* @param  Cmd: Command code
* @param  Buf: Buffer containing command data (request parameters)
* @param  Len: Number of data to be sent (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC2_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch (cmd)
    {
        case CDC_SEND_ENCAPSULATED_COMMAND:
        /* Add your code here */
        break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
        /* Add your code here */
        break;

        case CDC_SET_COMM_FEATURE:
        /* Add your code here */
        break;

        case CDC_GET_COMM_FEATURE:
        /* Add your code here */
        break;

        case CDC_CLEAR_COMM_FEATURE:
        /* Add your code here */
        break;

        case CDC_SET_LINE_CODING:
            LineCoding_P2.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                                  (pbuf[2] << 16) | (pbuf[3] << 24));
            LineCoding_P2.format     = pbuf[4];
            LineCoding_P2.paritytype = pbuf[5];
            LineCoding_P2.datatype   = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(LineCoding_P2.bitrate);
            pbuf[1] = (uint8_t)(LineCoding_P2.bitrate >> 8);
            pbuf[2] = (uint8_t)(LineCoding_P2.bitrate >> 16);
            pbuf[3] = (uint8_t)(LineCoding_P2.bitrate >> 24);
            pbuf[4] = LineCoding_P2.format;
            pbuf[5] = LineCoding_P2.paritytype;
            pbuf[6] = LineCoding_P2.datatype;
            break;

        case CDC_SET_CONTROL_LINE_STATE:
        /* Add your code here */
        break;

        case CDC_SEND_BREAK:
        /* Add your code here */
        break;

        default:
        break;
    }

    return (USBD_OK);
}

/**
* @brief  CDC2_Itf_Receive
*         Data received over USB OUT endpoint are sent over CDC interface
*         through this function.
* @param  Buf: Buffer of data to be transmitted
* @param  Len: Number of data received (in bytes)
* @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
*/
static int8_t CDC2_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
    CDC2_DataLen = *Len;
    strncpy(CDC2_Data, (const char*) Buf, CDC2_DataLen);
    return (USBD_OK);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

