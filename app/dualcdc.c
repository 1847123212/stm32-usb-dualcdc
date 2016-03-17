/**
 * Dual CDC module
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
#include "dualcdc.h"
#include "usbd_cdc_if.h"

/* Externs */
extern USBD_HandleTypeDef USBDevice;

/* Public */
// USBD CDC Tx/Rx buffers
uint8_t DCDC_RxBuf_P1[DCDC_RXBUF_SIZE]; // VCP1 RX Buffer
uint8_t DCDC_TxBuf_P1[DCDC_TXBUF_SIZE]; // VCP1 TX Buffer
uint8_t DCDC_RxBuf_P2[DCDC_RXBUF_SIZE]; // VCP2 RX Buffer
uint8_t DCDC_TxBuf_P2[DCDC_TXBUF_SIZE]; // VCP2 TX Buffer

/* Function prototypes */
static uint8_t  DCDC_Init (USBD_HandleTypeDef *pdev,
                                uint8_t cfgidx);
static uint8_t  DCDC_DeInit (USBD_HandleTypeDef *pdev,
                                  uint8_t cfgidx);
static uint8_t  DCDC_Setup (USBD_HandleTypeDef *pdev,
                                 USBD_SetupReqTypedef *req);
static uint8_t  DCDC_DataIn (USBD_HandleTypeDef *pdev,
                                  uint8_t epnum);
static uint8_t  DCDC_DataOut (USBD_HandleTypeDef *pdev,
                                   uint8_t epnum);
static uint8_t  DCDC_SOF (USBD_HandleTypeDef *pdev);
static uint8_t  DCDC_EP0_TxSent (USBD_HandleTypeDef *pdev);
static uint8_t  DCDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
static uint8_t  *DCDC_GetFSCfgDesc (uint16_t *length);
static uint8_t  *DCDC_GetHSCfgDesc (uint16_t *length);
static uint8_t  *DCDC_GetOtherSpeedCfgDesc (uint16_t *length);
static uint8_t  *DCDC_GetOtherSpeedCfgDesc (uint16_t *length);
static uint8_t  *DCDC_GetDeviceQualifierDescriptor (uint16_t *length);

static uint8_t  DCDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t epnum,
                                   uint8_t  *pbuff, uint16_t length);
static uint8_t  DCDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t epnum,
                                   uint8_t  *pbuff);

// DCDC interface class callbacks
USBD_ClassTypeDef  DCDC_cbs =
{
    DCDC_Init,
    DCDC_DeInit,
    DCDC_Setup,
    DCDC_EP0_TxSent,
    DCDC_EP0_RxReady,
    DCDC_DataIn,
    DCDC_DataOut,
    DCDC_SOF,
    NULL,
    NULL,
    DCDC_GetHSCfgDesc,
    DCDC_GetFSCfgDesc,
    DCDC_GetOtherSpeedCfgDesc,
    DCDC_GetDeviceQualifierDescriptor,
};

// DCDC Qualifier Descriptor
__ALIGN_BEGIN static uint8_t hUSBDeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
    USB_LEN_DEV_QUALIFIER_DESC,     /* bLength */
    USB_DESC_TYPE_DEVICE_QUALIFIER, /* bDescriptorType */
    0x00,                   /* bcdUSB */
    0x02,
    USBD_DEV_CLASS,     /* bDeviceClass */
    USBD_DEV_SUBCLASS,  /* bDeviceSubClass */
    USBD_DEV_PROTOCOL,  /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,          /* bMaxPacketSize0 */
    0x01,                   /* bNumConfigurations */
    0x00,                   /* bReserved */
};

// DCDC Configuration Descriptor
__ALIGN_BEGIN static uint8_t hUSBConfigDesc[] __ALIGN_END =
{
    /* Configuration Descriptor */
    USB_LEN_CFG_DESC,               /* bLength */
    USB_DESC_TYPE_CONFIGURATION,    /* bDescriptorType */
    DCDC_CONFIG_DESC_SIZE_LB,        /* wTotalLength */
    DCDC_CONFIG_DESC_SIZE_HB,
    0x04,   /* bNumInterfaces */
    0x01,   /* bConfigurationValue */
    0x00,   /* iConfiguration : Index of string descriptor describing the configuration */
    0xC0,   /* bmAttributes - Self Powered */
    0x32,   /* bMaxPower - 100 mA */

    /* Interface Association Descriptor */
    USB_LEN_IAD_DESC,   /* bLength */
    USB_DESC_TYPE_IAD,  /* bDescriptorType */
    0x00,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x02,   /* bFunctionClass: CDC */
    0x02,   /* bFunctionSubClass - Abstract Control Model */
    0x01,   /* bFunctionProtocol - Common AT commands */
    0x02,   /* iFunction */

    /* Interface Descriptor */
    USB_LEN_IF_DESC,            /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    0x00,   /* bInterfaceNumber */
    0x00,   /* bAlternateSetting */
    0x01,   /* bNumEndpoints - One endpoint used */
    0x02,   /* bInterfaceClass - CDC */
    0x02,   /* bInterfaceSubClass - Abstract Control Model */
    0x01,   /* bInterfaceProtocol - Common AT commands */
    0x00,   /* iInterface: */

    /* Header Functional Descriptor */
    0x05,   /* bLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x00,   /* bDescriptorSubtype */
    0x10,   /* bcdCDC */
    0x01,
    /* Call Managment Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x01,   /* bDescriptorSubtype */
    0x00,   /* bmCapabilities - D0+D1 */
    0x01,   /* bDataInterface - 1 */
    /* ACM Functional Descriptor */
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x02,   /* bDescriptorSubtype */
    0x02,   /* bmCapabilities */
    /* Union Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x06,   /* bDescriptorSubtype */
    0x00,   /* bMasterInterface - Communication Class Interface */
    0x01,   /* bSlaveInterface0 - Data Class Interface */
    /* EP2 Descriptor */
    USB_LEN_EP_DESC,        /* bLength */
    USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
    DCDC_P1_INTRIN_EP,     /* bEndpointAddress: (IN2) */
    0x03,           /* bmAttributes: Interrupt */
    DCDC_CMD_PACKET_SIZE,  /* wMaxPacketSize */
    0x00,
    0xFF,           /* bInterval: */
    /* Data class interface descriptor */
    USB_LEN_IF_DESC,            /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    0x01,   /* bInterfaceNumber */
    0x00,   /* bAlternateSetting */
    0x02,   /* bNumEndpoints - Two endpoints */
    0x0A,   /* bInterfaceClass - CDC */
    0x00,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x00,   /* iInterface */
    /* EP3 Descriptor */
    USB_LEN_EP_DESC,        /* bLength */
    USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
    DCDC_P1_BULKOUT_EP,    /* bEndpointAddress - (OUT3) */
    0x02,           /* bmAttributes - Bulk */
    LOBYTE(DCDC_DATA_PACKET_SIZE),  /* wMaxPacketSize */
    HIBYTE(DCDC_DATA_PACKET_SIZE),
    0x00,           /* bInterval - ignore for bulk transfer */
    /* EP1 Descriptor */
    USB_LEN_EP_DESC,        /* bLength */
    USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
    DCDC_P1_BULKIN_EP,     /* bEndpointAddress - (IN1) */
    0x02,           /* bmAttributes - Bulk */
    LOBYTE(DCDC_DATA_PACKET_SIZE),  /* wMaxPacketSize */
    HIBYTE(DCDC_DATA_PACKET_SIZE),
    0x00,           /* bInterval */

    /* Interface Association Descriptor */
    USB_LEN_IAD_DESC,   /* bLength */
    USB_DESC_TYPE_IAD,  /* bDescriptorType */
    0x02,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x02,   /* bFunctionClass: CDC */
    0x02,   /* bFunctionSubClass - Abstract Control Model */
    0x01,   /* bFunctionProtocol - Common AT commands */
    0x02,   /* iFunction */

    /* Interface Descriptor */
    USB_LEN_IF_DESC,            /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    0x02,   /* bInterfaceNumber */
    0x00,   /* bAlternateSetting */
    0x01,   /* bNumEndpoints - One endpoint used */
    0x02,   /* bInterfaceClass - CDC */
    0x02,   /* bInterfaceSubClass - Abstract Control Model */
    0x01,   /* bInterfaceProtocol - Common AT commands */
    0x00,   /* iInterface: */

    /* Header Functional Descriptor */
    0x05,   /* bLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x00,   /* bDescriptorSubtype */
    0x10,   /* bcdCDC */
    0x01,
    /* Call Managment Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x01,   /* bDescriptorSubtype */
    0x00,   /* bmCapabilities - D0+D1 */
    0x03,   /* bDataInterface - 3 */
    /* ACM Functional Descriptor */
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x02,   /* bDescriptorSubtype */
    0x02,   /* bmCapabilities */
    /* Union Functional Descriptor */
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType - CS_INTERFACE */
    0x06,   /* bDescriptorSubtype */
    0x02,   /* bMasterInterface - Communication Class Interface */
    0x03,   /* bSlaveInterface0 - Data Class Interface */
    /* EP2 Descriptor */
    USB_LEN_EP_DESC,        /* bLength */
    USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
    DCDC_P2_INTRIN_EP,     /* bEndpointAddress: (IN2) */
    0x03,           /* bmAttributes: Interrupt */
    DCDC_CMD_PACKET_SIZE,  /* wMaxPacketSize */
    0x00,
    0xFF,           /* bInterval: */
    /* Data class interface descriptor */
    USB_LEN_IF_DESC,            /* bLength */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType */
    0x03,   /* bInterfaceNumber */
    0x00,   /* bAlternateSetting */
    0x02,   /* bNumEndpoints - Two endpoints */
    0x0A,   /* bInterfaceClass - CDC */
    0x00,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x00,   /* iInterface */
    /* EP3 Descriptor */
    USB_LEN_EP_DESC,        /* bLength */
    USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
    DCDC_P2_BULKOUT_EP,    /* bEndpointAddress - (OUT3) */
    0x02,           /* bmAttributes - Bulk */
    LOBYTE(DCDC_DATA_PACKET_SIZE),  /* wMaxPacketSize */
    HIBYTE(DCDC_DATA_PACKET_SIZE),
    0x00,           /* bInterval - ignore for bulk transfer */
    /* EP1 Descriptor */
    USB_LEN_EP_DESC,        /* bLength */
    USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
    DCDC_P2_BULKIN_EP,     /* bEndpointAddress - (IN1) */
    0x02,           /* bmAttributes - Bulk */
    LOBYTE(DCDC_DATA_PACKET_SIZE),  /* wMaxPacketSize */
    HIBYTE(DCDC_DATA_PACKET_SIZE),
    0x00           /* bInterval */
};

/************************* Private ********************************************/
/* DCDC_Init
 * Initializes the DCDC interface
 */
static uint8_t  DCDC_Init (USBD_HandleTypeDef *pdev,
                           uint8_t cfgidx)
{
    pdev->pClassData = USBD_malloc(sizeof(DCDC_HandleTypeDef));
    if(pdev->pClassData == NULL)
    {
        return DCDC_FAIL;
    }

    if(pdev->dev_speed == USBD_SPEED_HIGH)
    {
        /* Open VCP1 EP IN */
        USBD_LL_OpenEP(pdev,
                       DCDC_P1_BULKIN_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_HS_IN_PACKET_SIZE);

        /* Open VCP1 EP OUT */
        USBD_LL_OpenEP(pdev,
                       DCDC_P1_BULKOUT_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_HS_OUT_PACKET_SIZE);

        /* Open VCP2 EP IN */
        USBD_LL_OpenEP(pdev,
                       DCDC_P2_BULKIN_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_HS_IN_PACKET_SIZE);

        /* Open VCP2 EP OUT */
        USBD_LL_OpenEP(pdev,
                       DCDC_P2_BULKOUT_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_HS_OUT_PACKET_SIZE);

    }
    else
    {
        /* Open VCP1 EP IN */
        USBD_LL_OpenEP(pdev,
                       DCDC_P1_BULKIN_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_FS_IN_PACKET_SIZE);

        /* Open VCP1 EP OUT */
        USBD_LL_OpenEP(pdev,
                       DCDC_P1_BULKOUT_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_FS_OUT_PACKET_SIZE);

        /* Open VCP2 EP IN */
        USBD_LL_OpenEP(pdev,
                       DCDC_P2_BULKIN_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_FS_IN_PACKET_SIZE);

        /* Open VCP2 EP OUT */
        USBD_LL_OpenEP(pdev,
                       DCDC_P2_BULKOUT_EP,
                       USBD_EP_TYPE_BULK,
                       DCDC_DATA_FS_OUT_PACKET_SIZE);
    }

    /* Open VCP1 Command IN EP */
    USBD_LL_OpenEP(pdev,
                   DCDC_P1_INTRIN_EP,
                   USBD_EP_TYPE_INTR,
                   DCDC_CMD_PACKET_SIZE);

    /* Open VCP2 Command IN EP */
    USBD_LL_OpenEP(pdev,
                   DCDC_P2_INTRIN_EP,
                   USBD_EP_TYPE_INTR,
                   DCDC_CMD_PACKET_SIZE);

    /* Init  physical Interface components */
    ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC1->Init();
    ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC2->Init();

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;

    /* Init Xfer states */
    hdls->hcdc1.TxState = 0;
    hdls->hcdc1.RxState = 0;
    hdls->hcdc2.TxState = 0;
    hdls->hcdc2.RxState = 0;

    /* Init Buffers */
    DCDC_SetRxBuffer(pdev, DCDC_P1_BULKOUT_EP, DCDC_RxBuf_P1);
    DCDC_SetTxBuffer(pdev, DCDC_P1_BULKIN_EP, DCDC_TxBuf_P1, 0);
    DCDC_SetRxBuffer(pdev, DCDC_P2_BULKOUT_EP, DCDC_RxBuf_P2);
    DCDC_SetTxBuffer(pdev, DCDC_P2_BULKIN_EP, DCDC_TxBuf_P2, 0);

    if(pdev->dev_speed == USBD_SPEED_HIGH)
    {
        /* Prepare VCP1 Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               DCDC_P1_BULKOUT_EP,
                               hdls->hcdc1.RxBuffer,
                               DCDC_DATA_HS_OUT_PACKET_SIZE);
        /* Prepare VCP2 Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               DCDC_P2_BULKOUT_EP,
                               hdls->hcdc2.RxBuffer,
                               DCDC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
        /* Prepare VCP1 Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               DCDC_P1_BULKOUT_EP,
                               hdls->hcdc1.RxBuffer,
                               DCDC_DATA_FS_OUT_PACKET_SIZE);
        /* Prepare VCP2 Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev,
                               DCDC_P2_BULKOUT_EP,
                               hdls->hcdc2.RxBuffer,
                               DCDC_DATA_FS_OUT_PACKET_SIZE);
    }

    return DCDC_OK;
}

/* DCDC_DeInit
 * DeInitializes the DCDC layer
 */
static uint8_t  DCDC_DeInit (USBD_HandleTypeDef *pdev,
                             uint8_t cfgidx)
{
    /* Flush and close VCP1 and VCP2 endpoints */
    USBD_LL_FlushEP(pdev, DCDC_P1_BULKIN_EP);
    USBD_LL_CloseEP(pdev, DCDC_P1_BULKIN_EP);
    USBD_LL_FlushEP(pdev, DCDC_P1_BULKOUT_EP);
    USBD_LL_CloseEP(pdev, DCDC_P1_BULKOUT_EP);
    USBD_LL_FlushEP(pdev, DCDC_P1_INTRIN_EP);
    USBD_LL_CloseEP(pdev, DCDC_P1_INTRIN_EP);
    USBD_LL_FlushEP(pdev, DCDC_P2_BULKIN_EP);
    USBD_LL_CloseEP(pdev, DCDC_P2_BULKIN_EP);
    USBD_LL_FlushEP(pdev, DCDC_P2_BULKOUT_EP);
    USBD_LL_CloseEP(pdev, DCDC_P2_BULKOUT_EP);
    USBD_LL_FlushEP(pdev, DCDC_P2_INTRIN_EP);
    USBD_LL_CloseEP(pdev, DCDC_P2_INTRIN_EP);

    /* DeInit  physical Interface components */
    if(pdev->pClassData != NULL)
    {
        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC1->DeInit();
        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC2->DeInit();
        USBD_free(pdev->pClassData);
        pdev->pClassData = NULL;
    }

    return DCDC_OK;
}

/* DCDC_Setup
 * Handle the setup requests
 */
static uint8_t  DCDC_Setup (USBD_HandleTypeDef *pdev,
                            USBD_SetupReqTypedef *req)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_CLASS :
        if (req->wLength)
        {
            if (req->bmRequest & 0x80)
            {
                switch(req->wIndex)
                {
                    case 2:
                        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC2->Control(req->bRequest,
                                                                                (uint8_t *)(hdls->hcdc2.data),
                                                                                req->wLength);
                        USBD_CtlSendData (pdev, (uint8_t *)(hdls->hcdc2.data),
                                            req->wLength);
                        break;

                    case 0:
                        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC1->Control(req->bRequest,
                                                                                 (uint8_t *)(hdls->hcdc1.data),
                                                                                req->wLength);
                          USBD_CtlSendData (pdev, (uint8_t *)(hdls->hcdc1.data),
                                            req->wLength);
                          break;

                    default:
                        break;
                }
            }
            else
            {
                switch(req->wIndex)
                {
                    case 2:
                        hdls->hcdc2.CmdOpCode = req->bRequest;
                        hdls->hcdc2.CmdLength = req->wLength;
                        USBD_CtlPrepareRx (pdev, (uint8_t *)(hdls->hcdc2.data),
                                           req->wLength);
                        break;

                    case 0:
                        hdls->hcdc1.CmdOpCode = req->bRequest;
                        hdls->hcdc1.CmdLength = req->wLength;
                        USBD_CtlPrepareRx (pdev, (uint8_t *)(hdls->hcdc1.data),
                                           req->wLength);
                        break;

                    default:
                        break;
                }
            }
        }
        else
        {
            switch(req->wIndex)
            {
                case 2:
                    ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC2->Control(req->bRequest,
                                                                             NULL, 0);
                      break;

                case 0:
                    ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC1->Control(req->bRequest,
                                                                             NULL, 0);
                      break;

                default:
                    break;
            }
        }

        break;

        default:
        break;
    }

    return USBD_OK;
}

/* DCDC_DataIn
 * Handle IN packets
 */
static uint8_t  DCDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_HandleTypeDef *hcdc;

    uint8_t ep_addr = epnum | 0x80;
    if (ep_addr == DCDC_P1_BULKIN_EP)
    {
        hcdc = &hdls->hcdc1;
    }
    else if (ep_addr == DCDC_P2_BULKIN_EP)
    {
        hcdc = &hdls->hcdc2;
    }
    else
    {
        return USBD_FAIL;
    }

    hcdc->TxState = 0;
    return USBD_OK;
}

/* DCDC_SOF
 * Handle SOF indication
 */
static uint8_t  DCDC_SOF (USBD_HandleTypeDef *pdev)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    return USBD_OK;
}

/* DCDC_DataOut
 * Handle OUT packets
 */
static uint8_t  DCDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_HandleTypeDef *hcdc;

    if (epnum == DCDC_P1_BULKOUT_EP)
    {
        hcdc = &hdls->hcdc1;
        /* Get the received data length */
        hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
        /* USB data will be immediately processed, this allow next USB traffic being
        NAKed till the end of the application Xfer */
        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC1->Receive(hcdc->RxBuffer, &hcdc->RxLength);
        /* Prepare VCP1 Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev, DCDC_P1_BULKOUT_EP, hcdc->RxBuffer,
                               DCDC_DATA_HS_OUT_PACKET_SIZE);
    }
    else if (epnum == DCDC_P2_BULKOUT_EP)
    {
        hcdc = &hdls->hcdc2;
        /* Get the received data length */
        hcdc->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
        /* USB data will be immediately processed, this allow next USB traffic being
        NAKed till the end of the application Xfer */
        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC2->Receive(hcdc->RxBuffer, &hcdc->RxLength);
        /* Prepare VCP2 Out endpoint to receive next packet */
        USBD_LL_PrepareReceive(pdev, DCDC_P2_BULKOUT_EP, hcdc->RxBuffer,
                               DCDC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
        return USBD_FAIL;
    }

    return USBD_OK;
}

/* DCDC_EP0_TxSent
 * Handles EP0 Tx complete indication
 */
static uint8_t  DCDC_EP0_TxSent (USBD_HandleTypeDef *pdev)
{
    return USBD_OK;
}

/* DCDC_EP0_RxReady
 * Handles data received indication on EP0
 */
static uint8_t  DCDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;

    if((pdev->pUserData != NULL) && (hdls->hcdc1.CmdOpCode != 0xFF))
    {
        ((DCDC_ItfTypeDef *)pdev->pUserData)->CDC1->Control(hdls->hcdc1.CmdOpCode,
                                                            (uint8_t *)(hdls->hcdc1.data),
                                                            hdls->hcdc1.CmdLength);
          hdls->hcdc1.CmdOpCode = 0xFF;
    }

    return USBD_OK;
}

/* DCDC_GetFSCfgDesc
 * Return FS configuration descriptor
 */
static uint8_t  *DCDC_GetFSCfgDesc (uint16_t *length)
{
    *length = sizeof (hUSBConfigDesc);
    return hUSBConfigDesc;
}

/* DCDC_GetHSCfgDesc
 * Return HS configuration descriptor
 */
static uint8_t  *DCDC_GetHSCfgDesc (uint16_t *length)
{
    *length = sizeof (hUSBConfigDesc);
    return hUSBConfigDesc;
}

/* DCDC_GetOtherSpeedCfgDesc
 * Return other speed configuration descriptor
 */
static uint8_t  *DCDC_GetOtherSpeedCfgDesc (uint16_t *length)
{
    *length = sizeof (hUSBConfigDesc);
    return hUSBConfigDesc;
}

/* DCDC_GetDeviceQualifierDescriptor
 * Return Device Qualifier descriptor
 */
static uint8_t  *DCDC_GetDeviceQualifierDescriptor (uint16_t *length)
{
    *length = sizeof (hUSBDeviceQualifierDesc);
    return hUSBDeviceQualifierDesc;
}

/* DCDC_SetTxBuffer
 * Assigns Tx buffer and Tx length
 */
static uint8_t  DCDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t epnum,
                                   uint8_t  *pbuff,
                                   uint16_t length)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_HandleTypeDef *hcdc;

    if (epnum == DCDC_P1_BULKIN_EP)
    {
        hcdc = &hdls->hcdc1;
    }
    else if (epnum == DCDC_P2_BULKIN_EP)
    {
        hcdc = &hdls->hcdc2;
    }
    else
    {
        return USBD_FAIL;
    }

    hcdc->TxBuffer = pbuff;
    hcdc->TxLength = length;
    return USBD_OK;
}

/* DCDC_SetRxBuffer
 * Assigns the Rx buffer
 */
static uint8_t  DCDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t epnum,
                                   uint8_t  *pbuff)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_HandleTypeDef *hcdc;

    if (epnum == DCDC_P1_BULKOUT_EP)
    {
        hcdc = &hdls->hcdc1;
    }
    else if (epnum == DCDC_P2_BULKOUT_EP)
    {
        hcdc = &hdls->hcdc2;
    }
    else
    {
        return USBD_FAIL;
    }

    hcdc->RxBuffer = pbuff;
    return USBD_OK;
}

/* DCDC_TransmitPacket
 * Transmits a packet over an endpoint
 */
static uint8_t DCDC_TransmitPacket(USBD_HandleTypeDef *pdev,
                                   uint8_t ep_addr)
{
    if(pdev->pClassData == NULL)
    {
        return USBD_FAIL;
    }

    DCDC_HandleTypeDef *hdls = (DCDC_HandleTypeDef*) pdev->pClassData;
    USBD_CDC_HandleTypeDef *hcdc;

    if (ep_addr == DCDC_P1_BULKIN_EP)
    {
        hcdc = &hdls->hcdc1;
    }
    else if (ep_addr == DCDC_P2_BULKIN_EP)
    {
        hcdc = &hdls->hcdc2;
    }
    else
    {
        return USBD_FAIL;
    }

    if(hcdc->TxState == 0)
    {
        /* Tx Transfer in progress */
        hcdc->TxState = 1;

        /* Transmit next packet */
        USBD_LL_Transmit(pdev,
                         ep_addr,
                         hcdc->TxBuffer,
                         hcdc->TxLength);

        return USBD_OK;
    }
    else
    {
        return USBD_BUSY;
    }
}

/************************** Public ********************************************/
/* DCDC_RegisterInterface
 * Registers fops for a CDC port
 */
uint8_t  DCDC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                  DCDC_ItfTypeDef *fops)
{
    if(fops != NULL)
    {
        pdev->pUserData= fops;
        return USBD_OK;
    }
    else
    {
        return USBD_FAIL;
    }
}

/* DCDC_TransmitData
 * Transmits data over a VCP Port
 */
uint8_t DCDC_TransmitData(uint8_t com_port,
                          uint8_t *tx_buf,
                          uint16_t tx_len)
{
    uint8_t epnum = 0;
    uint8_t* buf = 0;

    if(tx_buf == NULL)
    {
        return USBD_FAIL;
    }

    /* Get endpoint from port number */
    if(com_port == DCDC_PORT1)
    {
        epnum = DCDC_P1_BULKIN_EP;
        buf = DCDC_TxBuf_P1;
        if(tx_len > sizeof(DCDC_TxBuf_P1)) return USBD_FAIL;
    }
    else if(com_port == DCDC_PORT2)
    {
        epnum = DCDC_P2_BULKIN_EP;
        buf = DCDC_TxBuf_P2;
        if(tx_len > sizeof(DCDC_TxBuf_P2)) return USBD_FAIL;
    }
    else
    {
        return USBD_FAIL;
    }

    /* Set the Tx Buffer */
    memcpy(buf, tx_buf, tx_len);
    uint8_t ret = DCDC_SetTxBuffer(&USBDevice, epnum, buf, tx_len);
    /* Activate Tx */
    if(ret == USBD_OK)
    {
        ret = DCDC_TransmitPacket(&USBDevice, epnum);
    }

    return ret;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

