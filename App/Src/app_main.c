#include "app_main.h"

#include "can.h"
#include "main.h"

#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BUF_SIZE 128

static CAN_TxHeaderTypeDef s_tx_header;
static CAN_RxHeaderTypeDef s_rx_header;

static char s_tx_data[BUF_SIZE];
static char s_rx_data[BUF_SIZE];
static uint32_t s_tx_mailbox;

static volatile uint8_t s_data_received = 0;
static char s_usb_buf[BUF_SIZE * 2];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_header,
                           (uint8_t *)s_rx_data) != HAL_OK) {
    Error_Handler();
  }

  s_data_received = 1;
}

void app_main() {
  // main

  uint32_t id = HAL_GetUIDw0();
  const char msg[] = "Hello from %d";

  sprintf(s_tx_data, msg, id);

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    Error_Handler();
  }

  s_tx_header.StdId = id;
  s_tx_header.ExtId = 0x00;
  s_tx_header.RTR = CAN_RTR_DATA;
  s_tx_header.IDE = CAN_ID_STD;
  s_tx_header.DLC = strlen(s_tx_data);
  s_tx_header.TransmitGlobalTime = DISABLE;

  HAL_Delay(1000);

  HAL_CAN_AddTxMessage(&hcan, &s_tx_header, (uint8_t *)s_tx_data,
                       &s_tx_mailbox);

  while (1) {
    HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);

    if (s_data_received) {
      sprintf(s_usb_buf, "Received: %s\r\n", s_rx_data);
      CDC_Transmit_FS((uint8_t *)s_usb_buf, strlen(s_usb_buf));
      HAL_CAN_AddTxMessage(&hcan, &s_tx_header, (uint8_t *)s_tx_data,
                           &s_tx_mailbox);
      s_data_received = 0;
    }

    HAL_Delay(500);
  }

  return;
}