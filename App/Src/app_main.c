#include "app_main.h"

#include "can.h"
#include "main.h"

#include "i2c.h"
#include "stm32f1xx_hal.h"

#include "lcd.h"
#include "lcd_driver.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUF_SIZE 128

static CAN_TxHeaderTypeDef s_tx_header;
static CAN_RxHeaderTypeDef s_rx_header;

static char s_tx_data[BUF_SIZE];
static char s_rx_data[BUF_SIZE];
static uint32_t s_tx_mailbox;

volatile uint32_t msg_counter = 0;

float temperature;
float humidity;
float pressure;

char temp_string[50];
char hum_string[50];
char press_string[50];

static volatile uint8_t s_data_received = 0;
static char s_usb_buf[BUF_SIZE * 2];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_header,
                           (uint8_t *)s_rx_data) != HAL_OK) {
    // Error_Handler();
  }

  s_data_received = 1;
  msg_counter++;
}

void user_delay_ms(uint32_t period) { HAL_Delay(period); }

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data,
                     uint16_t len) {
  if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 10) != HAL_OK)
    return -1;
  if (HAL_I2C_Master_Receive(&hi2c1, (id << 1) | 0x01, data, len, 10) != HAL_OK)
    return -1;

  return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data,
                      uint16_t len) {
  int8_t *buf;
  buf = malloc(len + 1);
  buf[0] = reg_addr;
  memcpy(buf + 1, data, len);

  if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), (uint8_t *)buf, len + 1,
                              HAL_MAX_DELAY) != HAL_OK)
    return -1;

  free(buf);
  return 0;
}

void app_main() {
  // main
  volatile HAL_StatusTypeDef rc_can = HAL_OK;
  uint32_t id = HAL_GetUIDw0();
  const char msg[] = "Hello from %d";

  CAN_FilterTypeDef s_filter_config;
  sprintf(s_tx_data, msg, id);

  s_filter_config.FilterBank = 0;
  s_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  s_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  s_filter_config.FilterIdHigh = 0x0000;
  s_filter_config.FilterIdLow = 0x0000;
  s_filter_config.FilterMaskIdHigh = 0x0000;
  s_filter_config.FilterMaskIdLow = 0x0000;
  s_filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  s_filter_config.FilterActivation = ENABLE;
  s_filter_config.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &s_filter_config) != HAL_OK) {
    Error_Handler();
  }

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

  do {
    rc_can = HAL_CAN_AddTxMessage(&hcan, &s_tx_header, (uint8_t *)s_tx_data,
                                  &s_tx_mailbox);
  } while (rc_can != HAL_OK);

  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  while (1) {

    if (s_data_received) {
      sprintf(s_usb_buf, "Received: %s\r\n", s_rx_data);
      HAL_CAN_AddTxMessage(&hcan, &s_tx_header, (uint8_t *)s_tx_data,
                           &s_tx_mailbox);
      s_data_received = 0;

      HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
    }

    HAL_Delay(500);
  }

  return;
}
