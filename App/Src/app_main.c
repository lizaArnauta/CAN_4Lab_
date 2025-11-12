#include "app_main.h"

#include "can.h"
#include "main.h"

#include "i2c.h"
#include "stm32f1xx_hal.h"

#include "lcd.h"
#include "lcd_driver.h"

#include "bme280.h"

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

float temperature;
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char temp_string[50];
char hum_string[50];
char press_string[50];

static volatile uint8_t s_data_received = 0;
static char s_usb_buf[BUF_SIZE * 2];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &s_rx_header,
                           (uint8_t *)s_rx_data) != HAL_OK) {
    Error_Handler();
  }

  s_data_received = 1;
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

  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  rslt = bme280_init(&dev);

  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;
  rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
                                        BME280_OSR_HUM_SEL | BME280_FILTER_SEL,
                                    &dev);

  HAL_Delay(1000);

  HAL_CAN_AddTxMessage(&hcan, &s_tx_header, (uint8_t *)s_tx_data,
                       &s_tx_mailbox);

  ILI9341_Init();

  ILI9341_FillScreen(WHITE);
  ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
  ILI9341_DrawText("HELLO WORLD", FONT4, 90, 110, BLACK, WHITE);
  HAL_Delay(1000);

  while (1) {
    HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);

    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    HAL_Delay(40);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    if (rslt == BME280_OK) {
      temperature = comp_data.temperature / 100.0;
      humidity = comp_data.humidity / 1024.0;
      pressure = comp_data.pressure / 10000.0;

      sprintf(temp_string, "Temperature: %03.1f C", temperature);
      sprintf(hum_string, "Humidity: %03.1f %%", humidity);
      sprintf(press_string, "Pressure: %03.1f hPa", pressure);
    }

    if (s_data_received) {
      sprintf(s_usb_buf, "Received: %s\r\n", s_rx_data);
      HAL_CAN_AddTxMessage(&hcan, &s_tx_header, (uint8_t *)s_tx_data,
                           &s_tx_mailbox);
      s_data_received = 0;
    }

    HAL_Delay(500);
  }

  return;
}