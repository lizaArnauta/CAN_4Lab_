#include "app_main.h"
#include "can.h"
#include "stm32f1xx_hal_can.h"
#include <stdint.h>

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t datacheck;

uint32_t TxMailbox = 0;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);

  if (RxHeader.DLC == 2) {
    datacheck = 1;
  }
}

void app_main() {
  HAL_CAN_Start(&hcan);

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  TxHeader.DLC = 2;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x103;

  TxData[0] = 50;
  TxData[1] = 20;

  while (1) {
    if (datacheck) {
      for (int i = 0; i < RxData[1]; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(RxData[0]);
      }

      datacheck = 0;

      HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    }
  }
}