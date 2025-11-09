#include "app_main.h"
#include "main.h"
#include "stm32f1xx_hal.h"

void app_main() {
  // main

  while (1) {
    HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
    HAL_Delay(500);
  }

  return;
}