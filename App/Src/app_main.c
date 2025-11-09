#include "app_main.h"
#include "lcd_driver.h"
#include "lcd.h"

void app_main() {
  // main

  ILI9341_FillScreen(WHITE);
  ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
  ILI9341_DrawText("HELLO WORLD", FONT4, 90, 110, BLACK, WHITE);
  HAL_Delay(1000);

  return;
}
