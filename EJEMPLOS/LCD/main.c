//--------------------------------------------------------------
// File     : main.c
// Datum    : 17.04.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : CMSIS_BOOT, M4_CMSIS_CORE
// Funktion : Demo der LCD-2x16-Library
// Hinweis  : Diese zwei Files muessen auf 8MHz stehen
//              "cmsis_boot/stm32f4xx.h"
//              "cmsis_boot/system_stm32f4xx.c"
//--------------------------------------------------------------

#include "main.h"
#include "stm32_ub_lcd_2x16.h"

int main(void)
{
  SystemInit(); // Quarz Einstellungen aktivieren

  // Init vom LC-Display
  UB_LCD_2x16_Init();

  // Text auf Zeile-1 ausgeben
  UB_LCD_2x16_String(0,0,"LCD 2x16 STM32F4");

  // Text auf Zeile-2 ausgeben
  UB_LCD_2x16_String(3,1,"Discovery");

  while(1)
  {

  }
}

