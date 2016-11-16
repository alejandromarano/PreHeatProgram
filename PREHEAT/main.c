/** PROGRAMA PRINCIPAL PRE-CALENTADOR DE PCB
 *  Integrantes: Marano Garcia
 *
 *
 *  Adelante y suerte
 */

/**
  ******************************************************************************
  * @file    main.c
  * @author  German Feres - TDII
  * @version
  * @date    01-Mayo-2014
  * @brief   Inicializa el port D pin 12,13,14,15 como salidas
  * 		 y el port A pin 0 como entrada
  *
  * LEDS USER
  * led3 user naranja puerto I/O PD13
  * led4 user verde puerto I/O PD12
  * led5 user rojo puerto I/O PD14
  * led6 user azul puerto I/O PD15
  *
  * USER Button
  * PA0


  ******************************************************************************
 **/

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "PHinclude.h"
#include "adc.h"
#include "LCD/include/stm32_ub_lcd_2x16.h"
#include "stdio.h"

void Delay(__IO uint32_t nTime);  //funcion Delay que usa SysTick

#define MAX_ADC	4095.0

int32_t adc_valor_obtenido;

uint32_t time_ms = 0;

static __IO uint32_t TimingDelay;


int main(void)
	{

	SystemInit(); // inicializa el sistema

	UB_LCD_2x16_Init(); // inicializa el display 16x2

    declarar_leds();     // GPIO leds pin 11 12 13 14

	declarar_boton();    // GPIO boton

	adc_inicializar();   // Inicializa ADC polling

	SysTick_Config(SystemCoreClock / 1000);   // Ejemplo:

	// HCLK= 168MHz
	// Requerimiento= 1 mseg
	// 		1seg --- 168 Mticks
	// 		1ms ---- x
	// 		x = 168.000 ticks
	// 		para lograr este valor divido 168 M ticks / 1000 = 168.000 ticks

	char temperatura[4]; // String donde se guarda la temperatura

    while (1)
    	{

    	Delay(500);

    	adc_valor_obtenido=adc_leer_cuentas();   // Lee el ADC de la funcion adc.h y PHinclude

    	adc_valor_obtenido=((adc_valor_obtenido*300)/4095);  // de Tension de ADC a Grados centigrados

    	sprintf(temperatura,"%d",adc_valor_obtenido);   // pasa de un entero a un String para imprimir

    	UB_LCD_2x16_String(0,1,temperatura);    // usa una funcion ya definida para imprimir un string
    	Delay(500);
    	UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string


    	    GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

    	    if(adc_valor_obtenido<30)   //menor de 30 grados enciende el led VERDE
    	    {
    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12);
    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    	    }

    	    if(adc_valor_obtenido>=30) //Mayor o igual a  30 grados enciende el led ROJO
    	       {
    	    	GPIO_SetBits(GPIOD,GPIO_Pin_13);
    	       	GPIO_ResetBits(GPIOD,GPIO_Pin_14|GPIO_Pin_15);
    	       }

    	    Delay(500);

    	    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    	}
	}


void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
}

