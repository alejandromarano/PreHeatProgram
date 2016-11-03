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

#define MAX_ADC	4095.0

int32_t adc_valor_obtenido;

int main(void)
	{

	int_fast8_t menu=0;

	SystemInit();

	UB_LCD_2x16_Init();

    declarar_leds();

	declarar_boton();

	adc_inicializar();

    while (1)
    	{

    	Delay(100000);

    	adc_valor_obtenido=adc_leer_cuentas();

    	    GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

    	    if(adc_valor_obtenido<MAX_ADC/4 && adc_valor_obtenido>0.05 * MAX_ADC)
    	    {
    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12);
    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    	    }

    	    if(adc_valor_obtenido>MAX_ADC/4 && adc_valor_obtenido<MAX_ADC/2)
    	       {
    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13);
    	       	GPIO_ResetBits(GPIOD,GPIO_Pin_14|GPIO_Pin_15);
    	       }

    	    if(adc_valor_obtenido>MAX_ADC/2 && adc_valor_obtenido<MAX_ADC*3/4)
    	       {
    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
    	        GPIO_ResetBits(GPIOD,GPIO_Pin_15);
    	       }
    	    if(adc_valor_obtenido>MAX_ADC*3/4)
    	       {
    	       	GPIO_SetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    	       }


/*
		GPIO_SetBits(GPIOD, GPIO_Pin_12);	//PD12 encendido
		Delay(0x0FFFFF);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);	//PD13 encendido
		Delay(0x0FFFFF);


        if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
        	{
        	GPIO_SetBits(GPIOD, GPIO_Pin_14);	//PD14 encendido
        	Delay(0x0FFFFF);
        	GPIO_SetBits(GPIOD, GPIO_Pin_15);	//PD15 encendido
        	Delay(0x0FFFFF);
        	}

		//Se apagan todos los LEDs simultaneamente

		GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

		Delay(0x0FFFFF);
*/
    	 menu=GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

    	 if(menu){
    	 UB_LCD_2x16_String(0,0,"Pre-Calentador de PCB");    // usa una funcion ya definida para imprimir un string
		 Delay(10000000);
   	     UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string
	     UB_LCD_2x16_String(0,0,"lo mejor!");
	     Delay(10000000);                        // produce delay para poder ver los switcheos de string
	     UB_LCD_2x16_Clear();
    	 }
    	}
	}
