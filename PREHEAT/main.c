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

int main(void)
	{
	void Delay(uint32_t nCount); //Funci�n b�sica para generar un delay.

    declarar_leds();

	declarar_boton();


    while (1)
    	{
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
    	}
	}


void Delay(uint32_t nCount)
	{
	while(nCount--)
		{
		}
	}
