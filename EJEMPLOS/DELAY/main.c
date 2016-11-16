/**
  ******************************************************************************
  * @file    DELAY/main.c
  * @author  German Feres - UTNFRBB-TDII
  * @version V1.0.0
  * @date    1-Mayo-2014
  * @brief   Este ejemplo utiliza el Systick para generar un retardo
  * 		 de tiempo real
  ******************************************************************************


  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

uint32_t time_ms = 0;


static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);

void PORT_Init(void);

int main(void)
	{

	SystemInit(); //Fuerza a que tome la configuracion del CLOCK de system_stm32f4.c
	PORT_Init();  //Inicializamos los pines donde estan conectados los LEDs

	//Se configura el SysTick para que interrumpa cada 1 ms.
	//La base de tiempo del SysTick es el HCLK o clock del sistema
	//El valor colocado como parametro en la funcion SysTick_Config
	//indica cuantos ciclos de HCLK contará para generar una interrupción
	// Ejemplo:
	// HCLK= 168MHz
	// Requerimiento= 1 mseg
	// 		1seg --- 168 Mticks
	// 		1ms ---- x
	// 		x = 168.000 ticks
	// 		para lograr este valor divido 168 M ticks / 1000 = 168.000 ticks

	SysTick_Config(SystemCoreClock / 1000);


	//Lazo principal

    while(1)
    {

    	GPIO_SetBits(GPIOD,GPIO_Pin_12);
    	/* Inserta un retardo de 50 ms */
    	Delay(500);
    	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
    	Delay(500);

    }



	}



//Funcion que inicializa los pines 12, 13, 15 y 15 del puerto D, los cuales son los
//pines en donde estan conectados los LEDs

void PORT_Init()
	{
	GPIO_InitTypeDef GPIO_InitStructure; //Estructura de configuracion

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//Habilitacion de la senal de
															//reloj para el periferico GPIOD

	//Enmascaramos los pines que usaremos

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_12 |
									GPIO_Pin_13 |
									GPIO_Pin_14 |
									GPIO_Pin_15;

	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;		//Los pines seleccionados como salida
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;		//Tipo de salida como push pull
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;	//Velocidad del clock para el GPIO
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;		//Sin pull ups

	GPIO_Init(GPIOD, &GPIO_InitStructure); //Se aplica la configuracion definidas anteriormente
	}





/**
  * @brief  Aplica un retardo de tiempo real
  * @param  nTime: es el retardo en mseg
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrementa el valor de TimingDelay. Esta función
  * es llamada por la rutina de atención del SysTick
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0)
  {
    TimingDelay--;
  }
}

