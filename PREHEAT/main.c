/** PROGRAMA PRINCIPAL PRE-CALENTADOR DE PCB
 *  Integrantes: Marano Garcia
 *
 *LO IMPLEMENTADO:
 *-DISPLAY 16X2
 *-LM35 (1)
 *
 *SE BUSCA IMPLEMENTAR:
 *-PID (NADA MENOS :P )
 *
 *  Adelante y suerte
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "cmsis_lib/include/stm32f4xx_tim.h"   //lib timers
#include "PHinclude.h"       //lib propia de funciones
#include "adc.h"             // lib de ADC
#include "LCD/include/stm32_ub_lcd_2x16.h"  // lib LCD 16X2
#include "cmsis_lib/include/PID_FUNCIONES.h"  // lib adaptacion de arm_math... solo funciones PID
#include "stdio.h"

#define PID_PARAM_KP		100			/* Proporcional */  //PARAMETROS PID  //1
#define PID_PARAM_KI		0.025		/* Integral */                        //0.05
#define PID_PARAM_KD		20			/* Derivative */                      //0.25

void Delay(__IO uint32_t nTime);  //funcion Delay que usa SysTick
int devolver_temperatura_en_grados(); // funcion PHinclude
void color_segun_temperatura();
void TIM_Config(void);

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //variable para el timer y PWM
TIM_OCInitTypeDef  TIM_OCInitStructure;   //variable para el timer y PWM

uint16_t duty = 0; //   DUTY !!! (ciclo de trabajo)
uint16_t PrescalerValue = 0;

#define MAX_ADC	4095.0    // resolucion de ADC 12bit

int32_t adc_valor_obtenido;  // lectura del ADC
uint32_t time_ms = 0;

static __IO uint32_t TimingDelay;

int main(void)
	{

	SystemInit(); // inicializa el sistema

	UB_LCD_2x16_Init(); // inicializa el display 16x2

    declarar_leds();     // GPIO leds pin 11 12 13 14

	//declarar_boton();    // GPIO boton

	adc_inicializar();   // Inicializa ADC polling

	SysTick_Config(SystemCoreClock / 1000);

	// Ejemplo:
	// HCLK= 168MHz
	// Requerimiento= 1 mseg
	// 		1seg --- 168 Mticks
	// 		1ms ---- x
	// 		x = 168.000 ticks
	// 		para lograr este valor divido 168 M ticks / 1000 = 168.000 ticks

	char stringtemperatura[4]; // String donde se guarda la temperatura

	 /* Compute the prescaler value */
	  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2800) - 1; // 28000000

	//PWM PWM PWM PWM PWM PWM PWM PWM PMW
	/* Time base configuration */
	  TIM_TimeBaseStructure.TIM_Period = 499; //499
	  TIM_TimeBaseStructure.TIM_Prescaler =100000 ;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	  /* PWM1 Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = duty;                         // DUTY !!!
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  TIM_ARRPreloadConfig(TIM3, ENABLE);

	  /* TIM3 enable counter */
	  TIM_Cmd(TIM3, ENABLE);
	//PWM PWM PWM PWM PWM PWM PWM PWM PMW

	while (1)
    	{

    	sprintf(stringtemperatura,"%d",devolver_temperatura_en_grados());   // pasa de un entero a un String para imprimir

    	UB_LCD_2x16_Clear();                    //usa una funcion ya definida para limpiar las string
    	UB_LCD_2x16_String(0,0,"Temp actual:");
    	UB_LCD_2x16_String(0,1,stringtemperatura);    // usa una funcion ya definida para imprimir un string
    	UB_LCD_2x16_String(3,1,"\176");
    	Delay(250);

    	//color_segun_temperatura();

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

int devolver_temperatura_en_grados()
{
	int32_t temperatura=0;

	temperatura=adc_leer_cuentas();   // Lee el ADC de la funcion adc.h y PHinclude

	temperatura=((temperatura*3000)/4095);  // de Tension de ADC a Grados centigrados

	return temperatura;
}


void color_segun_temperatura()
{
				//GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); //restart todos los led

				int32_t temperaturaengrados=devolver_temperatura_en_grados();

	    	    if(temperaturaengrados<270)   //menor de 27 grados enciende el led VERDE
	    	    {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	    	    }

	    	    if(temperaturaengrados>=270) //Mayor o igual a  27 grados enciende el led ROJO
	    	       {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	    	       	GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15);
	    	       }
}
