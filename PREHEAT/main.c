/** PROGRAMA PRINCIPAL PRE-CALENTADOR DE PCB
 *  Integrantes: Marano Garcia
 *
 *
 *  Adelante y suerte
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "PHinclude.h"
#include "adc.h"
#include "LCD/include/stm32_ub_lcd_2x16.h"
#include "stdio.h"

void Delay(__IO uint32_t nTime);  //funcion Delay que usa SysTick
int devolver_temperatura_en_grados();

#define MAX_ADC	4095.0    // resolucion de ADC 12bit

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

	SysTick_Config(SystemCoreClock / 1000);

	// Ejemplo:
	// HCLK= 168MHz
	// Requerimiento= 1 mseg
	// 		1seg --- 168 Mticks
	// 		1ms ---- x
	// 		x = 168.000 ticks
	// 		para lograr este valor divido 168 M ticks / 1000 = 168.000 ticks

	char stringtemperatura[4]; // String donde se guarda la temperatura

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

	temperatura=((temperatura*300)/4095);  // de Tension de ADC a Grados centigrados

	return temperatura;
}


void color_segun_temperatura()
{
				//GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); //restart todos los led

				int32_t temperaturaengrados=devolver_temperatura_en_grados();

	    	    if(temperaturaengrados<27)   //menor de 27 grados enciende el led VERDE
	    	    {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	    	    	GPIO_ResetBits(GPIOD,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	    	    }

	    	    if(temperaturaengrados>=27) //Mayor o igual a  27 grados enciende el led ROJO
	    	       {
	    	    	GPIO_SetBits(GPIOD,GPIO_Pin_13);
	    	       	GPIO_ResetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15);
	    	       }
}
