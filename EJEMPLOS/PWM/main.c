/* SE BUSCA IMPLEMENTAR UN PID CON EL DUTY DEL PWM
 *
 * EJEMPLOS Y LIB:
 * https://stm32f4-discovery.net/2014/11/project-03-stm32f4xx-pid-controller/
 * http://mazsola.iit.uni-miskolc.hu/~drdani/docs_arm/st/fsfatdemo4disc/arm_math.h
 */


/*
  * NOTA: este ejemplo es una adaptación de uno brindado por ST en su página
  * oficial: http://www.st.com/web/en/catalog/tools/PF257904
  * En el mismo se configura el TIMER 3 en modo PWM, para que genere
  * 4 canales PWM
  * CH1 (PC6)
  * CH2 (PC7)
  * CH3 (PB0)
  * CH4 (PB1)
  * La frecuencia de la señal generada es de 56KHz con los siguientes anchos
  * de pulso:
  *
  *  TIM3 Channel1 duty cycle =  50%
  *  TIM3 Channel2 duty cycle =  37.5%
  *  TIM3 Channel3 duty cycle =  25%
  *  TIM3 Channel4 duty cycle =  12.5%
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
 /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.

    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1.
      TIM3CLK = 2 * PCLK1
      PCLK1 = HCLK / 4
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2  --> 84MHz

    Para obtener el clock del contador TIM3 de 28MHz, ajusto el prescaler con la siguiente ecuación:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /28 MHz) - 1

    Pra obtener el clock de salida del TIM3 de 56 KHz, el preriodo (ARR) se calcula de la siguiente manera:
        ARR = (TIM3 counter clock / TIM3 output clock) - 1
           = 499

    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.
  ----------------------------------------------------------------------- */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "cmsis_lib/include/PID_FUNCIONES.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

TIM_OCInitTypeDef  TIM_OCInitStructure;

uint16_t duty = 0; //   DUTY !!! (ciclo de trabajo)

uint16_t PrescalerValue = 0;

#define TEMP_CURRENT        temps[1]    /* Temperatura que tengo */
#define TEMP_WANT            temps[0]    /* Temperatura que quiero */

/* Parametros PID PARA UN FOCO */
#define PID_PARAM_KP        100            /* Proporcional */
#define PID_PARAM_KI        0.025        /* Integral */
#define PID_PARAM_KD        20            /* Derivative */

/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);

int main(void)
{
	/* Timer data for PWM */
//	    TM_PWM_TIM_t TIM_Data;
	    char buf[150];
	    uint8_t devices, i, count;
	    /* DS18B20 devices ID */
	    uint8_t device[EXPECTING_SENSORS][8];
	    /* Temperatures from 2 sensors */
	    float temps[EXPECTING_SENSORS];
	    /* PID error */
	    float pid_error;
	    /* Duty cycle for PWM */
	    float duty = 0;
	    /* ARM PID Instance, float_32 format */
	    arm_pid_instance_f32 PID;
	    /* One wire instance */
	    TM_OneWire_t OneWire;

	    /* Set PID parameters */
	    /* Set this for your needs */
	    PID.Kp = PID_PARAM_KP;        /* Proporcional */
	    PID.Ki = PID_PARAM_KI;        /* Integral */
	    PID.Kd = PID_PARAM_KD;        /* Derivative */

	    /* Initialize PID system, float32_t format */
	    arm_pid_init_f32(&PID, 1);


  SystemInit(); //Fuerza a que tome la configuracion del CLOCK de system_stm32f4.c

  /* TIM Configuration */
  TIM_Config();

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2800) - 1; // 28000000

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 499; //499
  TIM_TimeBaseStructure.TIM_Prescaler = 2000;
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

  while (1)
  {}
}

/**
  * @brief  Configure the TIM3 Ouput Channels.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
