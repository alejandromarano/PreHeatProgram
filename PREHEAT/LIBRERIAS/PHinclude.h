/**
 * Include para funciones
 */

void declarar_leds(void)
{
		GPIO_InitTypeDef GPIO_InitStructure; //Estructura para la inicialización de los GPIO

	    //Habilitación de la senal de reloj para el periferico GPIOD (el mismo esta conectado al bus
	    //AHB1), el cual es el puerto al cual estan conectado los LEDs de usuario

	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	    //Se configuran los pines PD12, 13, 14 y 15

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

		GPIO_Init(GPIOD, &GPIO_InitStructure);	//Se aplica la configuración definida anteriormente
												//al puerto D

}

void declarar_boton(void)
{
	    GPIO_InitTypeDef GPIO_InitStructure; //Estructura para la inicialización de los GPIO
	

		//Habilitación de la señal de reloj para el periferico GPIOA (el mismo esta conectado al bus
		//AHB1), el cual es el puerto al cual esta conectado el pulsador de usuario.
	    
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	    //Ahora se configura el pin PA0, el cual es el pin al cual esta conectado el pulsador
		//denominado "User" (pulsador azul)

	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Entrada
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

		//Nota: Se puede comprobar que si en lugar de:

		//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL

		//Se utiliza:

		//GPIO_PuPd_UP		Es como si el pulsador estuviese siempre pulsado
		//GPIO_PuPd_DOWN	Sin efecto

		//Esto se debe a que el pin P0 del GPIOA tiene un pull-down en el mismo kit, por lo que
		//configrar el pin sin pull-up / down o con pull-down es lo mismo. Distinto es claro esta
		//configurarlo con pull-up que hace que siempre este un 1 presente en el pin

	    GPIO_Init(GPIOA, &GPIO_InitStructure);	//Se aplica la configuración definida anteriormente
	    										//al puerto A
}

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
