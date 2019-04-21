#include "stm32f10x.h"
void RCC_init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void Button_LED_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;	
	//LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//EXTI9_5
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
	
	//EXTI
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line8;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	//EXTI
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	}

void USART2_init(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 

		USART_InitTypeDef USART_InitStructure;
		//USART_ClockInitTypeDef USART_ClockInitStructure; 

		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART2, &USART_InitStructure);
		
		USART_Cmd(USART2, ENABLE);

		NVIC_InitTypeDef NVIC_InitStructure;
		// Enable the USART2 RX Interrupt
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}

void SysTick_init(void){
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	//SysTick
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void TIM3_PWM_init(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		// Configure I/O for Left Wheel
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA, GPIO_Pin_0); //set = forward; reset = backwward(LEFT)	
	
	
		// Configure I/O for Right Wheel
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOC, GPIO_Pin_14); //set when run fast; reset when run slow
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_SetBits(GPIOC, GPIO_Pin_15); //set = forward; reset = backwward (RIGHT)
	
		// Configure I/O for Tim3 Ch1&2 PWM pin
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		
		TIM_TimeBaseInitTypeDef timerInitStructure; 
		timerInitStructure.TIM_Prescaler = 0;  //1/(72Mhz/1440)=0.2ms
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 5000-1;  
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &timerInitStructure);
		
		TIM_Cmd(TIM3, ENABLE);

		//Enable Tim3 PWM
		TIM_OCInitTypeDef outputChannelInit;
		//Enable Tim3 Ch1 PWM
		outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
		outputChannelInit.TIM_Pulse = 1-1; 
		outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
		outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM3, &outputChannelInit);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		//Enable Tim3 Ch2 PWM
		outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
		outputChannelInit.TIM_Pulse = 1-1; 
		outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
		outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC2Init(TIM3, &outputChannelInit);
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

