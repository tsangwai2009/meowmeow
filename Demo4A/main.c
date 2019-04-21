#include "stm32f10x.h" 
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
float new_AngleError,  
		acum_AngleError,
		last_AngleError
		= 0;

int flag_PIDFront,flag_PIDBack,flag_PIDBall = 0;
int progress = 0;
volatile float init_CarPos[2] = {0,0};
volatile float CarFrontxy[2] = {0,0};
volatile float CarBackxy[2] = {0,0};					
volatile float Ballxy[2] = {0,0};    		
volatile float MidLimit = 490;
volatile float LeftLimit = 350;					//LowerLimit,UpperLimit
volatile float RightLimit = 600;		
volatile float angleLarge = 0;
volatile float angleSmall = 0;
volatile float distanceLarge = 0;
volatile float distanceSmall = 0;
float Kp_angle = 12;					
float Ki_angle = 6;					//
float Kd_angle = 8;

float Kp_speed = 18;					//29
float Ki_speed = 0;	
float Kd_speed = 10;
				


int sysTick,
		msTick,
		strPos= 0;




unsigned char timer_start = 0;

char ch = 0; 
char strBuffer[128]= "";
//

char send_WifiTesting[] = "AT\r\n";																											//Receive OK
char send_WifiTesting1[] = "AT+CWJAP=\"IntegratedProject\",\"31053106\"\r\n";						//Right
char send_WifiTesting2[] = "AT+CWJAP=\"IntegratedProject1\",\"31053106\"\r\n";					//Left
char send_WifiTesting3[] = "AT+CIFSR\r\n";																									//Get IP Address
char send_WifiTesting4[] = "AT+CIPSTART=\"UDP\",\"0\",0,3105,2\r\n";										//Receive position



/*
AT+CWJAP="IntegratedProject1","31053106"
AT+CIFSR
AT+CIPSTART="UDP","0",0,3105,2
*/
void doNothing(){};
	
void delayMs(uint32_t ms)
{
	int i = 72000;
	int j	= ms *i;
	while(j!=0)//20ms
	{j--;}
}

float dec(char hex)
{
	switch(hex) {
			case '0': return 0;
			case '1': return 1;
			case '2': return 2;
			case '3': return 3;
			case '4': return 4;
			case '5': return 5;
			case '6': return 6;
			case '7': return 7;
			case '8': return 8;
			case '9': return 9;
			case 'a': return 10;
			case 'b': return 11;
			case 'c': return 12;
			case 'd': return 13;
			case 'e': return 14;
			case 'f': return 15;
		}
}

void USARTSend(USART_TypeDef *urt,char *pucBuffer,	unsigned long UlCount)
{
		while(UlCount--)
		{
			USART_SendData(urt,*pucBuffer++);
			while(USART_GetFlagStatus(urt,USART_FLAG_TC) == RESET);
		}
}

void USART2_IRQHandler(){
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
				ch = USART_ReceiveData(USART2);	
				strBuffer[strPos] = ch;
				if (ch==':')
				{		
						GPIO_ResetBits(GPIOB, GPIO_Pin_7);					
						memset(strBuffer, 0, sizeof(strBuffer));
						strPos = 0;
				}
				
				if (strBuffer[strPos-8] == 'C' && strBuffer[strPos-7] == 'B' && strBuffer[strPos-6] == 'E')						//Blue
					{
								CarFrontxy[0] = dec(strBuffer[strPos-5]) *256 +dec(strBuffer[strPos-4]) *16 + dec(strBuffer[strPos-3]);
								CarFrontxy[1] = dec(strBuffer[strPos-2]) *256 +dec(strBuffer[strPos-1]) *16 + dec(strBuffer[strPos]);
								flag_PIDFront = 1;
					}
				if (strBuffer[strPos-8] == 'C' && strBuffer[strPos-7] == 'V' && strBuffer[strPos-6] == 'T')						//Violet
					{
								CarBackxy[0] = dec(strBuffer[strPos-5]) *256 +dec(strBuffer[strPos-4]) *16 + dec(strBuffer[strPos-3]);
								CarBackxy[1] = dec(strBuffer[strPos-2]) *256 +dec(strBuffer[strPos-1]) *16 + dec(strBuffer[strPos]);
								flag_PIDBack =1;
					}
				if (strBuffer[strPos-8] == 'B' && strBuffer[strPos-7] == 'Y' && strBuffer[strPos-6] == 'W')
					{
								Ballxy[0] = dec(strBuffer[strPos-5]) *256 +dec(strBuffer[strPos-4]) *16 + dec(strBuffer[strPos-3]);
								Ballxy[1] = dec(strBuffer[strPos-2]) *256 +dec(strBuffer[strPos-1]) *16 + dec(strBuffer[strPos]);
								flag_PIDBall = 1;
					}

			strPos++;
      USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}


void tune_Motor(int offset, float Kp,float Ki,float Kd){
		float PID, left_Motor, Right_Motor;
			PID = Kp*new_AngleError  + Ki * acum_AngleError + Kd * (new_AngleError - last_AngleError);

			left_Motor = offset-PID ;
			Right_Motor = offset+PID; 

		if (left_Motor>=3000) left_Motor=3000;
		else if (left_Motor<=0) left_Motor=0;
	
		if (Right_Motor>=3000) Right_Motor=3000;
		else if (Right_Motor<=0) Right_Motor=0;
			
		TIM_SetCompare2(TIM3, left_Motor);
		TIM_SetCompare1(TIM3, Right_Motor);
		
		acum_AngleError += new_AngleError;
		if (acum_AngleError>300) acum_AngleError = 300;
		else if (acum_AngleError<-300) acum_AngleError = -300;
		last_AngleError = new_AngleError;
}



void reverse_Motor(){
		if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_0) ==SET && GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_15) ==SET)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_0); //set = forward; reset = backwward(LEFT)	
			GPIO_ResetBits(GPIOC, GPIO_Pin_15); 
		}
		else 
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			GPIO_SetBits(GPIOC, GPIO_Pin_15);
		} //set = forward; reset = backwward (RIGHT)
}

void stop_Motor(){
			TIM_SetCompare2(TIM3, 0);
			TIM_SetCompare1(TIM3, 0);
}

void cal_ErrorFront(volatile float Posxy[]){
				distanceLarge = sqrt(pow((Posxy[1]- CarBackxy[1]),2) + pow((Posxy[0] - CarBackxy[0]),2));
				distanceSmall = sqrt(pow((CarFrontxy[1]- CarBackxy[1]),2) + pow((CarFrontxy[0] - CarBackxy[0]),2));
				angleLarge = 	(CarBackxy[1] - Posxy[1]) / distanceLarge;			
				angleSmall =  (CarBackxy[1]- CarFrontxy[1] ) / distanceSmall;
				float angle_diff= (angleLarge-angleSmall)*90;
				new_AngleError = angle_diff;
}

void cal_ErrorBackward(volatile float Posxy[]){
				distanceLarge = sqrt(pow((Posxy[1]- CarBackxy[1]),2) + pow((Posxy[0] - CarBackxy[0]),2));
				distanceSmall = sqrt(pow((CarFrontxy[1]- CarBackxy[1]),2) + pow((CarFrontxy[0] - CarBackxy[0]),2));
				angleLarge = 	(CarFrontxy[1] -Posxy[1]) / distanceLarge;			
				angleSmall =  (CarFrontxy[1]- CarBackxy[1] ) / distanceSmall;
				float angle_diff= (angleLarge-angleSmall)*90;
				new_AngleError = angle_diff;
}

void progress_Table()
{
			//Car A
			if (progress ==0)																					//CarBackxy stop, wait movement
			{
					init_CarPos[0] = CarFrontxy[0];
					init_CarPos[1] = CarFrontxy[1];
					cal_ErrorFront(Ballxy);
					if (new_AngleError<5 && new_AngleError >-5) progress =1;
					tune_Motor(500,Kp_angle,Ki_angle,Kd_angle);

			}	
			else if (progress ==1)																				
			{		
					cal_ErrorFront(Ballxy);
					if (distanceLarge>100) tune_Motor(2000,Kp_speed,Ki_speed,Kd_speed);
					else 	tune_Motor(1200,Kp_speed,Ki_speed,Kd_speed);
					if (Ballxy[0]>LeftLimit)
					{
							stop_Motor();
							reverse_Motor();
							stop_Motor();
							progress =2;
					}
			}	
			else if (progress == 2)																				
				{
						cal_ErrorBackward(init_CarPos);		
						tune_Motor(2000,Kp_speed,Ki_speed,Kd_speed);		
						if (CarBackxy[0]<init_CarPos[0])
						{
							stop_Motor();
							reverse_Motor();
							stop_Motor();
							progress =-1;
						}
				}
			else if (progress == -1)
				{
					if (Ballxy[0]<LeftLimit)
						progress = 0;
				}
}

void SysTick_Handler(){
	if (timer_start==0)
		doNothing();
	else 
	{
		sysTick++;
		if (sysTick==1)
		{	
			if (flag_PIDFront == 1 && flag_PIDBack ==1 &&flag_PIDBall ==1)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_7);
				progress_Table();
				flag_PIDFront = 0;
				flag_PIDBack = 0;
				flag_PIDBall = 0;
			}
			sysTick=0;
		}
	}
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {		
			if (timer_start == 0){
				GPIO_SetBits(GPIOB, GPIO_Pin_7);
				timer_start = 1;
			}
			else{
				timer_start = 0;
				GPIO_ResetBits(GPIOB, GPIO_Pin_7);
			}
			delayMs(50);
			EXTI_ClearITPendingBit(EXTI_Line8);
		}
}



int main(void) 
{
	RCC_init();
	Button_LED_init();
	TIM3_PWM_init();
	SysTick_init();	
	USART2_init();
	USARTSend(USART2, send_WifiTesting2,sizeof(send_WifiTesting1));
	delayMs(100);
	USARTSend(USART2, send_WifiTesting4,sizeof(send_WifiTesting4));
	delayMs(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);			//Debugging LED
	delayMs(100);
	while(1){	}
}