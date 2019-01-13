#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

//PA0 is the Tx Pin
//PA1 is the Rx Pin
//GND of both the microcontrollers should be shorted

//Predefined Variables
volatile int PWM_Initial = 60;
volatile int Target = 60;
volatile int Max_PWM = 127;
volatile int Kp_Small_Val = 1, Kp_Medium_Val = 2, Kp_Large_Val = 3,
				Kd_Small_Val = 3, Kd_Medium_Val = 6, Kd_Large_Val = 9;

//Recieved Number of ticks
volatile int Ticks = 0;

//Compute_Error()
volatile int Prev_Error = 0;
volatile int Error=0, D_Error=0, Set_Value=10; // Bound on Membership Values

//Error_Fuzzification()
volatile int PE=0, NE=0, ZE=0,
			PDE=0, NDE=0, ZDE=0;

//Create_Fuzzy_Matrix()
volatile int FM[3][3] = {{0}};

//Defuzzification()
volatile int Kp_Small, Kp_Medium, Kp_Large,
				Kd_Small, Kd_Medium, Kd_Large;
volatile int Kp_Num=0, Kp_Den=0, Kd_Num=0, Kd_Den=0;

volatile int Expected_PWM=0, Fault=0;

volatile int PWM_Fuzzy = 0;

volatile int min(volatile int x,volatile int y){
	return x<y ? x:y;
}
volatile int max(volatile int x,volatile int y){
	return x>y ? x:y;
}

void Initialise_LED()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef GPIO_Init_Struct;
	GPIO_Init_Struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_Init_Struct.GPIO_OType=GPIO_OType_PP;
	GPIO_Init_Struct.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init_Struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_Init_Struct);
	GPIO_SetBits(GPIOD,GPIO_Pin_12);
}

void Initialise_UART()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,   GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,   GPIO_AF_UART4);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate=9600;
	USART_InitStruct.USART_WordLength=USART_WordLength_8b ; //Change this to increase the size of data send
	USART_InitStruct.USART_StopBits=USART_StopBits_1;
	USART_InitStruct.USART_Parity=USART_Parity_No;
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx ;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(UART4,&USART_InitStruct);

	USART_Cmd(UART4,ENABLE);
	USART_ITConfig(UART4,USART_IT_RXNE ,ENABLE);
}

void Initialise_NVIC()
{
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=UART4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStruct);
}

void Compute_Error()
{
	Prev_Error = Error;
	Error = Target - Ticks;
	D_Error = Error - Prev_Error;
}

void Error_Fuzzification()
{
	PE = 0, NE = 0, ZE = 0;
	if(Error>0)
	{
		PE = min(Error, Set_Value);
		ZE = max(Set_Value - Error, 0);
		NE = 0;
	}
	else if(Error<0)
	{
		PE = 0;
		ZE = max(Set_Value + Error, 0);
		NE = min(-Error, Set_Value);
	}

	PDE = 0, NDE = 0, ZDE = 0;
	if(D_Error>0)
	{
		PDE = min(D_Error, Set_Value);
		ZDE = max(Set_Value - D_Error, 0);
		NDE = 0;
	}
	else if(D_Error<0)
	{
		PDE = 0;
		ZDE = max(Set_Value + D_Error, 0);
		NDE = min(-D_Error, Set_Value);
	}
}

void Create_Fuzzy_Matrix()
{
	FM[0][0] = min(NE, NDE), FM[0][1] = min(NE, ZDE), FM[0][2] = min(NE, PDE);
	FM[1][0] = min(ZE, NDE), FM[1][1] = min(ZE, ZDE), FM[1][2] = min(ZE, PDE);
	FM[2][0] = min(PE, NDE), FM[2][1] = min(PE, ZDE), FM[2][2] = min(PE, PDE);
}

void Defuzzification()
{
	Kp_Small = FM[1][1];
	Kp_Medium = max(max(max(FM[1][0], FM[0][1]), max(FM[2][1], FM[1][2])), max(FM[0][0], FM[2][2]));
	Kp_Large = max(FM[0][2], FM[2][0]);

	Kd_Small = max(max(FM[0][2], FM[2][0]), FM[1][1]);
	Kd_Medium = max(max(FM[0][0], FM[0][1]), max(FM[2][1], FM[2][2]));
	Kd_Large = max(FM[1][0], FM[1][2]);

	Kp_Num = Kp_Small*Kp_Small_Val + Kp_Medium*Kp_Medium_Val + Kp_Large*Kp_Large_Val;
	Kp_Den = Kp_Small + Kp_Medium + Kp_Large;

	Kd_Num = Kd_Small*Kd_Small_Val + Kd_Medium*Kd_Medium_Val + Kd_Large*Kd_Large_Val;
	Kd_Den = Kd_Small + Kd_Medium + Kd_Large;
}

volatile int Fuzzy_PID()
{
	Error_Fuzzification();
	Create_Fuzzy_Matrix();
	Defuzzification();
	if(Kp_Den != 0 && Kd_Den != 0)
	{
		Fault = Error*Kp_Num/Kp_Den;
		Fault += D_Error*Kd_Num/Kd_Den;
	}
	Expected_PWM = PWM_Fuzzy;
	Expected_PWM += Fault;
	Expected_PWM = min(max(0, Expected_PWM), Max_PWM);
	return Expected_PWM;
}

void UART4_IRQHandler()
{
	//Recieving Data as ticks
	GPIO_SetBits(GPIOD,GPIO_Pin_13); //Pin_13_LED set during Recieving
	Ticks=USART_ReceiveData(UART4);
	Compute_Error(); // Compute the Error
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);

	//Call the Fuzzy Code Function Here and return the value to PWM_Fuzzy
	PWM_Fuzzy = Fuzzy_PID();

	//Transmitting Data as Fuzzy_PWM
	GPIO_SetBits(GPIOD,GPIO_Pin_12); //Pin_12_LED set during Transmitting
	while(USART_GetFlagStatus( UART4,USART_FLAG_TXE)!=SET){}
		USART_SendData(UART4, PWM_Fuzzy);
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
}

int main(void)
{
	Initialise_LED();
	Initialise_UART();
	Initialise_NVIC();

	//Send a PWM for the First Time
	while(USART_GetFlagStatus( UART4,USART_FLAG_TXE)!=SET){}
		USART_SendData(UART4, PWM_Initial);

	PWM_Fuzzy = PWM_Initial;
	//Subsequent Transmission will be initialise when any data is recieved in the UART4_IRQHandler();

    while(1)
    {

    }
}
