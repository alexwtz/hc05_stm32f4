#include <stm32f4xx.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src

static __IO uint32_t TimingDelay;
void UARTSend(const unsigned char * pucBuffer, unsigned long ulCount);
#define MAX_STRLEN 20 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
const char test[10] =  {1,2,3,4,5,6,7,8,9,0};

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */
void init_USART1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			// this activates the pullup resistors on the IO pins
        
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}
void  initPA15();
void  setPA15On();
void togglePA15();

int main(void) {

  initPA15();
  setPA15On();
//  togglePA15();
  int baud[] = {28800,38400};
  init_USART1(28800); // initialize USART1 @ 9600 baud
  //init_USART1(38400);
  
//  USART_puts(USART1, "AT\r\n");
   const unsigned char welcome_str[] = "Joyeux Noel\r\n";
   UARTSend(welcome_str, sizeof(welcome_str));
  while(1);
//  while (1){
//    USART_puts(USART1, "AT"); // just send a message to indicate that it works
//    Delay(10000000);
//    /*
//     * You can do whatever you want in here
//     */
//  }
}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		if( (t != 'n') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			USART_puts(USART1, received_string);
		}
	}
}






///**
//  ******************************************************************************
//  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
//  * @author  MCD Application Team
//  * @version V1.0.1
//  * @date    13-April-2012
//  * @brief   Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
//  *
//  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
//  * You may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at:
//  *
//  *        http://www.st.com/software_license_agreement_liberty_v2
//  *
//  * Unless required by applicable law or agreed to in writing, software 
//  * distributed under the License is distributed on an "AS IS" BASIS, 
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *
//  ******************************************************************************
//  */
//
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///** @addtogroup Template_Project
//  * @{
//  */
//
///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
//#define MESSAGE1   "     STM32F4xx      " 
//#define MESSAGE2   " Device running on  " 
//#define MESSAGE3   " STM3240_41_G-EVAL  " 
//
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//static __IO uint32_t TimingDelay;
//static int i;
///* Private function prototypes -----------------------------------------------*/
///* Private functions ---------------------------------------------------------*/
//void USART1_IRQHandler(void);
//void NVIC_Configuration(void);
//void UARTSend(const unsigned char * pucBuffer, unsigned long ulCount);
//
void initPA15(){
   GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void setPA15On(){
// GPIO port bit set/reset low register,  Address offset: 0x18      */
  GPIOA->BSRRL = GPIO_Pin_15;
}

void togglePA15(){
//  GPIO port output data register,        Address offset: 0x14      */
   GPIOA->ODR ^= GPIO_Pin_15;
}
//
//void RCC_Configuration(void)
//{
//  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
//  initialize the PLL and update the SystemFrequency variable. */
//  SystemInit();
//
//  /* --------------------------- System Clocks Configuration -----------------*/
//  /* USART1 clock enable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
// 
//  /* GPIOA clock enable */
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//}
//
//void initUART1(){
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  
//  /* Configure USART1 Rx as input floating */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  /* Configure USART1 Tx as alternate function push-pull */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//}
//
//void configUART1(){
//  USART_InitTypeDef USART_InitStructure;
//   
//  /* USARTx configuration ------------------------------------------------------*/
//  /* USARTx configured as follow:
//        - BaudRate = 9600 baud
//        - Word Length = 8 Bits
//        - One Stop Bit
//        - No parity
//        - Hardware flow control disabled (RTS and CTS signals)
//        - Receive and transmit enabled
//  */
//  USART_InitStructure.USART_BaudRate = 9600;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//   
//  USART_Init(USART1, &USART_InitStructure);
//   
//  USART_Cmd(USART1, ENABLE); 
//}
//
///**
//  * @brief  Main program.
//  * @param  None
//  * @retval None
//  */
//int main(void)
//{
//  RCC_ClocksTypeDef RCC_Clocks;
//
//  /*!< At this stage the microcontroller clock setting is already configured, 
//       this is done through SystemInit() function which is called from startup
//       file (startup_stm32f4xx.s) before to branch to application main.
//       To reconfigure the default setting of SystemInit() function, refer to
//       system_stm32f4xx.c file
//     */  
//
//  /* SysTick end of count event each 10ms */
//  RCC_GetClocksFreq(&RCC_Clocks);
//  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
//
//  /* Initialize LEDs and LCD available on STM324xG-EVAL board *****************/
////  STM_EVAL_LEDInit(LED3);
////  STM_EVAL_LEDInit(LED4);
////  STM_EVAL_LEDInit(LED5);
////  STM_EVAL_LEDInit(LED6);
////  
//  initPA15();
//
//  initUART1();
//  configUART1();
//  
//  setPA15On();
//  //togglePA15();
//  
//  
// 
//  /* Turn on LEDs available on STM324xG-EVAL **********************************/
////  STM_EVAL_LEDOn(LED3);
////  STM_EVAL_LEDOn(LED4);
////  STM_EVAL_LEDOn(LED5);
////  STM_EVAL_LEDOn(LED6);
//
//
//  /* Add your application code here
//     */
//
//  //USART Management
//   const unsigned char welcome_str[] = "AT+ROLE=0\r\n";
//   UARTSend(welcome_str, sizeof(welcome_str));
//   
//   while(1){
//    UARTSend(welcome_str, sizeof(welcome_str)); 
//    Delay(5);
//   };
////    
////  /* Infinite loop */
////  while (1)
////  {
////    togglePA15();
////    /* Toggle LD4 */
////    STM_EVAL_LEDToggle(LED4);
////
////    /* Insert 50 ms delay */
////    Delay(5);
////
////    /* Toggle LD2 */
////    STM_EVAL_LEDToggle(LED6);
////
////    /* Insert 50 ms delay */
////    Delay(5);
////    UARTSend(welcome_str, sizeof(welcome_str));
////  }
//}
//
///**
//  * @brief  Inserts a delay time.
//  * @param  nTime: specifies the delay time length, in 10 ms.
//  * @retval None
//  */
//void Delay(__IO uint32_t nTime)
//{
//  TimingDelay = nTime;
//
//  while(TimingDelay != 0);
//}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
//
//#ifdef  USE_FULL_ASSERT
//
///**
//  * @brief  Reports the name of the source file and the source line number
//  *   where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t* file, uint32_t line)
//{ 
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
//#endif
//
///**
//  * @}
//  */
//
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
//
/////**
////  ******************************************************************************
////  * @file    USART/main.c 
////  * @author  Andrew Markham
////  * @version V1.0.0
////  * @date    27-April-2012
////  * @brief   Main program body
////  ******************************************************************************
////  */ 
////
/////* Includes ------------------------------------------------------------------*/
////#include "stm32f4_discovery.h"
////#include <stdio.h>
////
/////* Private typedef -----------------------------------------------------------*/
////GPIO_InitTypeDef  GPIO_InitStructure;
////USART_InitTypeDef USART_InitStructure;
////TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
////TIM_OCInitTypeDef  TIM_OCInitStructure;
/////* Private define ------------------------------------------------------------*/
/////* Private macro -------------------------------------------------------------*/
/////* Private variables ---------------------------------------------------------*/
////static __IO uint32_t TimingDelay;
/////* Private function prototypes -----------------------------------------------*/
////void TIM_Config(void);
////void PWM_Config(int period);
////void PWM_SetDC(uint16_t channel,uint16_t dutycycle);
//void Delay(__IO uint32_t nCount);
////void LED_Config(void);
////
////void NVIC_Configuration(void);
////void GPIO_Configuration(void);
////void USART_Configuration(void);
////void UARTSend(const unsigned char * pucBuffer, unsigned long ulCount);
////
////int i;
/////* Private functions ---------------------------------------------------------*/
////
/////**
////  * @brief  Decrements the TimingDelay variable.
////  * @param  None
////  * @retval None
////  */
////void TimingDelay_Decrement(void)
////{
////  if (TimingDelay != 0x00)
////  { 
////    TimingDelay--;
////  }
////}
////
/////******************************************************************************/
/////*            STM32F10x Peripherals Interrupt Handlers                        */
/////******************************************************************************/
//// 
///**
//  * @brief  This function handles USARTx global interrupt request
//  * @param  None
//  * @retval None
//  */
//void USART1_IRQHandler(void)
//{
//    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
//    {
//        i = USART_ReceiveData(USART1);
//        if(i == '1'){
//            //STM_EVAL_LEDOn(LED4);        // Set '1' on PA8
//            UARTSend("LED ON\r\n",sizeof("LED ON\r\n"));    // Send message to UART1
//        }
//        else if(i == '0'){
//            //STM_EVAL_LEDOff(LED4);     // Set '0' on PA8
//            UARTSend("LED OFF\r\n",sizeof("LED OFF\r\n"));
//        }
//    }
//}
//
///**
//  * @brief  Configures the nested vectored interrupt controller.
//  * @param  None
//  * @retval None
//  */
//void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
// 
//  /* Enable the USARTx Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//}
////
/////*******************************************************************************
////* Function Name  : GPIO_Configuration
////* Description    : Configures the different GPIO ports
////*******************************************************************************/
////void GPIO_Configuration(void)
////{
////  GPIO_InitTypeDef GPIO_InitStructure;
//// 
//////  /* Configure (PA.8) as output */
//////  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
//////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//////  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//////  GPIO_Init(GPIOA, &GPIO_InitStructure); // Save
//// 
////  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////  GPIO_Init(GPIOA, &GPIO_InitStructure);
//// 
////  /* Configure USART1 Rx (PA.10) as input floating */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////  GPIO_Init(GPIOA, &GPIO_InitStructure);
////    
////  /* Configure PIN34 (PA.15) as output */
////  /* Configure the GPIO_LED pin */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
////  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////  GPIO_Init(GPIOA, &GPIO_InitStructure);
////      
////  /* Configure PIN32 (PA.14) as input floating */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
////  GPIO_Init(GPIOA, &GPIO_InitStructure);
////  
////}
////
/////*******************************************************************************
////* Function Name  : USART_Configuration
////* Description    : Configures the USART1
////*******************************************************************************/
////void USART_Configuration(void)
////{
////  USART_InitTypeDef USART_InitStructure;
//// 
/////* USART1 configuration ------------------------------------------------------*/
////  USART_InitStructure.USART_BaudRate = 9600;        // Baud Rate
////  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
////  USART_InitStructure.USART_StopBits = USART_StopBits_1;
////  USART_InitStructure.USART_Parity = USART_Parity_No;
////  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
////  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//// 
////  USART_Init(USART1, &USART_InitStructure);
//// 
////  /* Enable USART1 */
////  USART_Cmd(USART1, ENABLE);
////}
//// 
/*******************************************************************************
* Function Name  : UARTSend
* Description    : Send a string to the UART.
* Input          : - pucBuffer: buffers to be printed.
*                : - ulCount  : buffer's length
*******************************************************************************/
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART1, (uint8_t) *pucBuffer++);
        /* Loop until the end of transmission */
        
    }
}
////
/////**
////  * @brief  Main program
////  * @param  None
////  * @retval None
////  */
////int main(void)
////{
////  uint16_t pulse_width = 0;
////  /* TIM Configuration */
////  TIM_Config();
////  /* LED Configuration */
////  //LED_Config();
////  /* PWM Configuration */
//////  PWM_Config(800);
//////
//////  PWM_SetDC(1,100);
//////  PWM_SetDC(2,100);
//////  PWM_SetDC(3,100);
//////  PWM_SetDC(4,100);
////  
////  STM_EVAL_LEDInit(LED4);
////  STM_EVAL_LEDInit(LED3);
////  STM_EVAL_LEDOn(LED4);  
////  
////  //USART Management
////   const unsigned char welcome_str[] = " Welcome to Bluetooth!\r\n";
//// 
////    /* Enable USART1 and GPIOA clock */
////    RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_AHB1Periph_GPIOA, ENABLE);
////    
////    /* NVIC Configuration */
////    NVIC_Configuration();
//// 
////    /* Configure the GPIOs */
////    GPIO_Configuration();
////    GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_SET);
////    while(1){
////      GPIO_ToggleBits( GPIOA, GPIO_Pin_15);
////    }
////    GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_RESET);
////    
////    /* Configure the USART1 */
////    USART_Configuration();
//// 
////    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
////         USART1 receive data register is not empty */
////    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//// 
////    
////    //Configure HC-05
////    
//////    const unsigned char test[] = "AT+NAME=ALEX\r\n";
//////    UARTSend(test,sizeof(test));
////    
////    
////    
////    
////    
////    
//////    /* print welcome information */
//////    UARTSend(welcome_str, sizeof(welcome_str));
////  
////    
////
////  
////  while (1)
////  {
//////    /* PD12 to be toggled */
////    GPIO_ToggleBits( GPIOD, GPIO_Pin_13);
//////GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
//////    GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
//////    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
//////    PWM_SetDC(1,pulse_width++);
//////    if (pulse_width > 800)
//////    {
//////      pulse_width = 0;
//////    }
////    Delay(10000000);
////  }
////}
////
/////**
////  * @brief  Configure the TIM3 Ouput Channels.
////  * @param  None
////  * @retval None
////  */
////void TIM_Config(void)
////{
////  GPIO_InitTypeDef GPIO_InitStructure;
////
////  /* TIM3 clock enable */
////  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
////
////  /* GPIOC and GPIOB clock enable */
////  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
////  
////  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
////  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
////  GPIO_Init(GPIOC, &GPIO_InitStructure); 
////  
////  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
////  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
////  GPIO_Init(GPIOB, &GPIO_InitStructure); 
////
////  /* Connect TIM3 pins to AF2 */  
////  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
////  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
////  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
////  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
////}
////
////void PWM_Config(int period)
////{
////  uint16_t PrescalerValue = 0;
////  /* -----------------------------------------------------------------------
////    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.
////    
////    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
////    since APB1 prescaler is different from 1.   
////      TIM3CLK = 2 * PCLK1  
////      PCLK1 = HCLK / 4 
////      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
////          
////    To get TIM3 counter clock at 28 MHz, the prescaler is computed as follows:
////       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
////       Prescaler = ((SystemCoreClock /2) /28 MHz) - 1
////                                              
////    To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
////       ARR = (TIM3 counter clock / TIM3 output clock) - 1
////           = 665
////
////    Note: 
////     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
////     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
////     function to update SystemCoreClock variable value. Otherwise, any configuration
////     based on this variable will be incorrect.    
////  ----------------------------------------------------------------------- */  
////
////  /* Compute the prescaler value */
////  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;
////
////  /* Time base configuration */
////  TIM_TimeBaseStructure.TIM_Period = period;
////  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
////  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
////  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
////
////  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
////
////  /* PWM1 Mode configuration: Channel1 */
////  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
////  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////  TIM_OCInitStructure.TIM_Pulse = 0;
////  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
////
////  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
////
////  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
////
////  /* PWM1 Mode configuration: Channel2 */
////  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////  TIM_OCInitStructure.TIM_Pulse = 0;
////
////  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
////
////  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
////
////  /* PWM1 Mode configuration: Channel3 */
////  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////  TIM_OCInitStructure.TIM_Pulse = 0;
////
////  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
////
////  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
////
////  /* PWM1 Mode configuration: Channel4 */
////  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////  TIM_OCInitStructure.TIM_Pulse = 0;
////
////  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
////
////  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
////
////  TIM_ARRPreloadConfig(TIM3, ENABLE);
////
////  /* TIM3 enable counter */
////  TIM_Cmd(TIM3, ENABLE);
////}
////
////void PWM_SetDC(uint16_t channel,uint16_t dutycycle)
////{
////  if (channel == 1)
////  {
////    TIM3->CCR1 = dutycycle;
////  }
////  else if (channel == 2)
////  {
////    TIM3->CCR2 = dutycycle;
////  }
////  else if (channel == 3)
////  {
////    TIM3->CCR3 = dutycycle;
////  }
////  else 
////  {
////    TIM3->CCR4 = dutycycle;
////  }
////}
////
////void LED_Config(void)
////{
////  /* GPIOD Periph clock enable */
////  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
////
////  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
////  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
////  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////  GPIO_Init(GPIOD, &GPIO_InitStructure);
////
////}
////
////
////
////
////
/////**
////  * @brief  Delay Function.
////  * @param  nCount:specifies the Delay time length.
////  * @retval None
////  */
////void Delay(__IO uint32_t nCount)
////{
////  while(nCount--)
////  {
////  }
////}
////
////#ifdef  USE_FULL_ASSERT
////
/////**
////  * @brief  Reports the name of the source file and the source line number
////  *         where the assert_param error has occurred.
////  * @param  file: pointer to the source file name
////  * @param  line: assert_param error line source number
////  * @retval None
////  */
////void assert_failed(uint8_t* file, uint32_t line)
////{
////  /* User can add his own implementation to report the file name and line number,
////     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
////
////  while (1)
////  {}
////}
////#endif
////
/////**
////  * @}
////  */ 
////
/////**
////  * @}
////  */ 
////
/////******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/