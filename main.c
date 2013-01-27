#include "main.h"

/* Private variables ---------------------------------------------------------*/
//UART connection  
static __IO uint32_t TimingDelay;
void UARTSend(const unsigned char * pucBuffer, unsigned long ulCount);
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

//LIS302DL accelerometer
LIS302DL_InitTypeDef  LIS302DL_InitStruct;
LIS302DL_FilterConfigTypeDef LIS302DL_FilterStruct;  
__IO int8_t X_Offset, Y_Offset, Z_Offset  = 0x00;
uint8_t Buffer[6];

/* Method definition ---------------------------------------------------------*/



int main(void) {
  
  unsigned char welcome_str[] = "xxyyzz\r\n";
  u8 loop = 1;
  
  initPA15();
  init_USART1(BT_BAUD);
  init_LIS302DL();
    
//  setPA15On();
//  togglePA15();

  while(loop){
    //Read and print the accelerometer values
    LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
    printf("%d, %d ,%d\n",Buffer[0],Buffer[2],Buffer[4]);
    
    //Send data through the bluetooth communication
    UARTSend(welcome_str, sizeof(welcome_str));
    
    //Wait some time befor ending the loop
    Delay(10000000);
  }
  
    /* Disable SPI1 used to drive the MEMS accelerometre */
    SPI_Cmd(LIS302DL_SPI, DISABLE);
  
    /* Disable the UART connection */
    USART_Cmd(USART1, DISABLE);
}

/*- Normal method ------------------------------------------------------------*/

/**
* Method that send a string to the UART.
* @param *pcBuffer buffers to be printed.
*@param ulCount the buffer's length
*/
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

void setPA15On(){
  // GPIO port bit set/reset low register,  Address offset: 0x18      */
  GPIOA->BSRRL = GPIO_Pin_15;
}

void togglePA15(){
  //  GPIO port output data register,        Address offset: 0x14      */
  GPIOA->ODR ^= GPIO_Pin_15;
}

/**
* @brief  MEMS accelerometre management of the timeout situation.
*/
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
    while (1);
}

/*- Initialisation methods ---------------------------------------------------*/

/**
*@brief This method is the one provided by STMicroelectronic with the discoverycard demo. It is used to configure the LIS302DL accelerometer embedded on the discovery card.
*/
void init_LIS302DL(){
/* MEMS configuration */
LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
LIS302DL_Init(&LIS302DL_InitStruct);

/* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate 
= 3/100 = 30ms */
Delay(30);

/* MEMS High Pass Filter configuration */
LIS302DL_FilterStruct.HighPassFilter_Data_Selection = LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
LIS302DL_FilterStruct.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
LIS302DL_FilterStruct.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;
LIS302DL_FilterConfig(&LIS302DL_FilterStruct);
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

/*- Interruption handler -----------------------------------------------------*/

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
    else{ // otherwise reset the character counter
      cnt = 0;
    }
  }
}


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

/*- Timing methods -----------------------------------------------------------*/

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

/**
*@brief Method used to wait a certain amount of time
*@param nCount the time you want to wait
*/
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}