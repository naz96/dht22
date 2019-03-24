#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#define portB (RCC->APB2ENR |= RCC_APB2ENR_IOPBEN)

void delay(long cycles);
void delay(long cycles){
while(cycles>0)
cycles--;
}

 
void init_dht22 (void){
int i = 0;
	GPIOB->CRL = 0x00000001;
	GPIOB->ODR = 0x00;
	
	i = 100; // чтобы равнялось 1 мс
	while(i--);
	
	GPIOB->CRL = 0x00000004;
	while(GPIOB->IDR & 0x01);
}

int RX_data_dht22(void) {
	int j = 0;
	int data;
	for(int i = 0; i < 41; i++){
		while((GPIOB->IDR ^ 0x01) & 0x01);
		j = 0;
		while(GPIOB->IDR & 0x01) {
			j++;
		}
		if ((i > 0) && (i < 33)) {
			if (j > 300)	{// время около 30 мкс
				data = data >> 1;
				data = 0x80000000 | data;
			}
			else {
				data = data >> 1;
				data = 0x7FFFFFFF & data;
			}
		}
	}
	return data;
}


void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
 
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);
 
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);
 
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
 
        /* Enable PLL */
        RCC_PLLCmd( ENABLE);
 
        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
 
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
 
        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */
 
        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

int main(){

RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
     GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure the USART1 */
    USART_InitTypeDef USART_InitStructure;
 
    /* USART1 configuration ------------------------------------------------------*/
    /* USART1 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
            the SCLK pin
     */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    USART_Init(USART1, &USART_InitStructure);
 
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
 
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
        USART1 receive data register is not empty */
 //   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	
	//**********************************************


		char per;
init_dht22();
int r;
while(1){
	//per = (uint8_t)data;
	r = RX_data_dht22();
	per = (uint8_t)(r & 0xFF);
	USART_SendData(USART1, per);
	per = (uint8_t)((r >> 8) & 0xFF);
	USART_SendData(USART1, per);
	per = (uint8_t)((r >> 16) & 0xFF);
	USART_SendData(USART1, per);
	per = (uint8_t)((r >> 24) & 0xFF);
	USART_SendData(USART1, per);
	delay(2);
	
}

}