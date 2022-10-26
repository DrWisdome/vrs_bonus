/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* Function processing DMA Rx data. Counts how many capital and small letters are in sentence.
 * Result is supposed to be stored in global variable of type "letter_count_" that is defined in "main.h"
 *
 * @param1 - received sign
 */
void proccesDmaData(const uint8_t* data, uint16_t len);
void searchForCommand(const uint8_t* data,uint16_t len);


/* Space for your global variables. */
uint8_t tx_data[256];
uint8_t tx_string[256] = "";
uint8_t led_state=0;
uint8_t counter = 0;

	// type your global variables here:


int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* Space for your local variables, callback registration ...*/
  	  //type your code here:
  USART2_RegisterCallback(proccesDmaData);
  *((volatile uint32_t *) (uint32_t)(0x40021000 + 0x00000014U)) |= (uint32_t)(1 << 18);

    *((volatile uint32_t *)((uint32_t)0x48000400)) &= ~(uint32_t)(0x3 << 6);
    *((volatile uint32_t *)((uint32_t)0x48000400)) |= (uint32_t)(1 << 6);

    *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x04U))) &= ~(1 << 3);

    *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x08U))) &= ~(0x3 << 6);

    *((volatile uint32_t *)((uint32_t)(0x48000400 + 0x0CU))) &= ~(0x3 << 6);

  while (1)
  {
	  /* Periodic transmission of information about DMA Rx buffer state.
	   * Transmission frequency - 0.5Hz.
	   * Message format - "Buffer capacity: %d bytes, occupied memory: %d bytes, load [in %]: %f%"
	   * Example message (what I wish to see in terminal) - Buffer capacity: 1000 bytes, occupied memory: 231 bytes, load [in %]: 23.1%
	   */

	  /* Valid text string information transmission.
	   * Transmission frequency - when new valid string is received.
	   * Message format - "Valid string: %s, lower-case: %d, upper-case: %d"
	   * Example message (what I wish to see in terminal) - Valid string: Platn15uborZnakov, lower-case: 13, upper-case: 2
	   */

  	  	  	  //type your code here:

	  USART2_PutBuffer(tx_data, sprintf((char *)tx_data, "LED state: %d\n",led_state));
	  LL_mDelay(2000);

  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}

/*
 * Implementation of function processing data received via USART.
 */
void proccesDmaData(const uint8_t* data,uint16_t len)
{
	/* Process received data */

		// type your algorithm here:
		if(!led_state){
			searchForCommand(data,len);
			if(strstr(tx_string, "ledON")){
				LED_ON;
				led_state=1;
				strcpy(tx_string,"");
			}
		}
		else{
			searchForCommand(data,len);
			if(strstr(tx_string, "ledOFF")){
				LED_OFF;
				led_state=0;
				strcpy(tx_string,"");
			}
		}



		}
void searchForCommand(const uint8_t* data,uint16_t len){
	if(counter>=250){
		strcpy(tx_string,"");
		counter=0;
	}
	for(int i = 0; i< len; i++){


				if(*(data+i) >= 'a' && *(data+i) <= 'z'){
						strncat(tx_string, &(*(data+i)), 1);
						counter++;
					}

				if(*(data+i) >= 'A' && *(data+i) <= 'Z'){
						strncat(tx_string, &(*(data+i)), 1);
						counter++;
					}

				}
}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{ 

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
