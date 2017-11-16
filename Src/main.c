/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

#include "programmer.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char buf[128];
uint32_t state;
uint8_t cmd;
uint32_t param_len;
static uint8_t write_buf_flg;
static uint8_t read_buf_flg;
static uint8_t read_flg;
static uint8_t write_flg;
static uint8_t erase_flg;

#define CMD_WRITE_BUF 	0x31
#define CMD_READ_BUF 	0x32
#define CMD_WRITE 		0x33
#define CMD_READ 		0x34
#define CMD_ERASE 		0x35

#define STATE_IDLE  0
#define STATE_BITCNT 1
#define STATE_OFFSET 2
#define STATE_DATLEN 3
#define STATE_DATA 4

#define PAGE_SIZE 512

/* USER CODE END 0 */

uint8_t addr_buf[2];

uint8_t rnw;
uint8_t offset;
uint8_t bit_cnt;
uint8_t data_len;
uint8_t data_buf[128];

void delay_us(uint32_t Number)
{
    volatile uint32_t i=0;
    while(Number--){
        i = 40; while(i--);
    }
}

void bitbang_interface_write(int clk, int nothing, int tdi)
{
    if (tdi) {
        GPIOA->BSRR = SWDIO;
    } else {
        GPIOA->BRR = SWDIO;
    }
    if (clk) {
        GPIOA->BSRR = SWCLK;        
    } else {
        GPIOA->BRR = SWCLK;                
    }
    delay_us(3);
}
int bitbang_interface_swdio_read()
{
    return !!(GPIOA->IDR & SWDIO);
}

static void bitbang_exchange(uint8_t rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
    int tdi;

    for (unsigned int i = offset; i < bit_cnt + offset; i++) {
        int bytec = i/8;
        int bcval = 1 << (i % 8);
        tdi = !rnw && (buf[bytec] & bcval);

        bitbang_interface_write(0, 0, tdi);

        if (rnw && buf) {
            if (bitbang_interface_swdio_read())
                buf[bytec] |= bcval;
            else
                buf[bytec] &= ~bcval;
        }

        bitbang_interface_write(1, 0, tdi);
    }
}


void process_usb(uint8_t* buf, uint32_t len) 
{
    uint32_t i;
    static uint32_t pi;
    uint8_t ch;
    i = 0;
    while(i < len) {
        switch(state) {
        case STATE_IDLE:
            switch(buf[i++]) {
            case 0xE0:
                SET_DIO_INPUT();
                break;
            case 0xE1:
                SET_DIO_OUTPUT();
                break;
            case 0xF0:
                rnw = 0;
                state = STATE_BITCNT;
                pi = 0;
                break;
            case 0xF1:
                rnw = 1;
                state = STATE_BITCNT;
                pi = 0;                
                break;
            default:
                break;
            }
            break;
        case STATE_BITCNT:
            ch = buf[i++];            
            if (pi & 0x01) {
                bit_cnt |= (ch > '9' ? ch - 'A' + 10 : ch - '0');
            } else {
                bit_cnt = ((ch > '9' ? ch - 'A' + 10 : ch - '0') << 4);
            };
            pi++;            
            if (pi == 2) {
                pi = 0;
                state = STATE_OFFSET;                
            }
            break;
        case STATE_OFFSET:
            ch = buf[i++];            
            if (pi & 0x01) {
                offset |= (ch > '9' ? ch - 'A' + 10 : ch - '0');
            } else {
                offset = ((ch > '9' ? ch - 'A' + 10 : ch - '0') << 4);
            };
            pi++;            
            if (pi == 2) {
                pi = 0;
                state = STATE_DATLEN;
            }
            break;
        case STATE_DATLEN:
            ch = buf[i++];            
            if (pi & 0x01) {
                data_len |= (ch > '9' ? ch - 'A' + 10 : ch - '0');
            } else {
                data_len = ((ch > '9' ? ch - 'A' + 10 : ch - '0') << 4);
            };
            pi++;            
            if (pi == 2) {
                pi = 0;
                if (data_len == 0) {
                    read_flg = 1;
                    state = STATE_IDLE;
                } else if (rnw){
                    read_flg = 1;                    
                    state = STATE_IDLE;
                } else {
                    state = STATE_DATA;                    
                }
            }
            break;
        case STATE_DATA:
            ch = buf[i++];            
            if (pi & 0x01) {
                data_buf[pi / 2] |= (ch > '9' ? ch - 'A' + 10 : ch - '0');
            } else {
                data_buf[pi / 2] = ((ch > '9' ? ch - 'A' + 10 : ch - '0') << 4);
            }
            pi++;
            if (pi/2 == data_len) {
                write_flg = 1;
                state = STATE_IDLE;                
            }
            break;
        }
    }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    uint32_t i;
    uint32_t addr;
    unsigned char chip_id = 0;
    unsigned char debug_config = 0;
    state = 0;
    write_buf_flg = 0;    
    read_buf_flg = 0;
    write_flg = 0;
    read_flg = 0;
    erase_flg = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
    programmer_init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
        if (read_flg) {
            read_flg = 0;
            if (data_len == 0) {
                bitbang_exchange(rnw, NULL, offset, bit_cnt);
            } else {
                bitbang_exchange(rnw, data_buf, offset, bit_cnt);                
            }
            for (i = 0; i < data_len; i++) {
                sprintf(&(buf[0]), "%02X", data_buf[i]);
                delay_us(2000);                                    
                while(CDC_Transmit_FS((uint8_t *)buf, 2) != USBD_OK);                                    
            }
        }
        if (write_flg) {
            write_flg = 0;
            if (data_len == 0) {
                bitbang_exchange(rnw, NULL, offset, bit_cnt);
            } else {
                bitbang_exchange(rnw, data_buf, offset, bit_cnt);                
            }
            sprintf(&(buf[0]), "%02X", bit_cnt); 
            delay_us(2000);
            while(CDC_Transmit_FS((uint8_t *)buf, 2) != USBD_OK);
            /*   
            /* for (i = 0; i < 3; i++) { */
            /*     switch(i) { */
            /*     case 0: */
            /*         sprintf(&(buf[0]), "%02X", bit_cnt); */
            /*         delay_us(2000);                     */
            /*         while(CDC_Transmit_FS((uint8_t *)buf, 2) != USBD_OK); */
            /*         break; */
            /*     case 1: */
            /*         sprintf(&(buf[0]), "%02X", offset); */
            /*         delay_us(2000);                                         */
            /*         while(CDC_Transmit_FS((uint8_t *)buf, 2) != USBD_OK); */
            /*         break; */
            /*     case 2: */
            /*         sprintf(&(buf[0]), "%02X", data_len); */
            /*         delay_us(2000);                                         */
            /*         while(CDC_Transmit_FS((uint8_t *)buf, 2) != USBD_OK); */
            /*         break; */
            /*     } */
            /* } */
            /* for (i = 0; i < data_len; i++) { */
            /*     sprintf(&(buf[0]), "%02X", data_buf[i]); */
            /*     delay_us(2000); */
            /*     while(CDC_Transmit_FS((uint8_t *)buf, 2) != USBD_OK); */
            /* } */
        }
        i++;
        if (i & 1) 
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);   
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA7 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
