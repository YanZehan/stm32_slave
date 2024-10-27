/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

void gpio_control(uint8_t channel, uint8_t value);
void pwm_control(uint8_t channel, uint16_t value);
void data_control();

typedef struct {  
    GPIO_TypeDef *port;  
    uint16_t pin;  
} GPIO_Pin_t;  


const GPIO_Pin_t gpio_pins[] = {  
    {GPIOA, GPIO_PIN_3},  
    {GPIOA, GPIO_PIN_4},  
    {GPIOA, GPIO_PIN_5},  
    {GPIOA, GPIO_PIN_6},  
    {GPIOA, GPIO_PIN_7}  
};  

#define MAX_GPIO_CHANNELS (sizeof(gpio_pins) / sizeof(gpio_pins[0])) 
	


static const struct {
	TIM_HandleTypeDef *htim;
	uint32_t tim_channel;
} pwm_channels[] = {
	{&htim1, TIM_CHANNEL_1}, // Channel 1
	{&htim1, TIM_CHANNEL_2}, // Channel 2
	{&htim1, TIM_CHANNEL_3}, // Channel 3
	{&htim1, TIM_CHANNEL_4}, // Channel 4
	//{&htim2, TIM_CHANNEL_1}, // Channel 5 
};

// 计算通道数量
const uint8_t max_channels = sizeof(pwm_channels) / sizeof(pwm_channels[0]);




uint8_t I2C_recvBuf[3] = {0};
uint8_t temp_buffer[1000]={0};
uint16_t m=0;
int flag = 0;
int a=0;



#define FLASH_LAST_PAGE_ADDRESS  (0x08000000 + (FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1)))
#define FLASH_PAGE_COUNT 64U






/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f){
 uint8_t temp[1] = {ch};
 HAL_UART_Transmit(&huart1, temp, 1, 0xffff);
return ch;
}
#define user_main_printf(format, ...)  printf( format "\r\n", ##__VA_ARGS__)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Erase_Last_Page(void) {
    FLASH_EraseInitTypeDef eraseInitStruct;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_LAST_PAGE_ADDRESS;
    eraseInitStruct.NbPages = 1;
    uint32_t pageError = 0;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
        // 处理擦除错误
    }
}

void Write_Data_To_Flash(uint8_t *data, uint16_t length) {
    uint32_t flashAddress = FLASH_LAST_PAGE_ADDRESS;

    // 擦除最后一页
    Erase_Last_Page();

    // 写入数据（以半字为单位）
    HAL_FLASH_Unlock(); // 解锁Flash
    for (uint16_t i = 0; i < length; i += 2) {
        uint16_t halfword = (data[i + 1] << 8) | data[i]; // 将两个字节合并为一个16位半字
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress + i, halfword) != HAL_OK) {
            // 处理写入错误
        }
    }
    HAL_FLASH_Lock(); // 锁定Flash，防止意外写入
}

// 从Flash读取数据
void Read_Data_From_Flash(uint8_t *data, uint16_t length) {
    uint32_t flashAddress = FLASH_LAST_PAGE_ADDRESS;

    for (uint16_t i = 0; i < length; i++) {
        data[i] = *(uint8_t *)(flashAddress + i);
    }
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	 
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   
   
   if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
	  Error_Handler();
  }
	 

 
 
 
 
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
	printf("while begin:  \r\n");
    printf("data handled\r\n");
	  
	switch(I2C_recvBuf[0])
	{
		case 1:
		{
			
		__disable_irq(); // 禁用全局中断
		data_control();
		printf("data handled\r\n");
		__enable_irq();  // 重新启用全局中断
		HAL_I2C_EnableListen_IT(&hi2c1);
		break;
			
			
		}
		case 2:
		{
			
			
		
		__disable_irq();
		temp_buffer[m]=I2C_recvBuf[1];
		printf("temp_buffer[%d]:%d \r\n",m,temp_buffer[m]);
		m++;
		temp_buffer[m]=I2C_recvBuf[2];
		printf("temp_buffer[%d]:%d \r\n",m,temp_buffer[m]);
		m++;
		__enable_irq();
			
			
			
			
			
		break;
		}
		case 3:
		{
			
		 HAL_FLASH_Unlock();
		 // 将后两个字节写入Flash
        Write_Data_To_Flash(&I2C_recvBuf[1], 2);

        // 读取数据测试
        uint8_t readData[2];
        Read_Data_From_Flash(readData, 2);
		printf("Hex format: readData[0] = 0x%02X, readData[1] = 0x%02X\n", readData[0], readData[1]);
        // 检查读取的数据是否正确
		
		
		
		}
	 }
	  
	

	  
	  
	HAL_SuspendTick(); 
	printf("stop systick and sleep\r\n\n");
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
	printf("stm32 wake up\r\n");
	HAL_ResumeTick();
//	 } 
	  
	  
    

	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
  }
  
  
  
  
  
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */






void data_control()
{
    uint8_t mode = (I2C_recvBuf[1] >> 4);  
    uint8_t channel = (I2C_recvBuf[1] & 0x0F);

    switch (mode)
    {
        case 0:  // GPIO模式
            gpio_control(channel, I2C_recvBuf[2]);
            break;

        case 1:  // PWM模式
            pwm_control(channel, I2C_recvBuf[2]);
            break;

        default: 
            // 可以添加错误处理逻辑
            return;
    }

  
    memset(I2C_recvBuf, 0, sizeof(I2C_recvBuf));
}






void gpio_control(uint8_t channel, uint8_t value)
{   
	if (channel < 1 || channel > MAX_GPIO_CHANNELS) 
	{  
        // 错误处理：无效的channel  
        return;  
    }  
  
    value = (value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;  
  
    // 使用数组索引访问对应的GPIO引脚  
    HAL_GPIO_WritePin(gpio_pins[channel - 1].port, gpio_pins[channel - 1].pin, value);  
	printf("gpio_control_ok\r\n");
}




void pwm_control(uint8_t channel, uint16_t value)  
{  

    if (channel > 0 && channel <= max_channels) {
        // 通过Channel直接索引访问对应的PWM通道
        __HAL_TIM_SET_COMPARE(pwm_channels[channel - 1].htim, pwm_channels[channel - 1].tim_channel, value);
		printf("pwm_control_ok\r\n");
    } else {
        // 错误处理：无效的channel

    }

	
}  

























/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
