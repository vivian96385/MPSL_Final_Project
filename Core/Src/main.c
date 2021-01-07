/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "accel.h"
#include <stdio.h>
#include <string.h>
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int brickRowCount = 5;
int brickColumnCount = 11;
int brickWidth = 10;
int brickHeight = 10;
int brickPadding = 1;
int brickOffsetTop = 1;
int brickOffsetLeft = 1;

int ball_radius = 4;
int ball_x = 50;
int ball_y = 90;
int ball_x_speed = 2;
int ball_y_speed = 2;

int platform_x = 60;
int platform_y = 120;
int platform_width = 30;
int platform_height = 5;

char sprintbuff[64] = "";
unsigned char uartbuff[64];
float tmp1 = 0.0;
float tmp2 = 0.0;
float tmp3 = 0.0;
float tmp4 = 0.0;

int score = 0;

struct Brick {
	int x;
	int y;
	int status;
};

struct Brick Brick_Field[15][15];

void collisionDetection(){
	for (int row=0; row<brickRowCount; row++)
	{
		for (int col=0; col<brickColumnCount; col++)
		{
			struct Brick b = Brick_Field[row][col];
			if (b.status == 1)
			{
				//left -> right side
				if(ball_x+ball_radius>=b.x && ball_x+ball_radius<=b.x+brickWidth && ball_y>b.y && ball_y<b.y+brickHeight)
				{
					ball_x_speed = -ball_x_speed;
					Brick_Field[row][col].status = 0;
					fillRect(Brick_Field[row][col].x, Brick_Field[row][col].y, brickWidth, brickHeight, BLACK);
					score++;
				}
				//right -> left side
				else if (ball_x-ball_radius>=b.x && ball_x-ball_radius<=b.x+brickWidth && ball_y>b.y && ball_y<b.y+brickHeight)
				{
					ball_x_speed = -ball_x_speed;
					Brick_Field[row][col].status = 0;
					fillRect(Brick_Field[row][col].x, Brick_Field[row][col].y, brickWidth, brickHeight, BLACK);
					score++;
				}
				//up -> down side
				else if (ball_y+ball_radius>=b.y && ball_y+ball_radius<=b.y+brickHeight && ball_x>b.x && ball_x<b.x+brickWidth)
				{

					ball_y_speed = -ball_y_speed;
					Brick_Field[row][col].status = 0;
					fillRect(Brick_Field[row][col].x, Brick_Field[row][col].y, brickWidth, brickHeight, BLACK);
					score++;
				}
				//down -> up side
				else if (ball_y-ball_radius>=b.y && ball_y-ball_radius<=b.y+brickHeight && ball_x>b.x && ball_x<b.x+brickWidth)
				{

					ball_y_speed = -ball_y_speed;
					Brick_Field[row][col].status = 0;
					fillRect(Brick_Field[row][col].x, Brick_Field[row][col].y, brickWidth, brickHeight, BLACK);
					score++;
				}
			}
		}
	}
}


void game(){
	while (1)
	  {
	    /* USER CODE END WHILE */
		uint8_t		imu_readings[IMU_NUMBER_OF_BYTES]={0};
		int16_t 	accel_data[3];
		float		acc_x, acc_y, acc_z;

		  //testFastLines(RED, BLUE);
		  GetAccelData(&hi2c1, (uint8_t*)imu_readings);
		  accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
		  accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
		  accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
		  acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
		  acc_y = ((float)(accel_data[1]))/100.0f;
		  acc_z = ((float)(accel_data[2]))/100.0f;
		  if(acc_y<200 && acc_y>-200.0)
		  {
			  if(tmp1>2.0&&tmp2>2.0&&tmp3>2.0&&tmp4>3.0&&acc_y>2.0)
			  {
	//			  sprintf(sprintbuff,"0\n");
	//			  memcpy(uartbuff,sprintbuff,sizeof(sprintbuff));
	//			  HAL_UART_Transmit(&huart2,uartbuff,sizeof(uartbuff),1000);
				  if(platform_x<=98)
				  {
					  fillRect(platform_x+5, platform_y, platform_width, platform_height, GREEN);
					  fillRect(platform_x, platform_y, 5, platform_height, BLACK);
					  platform_x+=5;
				  }

			  }
			  else if(tmp1<-2.0&&tmp2<-2.0&&tmp3<-2.0&&tmp4<-2.0&&acc_y<-2.0)
			  {
				  //sprintf(sprintbuff,"1\n");
				  //memcpy(uartbuff,sprintbuff,sizeof(sprintbuff));
				  //HAL_UART_Transmit(&huart2,uartbuff,sizeof(uartbuff),1000);
				  if(platform_x>=0 )
				  {
					  fillRect(platform_x-5, platform_y, platform_width, platform_height, GREEN);
					  fillRect(platform_x+29, platform_y, 5, platform_height, BLACK);
					  platform_x-=5;
				  }

			  }
			  tmp4 = tmp3;
			  tmp3 = tmp2;
			  tmp2 = tmp1;
			  tmp1 = acc_y;
		  }

		  collisionDetection();
		  drawCircle(ball_x, ball_y, 4, BLACK);
		  //board
		  if((ball_y+ball_radius==platform_y-2 && ball_x>platform_x-8 && ball_x<platform_x+platform_width+8)|| ball_y+ball_radius<=0)
		  {
			  ball_y_speed = -ball_y_speed;

			  if(ball_x>platform_x-8 && ball_x<(platform_x+(platform_width/3))) //left 1/3
			  {
				  if(ball_x_speed>=2)
					  ball_x_speed--;
			  }
			  else if(ball_x>(platform_x+2*(platform_width/3)) && ball_x<platform_x+platform_width+8) // right 1/3
			  {
				  if(ball_x_speed<=5)
				  		ball_x_speed++;
			  }
			  else // middle 1/3
			  {

			  }
		  }

		  if(ball_x <= 4 || ball_x >= 124) ball_x_speed = -ball_x_speed;
		  ball_x += ball_x_speed;
		  ball_y += ball_y_speed;
		  drawCircle(ball_x, ball_y, 4, CYAN);

		  if (ball_y>=124)
		  {
			  break;
		  }

		  //}
		  //HAL_Delay(50);

	    /* USER CODE BEGIN 3 */
	  }
	  /* USER CODE END 3 */
}

void game_reset(){
	ball_x = 50;
	ball_y = 90;
	ball_x_speed = 2;
	ball_y_speed = 2;
	platform_x = 60;
	platform_y = 120;
	score = 0;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  ST7735_Init(0);
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  BNO055_Init_I2C(&hi2c1);
  /* USER CODE END 2 */
  //fillScreen(BLACK);
  //testAll();

  /* USER CODE BEGIN WHILE */
  //unsigned char address = 0x28;
  while(1){

	  /* construct block field */
	  fillScreen(BLACK);

	  for(int row=0; row<brickRowCount; row++)
	  {
		  for(int col=0; col<brickColumnCount; col++)
		  {
			  Brick_Field[row][col].x = (col*(brickWidth+brickPadding))+brickOffsetLeft + 4;
			  Brick_Field[row][col].y = (row*(brickHeight+brickPadding))+brickOffsetTop;
			  Brick_Field[row][col].status = 1;
			  fillRect(Brick_Field[row][col].x, Brick_Field[row][col].y, brickWidth, brickHeight, YELLOW);
		  }
	  }

	  fillRect(platform_x, platform_y, platform_width, platform_height, GREEN);
	  drawCircle(50, 90, 4, CYAN);

	  /* wait for start signal*/
	  uint8_t value = 0;
	  while(!value)
	  {
		  HAL_UART_Receive(&huart2,(uint8_t *)&value,1, 1000);
	  }


	  /* game */
	  game();

	  /* wait for retry signal*/

	  /* send score */
	  sprintf(sprintbuff,"%d\n", score);
	  memcpy(uartbuff,sprintbuff,sizeof(sprintbuff));
	  HAL_UART_Transmit(&huart2,uartbuff,sizeof(uartbuff),1000);

	  /* reset */
	  game_reset();

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
