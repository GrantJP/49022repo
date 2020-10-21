/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Addresses of the registers

#define _ENABLE  0x80    // Enable status and interrupts

#define _ATIME   0x81    // RGBC ADC time

#define _PTIME	 0x82    // proximity time

#define _WTIME   0x83    // Wait time

#define _AILTL   0x84    // clear Interrupt low treshold low byte

#define _AILTH   0x85    // clear Interrupt low treshold high byte

#define _AIHTL   0x86    // clear Interrupt high treshold low byte

#define _AIHTH   0x87    // clear Interrupt high treshold high byte



#define _PILTL	 0x88	 // proximity interrupt low threshold low byte

#define _PILTH	 0x89	 // proximity interrupt low threshold high byte

#define _PIHTL	 0x8A	 // proximity interrupt high threshold low byte

#define _PIHTH	 0x8B	 // proximity interrupt high threshold high byte



#define _PERS    0x8C    // Interrupt persistence filters

#define _CONFIG  0x8D    // Configuration

#define _CONTROL 0x8F    // Gain control register

#define _ID      0x92    // Device ID

#define _STATUS  0x93    // Device status

#define _CDATA   0x94    // Clear ADC low data register

#define _CDATAH  0x95    // Clear ADC high data register

#define _RDATA   0x96    // RED ADC low data register

#define _RDATAH  0x97    // RED ADC high data register

#define _GDATA   0x98    // GREEN ADC low data register

#define _GDATAH  0x99    // GREEN ADC high data register

#define _BDATA   0x9A    // BLUE ADC low data register

#define _BDATAH  0x9B    // BLUE ADC high data register



#define _PDATA	 0x9C	 // Proximity ADC data low byte

#define _PDATAH	 0x9D 	 // Proximity ADC data high byte



#define _COLOR_W_ADDRESS 0x52  //color address with a zero

#define _COLOR_R_ADDRESS 0x53  // color address with a following one

#define _COLOR_ADDRESS 	 0x29   // slave address but unsure where it came from







#define PURPLE_FLAG 1

#define BLUE_FLAG   2

#define CYAN_FLAG   3

#define GREEN_FLAG  4

#define PINK_FLAG   5

#define RED_FLAG    6

#define ORANGE_FLAG 7

#define YELLOW_FLAG 8



#define _GAIN_x4 1  0x01 	// seems like these should be written as bx01 and such

#define _GAIN_x16   0x10

#define _GAIN_x60   0x11



char i, color_detected, color_flag;

unsigned char buffer[5];

unsigned int Clear, Red, Green, Blue;

float hue, color_value, color_value_sum;

float Red_Ratio, Green_Ratio, Blue_Ratio;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t bufferRFID[14];
uint8_t tag1[14] = {0x02, 54,48,48,48,51,65,53,51,50,53,50,67,0x03};
uint8_t tag2[14] = {2, 54,48,48,48,51,67,49,65,68,65,57,67,3};
uint8_t tag3[14] = {0x02, 54,48,48,48,51,65,54,56,54,54,53,52,0x03};
uint8_t tag4[14] = {2, 54,48,48,48,51,67,49,65,69,49,65,55,3};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned int Color_Read_value(char reg) {

  unsigned short low_byte;

  uint16_t Out_color;



  switch(reg) {





    case 'C': buffer[0] =0x94;

    		  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);



    		  HAL_Delay(20);



    		  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

    		  low_byte = buffer[0];

    		  buffer[0] = 0x95;

    		  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);

    		  HAL_Delay(20);



    		  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

    		  Out_color = buffer[0];





    		  //low_byte = Color_Read(_CDATA);

              //Out_color = Color_Read(_CDATAH);

              Out_color = (Out_color << 8);

              Out_color = (Out_color | low_byte);

              //Out_color = 0x0001;

              return Out_color;

              break;



    case 'R': buffer[0] =0x96;

	  	  	  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);



	  	  	  HAL_Delay(20);



	  	  	  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

	  	  	  low_byte = buffer[0];

	  	  	  buffer[0] = 0x97;

	  	  	  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);

	  	  	  HAL_Delay(20);



	  	  	  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

	  	  	  Out_color = buffer[0];





	  	  	  //low_byte = Color_Read(_CDATA);

	  	  	  //Out_color = Color_Read(_CDATAH);

	  	  	  Out_color = (Out_color << 8);

	  	  	  Out_color = (Out_color | low_byte);

	  	  	  //Out_color = 0xFF05;

	  	  	  return Out_color;

	  	  	  break;



    case 'G': buffer[0] =0x98;

	  	  	  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);



	  	  	  HAL_Delay(20);



	  	  	  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

	  	  	  low_byte = buffer[0];

	  	  	  buffer[0] = 0x99;

	  	  	  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);

	  	  	  HAL_Delay(20);



	  	  	  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

	  	  	  Out_color = buffer[0];





	  	  	  //low_byte = Color_Read(_CDATA);

	  	  	  //Out_color = Color_Read(_CDATAH);

			  Out_color = (Out_color << 8);

			  Out_color = (Out_color | low_byte);

			  //Out_color = 0x0006;

			  return Out_color;

			  break;



    case 'B': buffer[0] =0x9A;

	  	  	  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);



	  	  	  HAL_Delay(20);



	  	  	  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

	  	  	  low_byte = buffer[0];

	  	  	  buffer[0] = 0x9B;

	  	  	  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1,buffer,1,100);

	  	  	  HAL_Delay(20);



	  	  	  HAL_I2C_Master_Receive(&hi2c1, _COLOR_ADDRESS<<1,buffer,1, 100);

	  	  	  Out_color = buffer[0];





	  	  	  //low_byte = Color_Read(_CDATA);

	  	  	  //Out_color = Color_Read(_CDATAH);

	  	  	  Out_color = (Out_color << 8);

	  	  	  Out_color = (Out_color | low_byte);

	  	  	  //Out_color = 0x001;

	  	  	  return Out_color;

	  	  	  break;



    default:  return 0;

  }

}










void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//GPIOA->BSRR |= 0x00000100;
//void RFIDmatcher(){

if(memcmp(bufferRFID,tag1,14) == 0){
	detected1 = 1;
}

if(memcmp(bufferRFID,tag2,14)==0){
	detected2 = 1;
}

if(memcmp(bufferRFID,tag3,14)==0){
	detected3 = 1;
}

if(memcmp(bufferRFID,tag4,14)==0){
	detected4 = 1;
}


}

void RFIDcaller(void){
	  //HAL_UART_Receive_IT(&huart1,bufferRFID,14);

		detected1=0;
		detected2=0;
		detected3=0;
		detected4=0;

		//for (int j=0 ; j<10000;j++){
		//	  HAL_UART_Receive_IT(&huart1,bufferRFID,14);
		//}

		//LED for testing
		GPIOB->BSRR |= 0x00000800;
		for (int i=0; i<100;i++){
 		  HAL_Delay(50);
 		  HAL_UART_Receive_IT(&huart1,bufferRFID,14);
		}
		GPIOB->BSRR |= 0x08000000;
		//detected2 = 1;

 }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	detected1 = 0;
	detected2 = 0;
	detected3 = 0;
	detected4 = 0;


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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  scoreA = 0;
  scoreB = 0;

  bagsHoleA = 0;
  bagsHoleB = 0;

  bagsBoardA = 0;
  bagsBoardB = 0;
  throwFirst = 1;


  DisplayScore();


  bagsHoleB = 0;



  color_value_sum = 0;

  color_detected = 0;



  buffer[0] = _ENABLE;

  buffer[1] = 0x0F;  // value to be written to enabled



  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1, buffer, 2, 100); //writes enabled



  buffer[0] = _CONTROL;

  buffer[1] = 0x00;

  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1, buffer, 2, 100);





  //Color_Write(_CONTROL, _GAIN_x16); // again may need to adjust gain number

  buffer[0] = _ATIME;

  buffer[1] = 0x00;

  HAL_I2C_Master_Transmit(&hi2c1, _COLOR_ADDRESS<<1, buffer, 2, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  color_detected = 0;

	  	  Sum_Red = 0;

	  	  Sum_Blue = 0;





	  	  	      // Get the average color value of 16 measurements



	  	  	         //Read Clear, Red, Green and Blue channel register values

	  	  	         for(i=0;i<16;i++){

	  	  	  	  	 Clear = Color_Read_value('C');

	  	  	         Red = Color_Read_value('R');

	  	  	         //Green = Color_Read_value('G');

	  	  	         Blue = Color_Read_value('B');



	  	  	        // Divide Red, Green and Blue values with Clear value

	  	  	        Red_Ratio   = ((float)Red   / (float)Clear);

	  	  	        Blue_Ratio	= ((float)Blue   / (float)Clear);

	  	  	        Sum_Red += Red_Ratio;

	  	  	        Sum_Blue += Blue_Ratio;

	  	  	         }

	  	  	         Red_Score_Ratio = Sum_Red/i;

	  	  	         Blue_Score_Ratio = Sum_Blue/i;

	  	  	        // Convert RGB values to HSL values

	  	  	        //color_value = RGB_To_HSL(Red_Ratio, Green_Ratio, Blue_Ratio);



	  	  	        // Sum the color values

	  	  	        //color_value_sum = color_value_sum + color_value;



	  	  	      //color_value = color_value_sum / 16.0;



	  	  	      if (Red_Score_Ratio > 0.53) {

	  	  	            color_detected = 1;

	  	  	            if (color_flag != RED_FLAG){

	  	  	              color_flag = RED_FLAG;

	  	  	              bagsHoleA ++;
	  	  	              GPIOB->BSRR|= 0x00000800;


	  	  	            }

	  	  	          }

	  	  	      else if (Blue_Score_Ratio >.38) {

	  	  	            color_detected = 1;


	  	  	            if (color_flag != BLUE_FLAG){

	  	  	              color_flag = BLUE_FLAG;

	  	  	              bagsHoleB ++;

	  	  	              GPIOB->BSRR|= 0x00000800;


	  	  	            }

	  	  	          }

	  	  	      else {

	  	  	        if (color_detected == 0){

	  	  	          color_flag = 0;

	  	  	          HAL_Delay(100);

  	  	              GPIOB->BSRR|= 0x08000000;


	  	  	      }

	  	  	      }

		while ((scoreA >= 21 || scoreB >= 21) && scoreA != scoreB){
			if (scoreA >= scoreB){
				DisplayNumA10(10);//arg outside 0-9 clears display
				DisplayNumA1(10);
			}
			else{
				DisplayNumB10(10);
				DisplayNumB1(10);
			}
			//wait a momement
			for(int i = 0; i<100000; i++){
				;
			}
			DisplayNumA10((scoreA/10)%10);
			DisplayNumA1(scoreA%10);
			DisplayNumB10((scoreB/10)%10);
			DisplayNumB1(scoreB%10);
			for(int i = 0; i<100000; i++){
				;
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC1 PC2 PC3 PC6
                           PC7 PC8 PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB13 PB14 PB15
                           PB3 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void DisplayScore(){

//while scoreA|scoreB >= 21     flash winner score(loop display off, display num)

	//otherwise
	DisplayNumA10((scoreA/10)%10);
	DisplayNumA1(scoreA%10);
	DisplayNumB10((scoreB/10)%10);
	DisplayNumB1(scoreB%10);
	if (throwFirst==0){
		GPIOB->BSRR |= 0x04000008;
	}
	else {
		GPIOB->BSRR |= 0x00080400;
	}

}

void DisplayNumB10(int num){
	//clockwise from top, A:PC12  B:PC11  C:PC10  D:PB4  E:PB5  F:PB9  G:PB8
    switch(num){
    case 0: GPIOB->BSRR |= 0x02300100;
            GPIOC->BSRR |= 0x1C000000;
            break;
    case 1: GPIOB->BSRR |= 0x00000330;
            GPIOC->BSRR |= 0x0C001000;
            break;
    case 2: GPIOB->BSRR |= 0x01300200;
            GPIOC->BSRR |= 0x18000400;
            break;
    case 3: GPIOB->BSRR |= 0x01100220;
            GPIOC->BSRR |= 0x1C000000;
            break;
    case 4: GPIOB->BSRR |= 0x03000030;
            GPIOC->BSRR |= 0x0C001000;
            break;
    case 5: GPIOB->BSRR |= 0x03100020;
            GPIOC->BSRR |= 0x14000800;
            break;
    case 6: GPIOB->BSRR |= 0x03300000;
            GPIOC->BSRR |= 0x14000800;
            break;
    case 7: GPIOB->BSRR |= 0x00000330;
            GPIOC->BSRR |= 0x1C000000;
            break;
    case 8: GPIOB->BSRR |= 0x03300000;
            GPIOC->BSRR |= 0x1C000000;
            break;
    case 9: GPIOB->BSRR |= 0x03100020;
            GPIOC->BSRR |= 0x1C000000;
            break;

    default: GPIOB->BSRR |= 0x00000330;
            GPIOC->BSRR |= 0x00001C00;
            break;
    }
}


void DisplayNumA10(int num){
	//clockwise from top, A:PC13  B:PC14  C:PC15  D:PC3  E:PC2  F:PC0  G:PC1
    switch(num){
    case 0: GPIOC->BSRR |= 0xE00D0002;
            break;
    case 1: GPIOC->BSRR |= 0xC000200F;
            break;
    case 2: GPIOC->BSRR |= 0x600E8001;
            break;
    case 3: GPIOC->BSRR |= 0xE00A0005;
            break;
    case 4: GPIOC->BSRR |= 0xC003200C;
            break;
    case 5: GPIOC->BSRR |= 0xA00B4004;
            break;
    case 6: GPIOC->BSRR |= 0xA00F4000;
            break;
    case 7: GPIOC->BSRR |= 0xE000000F;
            break;
    case 8: GPIOC->BSRR |= 0xE00F0000;
            break;
    case 9: GPIOC->BSRR |= 0xE00B0004;
            break;
    default: GPIOC->BSRR |= 0x0000E00F;
            break;
    }

}

void DisplayNumB1(int num){
	//clockwise from top, A:PB15  B:PB14  C:PB13  D:PC6  E:PC7  F:PC9  G:PC8
    switch(num){
    case 0: GPIOC->BSRR |= 0x02C00100;
            GPIOB->BSRR |= 0xE0000000;
            break;
    case 1: GPIOC->BSRR |= 0x000003C0;
            GPIOB->BSRR |= 0x60008000;
            break;
    case 2: GPIOC->BSRR |= 0x01C00200;
            GPIOB->BSRR |= 0xC0002000;
            break;
    case 3: GPIOC->BSRR |= 0x01400280;
            GPIOB->BSRR |= 0xE0000000;
            break;
    case 4: GPIOC->BSRR |= 0x030000C0;
            GPIOB->BSRR |= 0x60008000;
            break;
    case 5: GPIOC->BSRR |= 0x03400080;
            GPIOB->BSRR |= 0xA0004000;
            break;
    case 6: GPIOC->BSRR |= 0x03C00000;
            GPIOB->BSRR |= 0xA0004000;
            break;
    case 7: GPIOC->BSRR |= 0x000003C0;
            GPIOB->BSRR |= 0xE0000000;
            break;
    case 8: GPIOC->BSRR |= 0x03C00000;
            GPIOB->BSRR |= 0xE0000000;
            break;
    case 9: GPIOC->BSRR |= 0x03400080;
            GPIOB->BSRR |= 0xE0000000;
            break;

    default: GPIOC->BSRR |= 0x000003C0;
            GPIOB->BSRR |= 0x0000E000;
            break;
    }

}

void DisplayNumA1(int num){
	//clockwise from top, A:PB0  B:PB1  C:PB2  D:PA7  E:PA6  F:PA4  G:PA5
    switch(num){
    case 0: GPIOA->BSRR |= 0x00D00020;
            GPIOB->BSRR |= 0x00070000;
            break;
    case 1: GPIOA->BSRR |= 0x000000F0;
            GPIOB->BSRR |= 0x00060001;
            break;
    case 2: GPIOA->BSRR |= 0x00E00010;
            GPIOB->BSRR |= 0x00030004;
            break;
    case 3: GPIOA->BSRR |= 0x00A00050;
            GPIOB->BSRR |= 0x00070000;
            break;
    case 4: GPIOA->BSRR |= 0x003000C0;
            GPIOB->BSRR |= 0x00060001;
            break;
    case 5: GPIOA->BSRR |= 0x00B00040;
            GPIOB->BSRR |= 0x00050002;
            break;
    case 6: GPIOA->BSRR |= 0x00F00000;
            GPIOB->BSRR |= 0x00050002;
            break;
    case 7: GPIOA->BSRR |= 0x000000F0;
            GPIOB->BSRR |= 0x00070000;
            break;
    case 8: GPIOA->BSRR |= 0x00F00000;
            GPIOB->BSRR |= 0x00070000;
            break;
    case 9: GPIOA->BSRR |= 0x00B00040;
            GPIOB->BSRR |= 0x00070000;
            break;
    default: GPIOA->BSRR |= 0x000000F0;
            GPIOB->BSRR |= 0x00000007;
            break;
    }

}

void ComputeScore(){
	//get new RFID data
	RFIDcaller();

	//move RFID to counts
	bagsBoardA = detected1 + detected2;
	bagsBoardB = detected3 + detected4;

	int roundA = bagsBoardA + 3*bagsHoleA;
	int roundB = bagsBoardB + 3*bagsHoleB;
	if (roundA > roundB){
		scoreA += roundA-roundB;
		throwFirst = 0;
	}
	else{
		scoreB += roundB-roundA;
	}
	if (roundB > roundA){
		throwFirst = 1;
	}
	//reset some vars
	bagsHoleA=0;
	bagsHoleB=0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
