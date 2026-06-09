/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	double Kp , Ki , Kd;
	double P , I , D;
	double set_base_speed;
    int pwm;
    double error , pre_error;
    int SPEED_R , SPEED_L;
}pid;
pid x;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


uint8_t hang_cot[4][2] =  {
		                  {11 , 12},
						  {21 , 22},
		                  {31 , 32},
						  {41 , 42},
                          };

 uint8_t rx_scan[2];
 char buffer[200];
 uint16_t index_rx = 0;
 uint8_t rx;
uint8_t IR,IR_status = 1,IR_count;
int status = 1;
int counter = 0;

uint32_t pre_time = 0;

double error = 0;

int sensor[16];

char line_binary[17];// 16 bit + null terminator

uint16_t line_value = 0;

uint8_t LY[16] = {14 , 15 , 7 , 3 , 4 , 14 , 5 , 4 , 13 , 14 , 11 , 12 , 9 , 10 , 8 , 0};

GPIO_TypeDef* sensor_port[16] =
{
    GPIOG, GPIOG, GPIOD, GPIOD,
    GPIOD, GPIOF, GPIOG, GPIOG,
    GPIOE, GPIOE, GPIOE, GPIOE,
    GPIOE, GPIOE, GPIOE, GPIOG
};
uint8_t dir = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void pid_controller(pid *x , double error);
void warning_led(uint16_t timer);

void control_motor(uint16_t xung1 , uint16_t xung2, uint8_t dir1, uint8_t dir2)
{
	  GPIOD -> ODR |= (1 << 11);//EN
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, dir1);//F/R
	  (&htim3) -> Instance -> CCR2 = xung1;

	  GPIOG -> ODR |= (1 << 3);//EN
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, dir2);//F/R
      (&htim3) -> Instance -> CCR1 = xung2;
      //en : 1 là chạy , 0 là dừng
}
void read_sensor_AGV()
{
	     for(int i = 0 ; i < 16 ; i ++)
		 {
		 sensor[i] = (sensor_port[i] -> IDR >> LY[i] & 1);
		 }

		 for(int i = 0; i < 16; i++)
		 {
		 line_binary[i] = sensor[i] ? '1' : '0';
		 }
		 line_binary[16] = '\0';  // Kết thúc chuỗi
}
void follow_line_robot_AGV()
{// tính thêm trư�?ng hợp lệch 5,6
	     if(strcmp(line_binary, "1111110000111111") == 0)
	     {
			 //control_motor(100, 100, 0 , 1);
	    	 error = 0;
	     }
		 //------------------------------lệch trái 4 phân ----------------------------------//
		 if(strcmp(line_binary, "1111100001111111") == 0)
			   error = -1;
		 		// control_motor(100, 100, 0 , 0);// rẽ phải
		 if(strcmp(line_binary, "1111000011111111") == 0)
		 		 //control_motor(100, 100, 0 , 0);
			   error = -1.5;
		 if(strcmp(line_binary, "1110000111111111") == 0)
		 		 //control_motor(100, 100, 0 , 0);
			   error = -2;
		 if(strcmp(line_binary, "1100001111111111") == 0)
		 		 //control_motor(100, 100, 0 , 0);
			    error = -2.5;
		 if(strcmp(line_binary, "1000011111111111") == 0)
		 		// control_motor(100, 100, 0 , 0);
			   error = -3;
		 if(strcmp(line_binary, "0000111111111111") == 0)
		 		// control_motor(100, 100, 0 , 0);
			  error = -3.5;
		 //--------------------------lệch trái 3 phân---------------------------------------//
		 if(strcmp(line_binary,  "1111110001111111") == 0)
			  error = -1;
			 	 //control_motor(100, 100, 0 , 0);
	     if(strcmp(line_binary,  "1111100011111111") == 0){
	    	 error = -1.5;}
			 	// control_motor(100, 100, 0 , 0);
		 if(strcmp(line_binary,  "1111000111111111") == 0){
			 error = -2;}
			 	// control_motor(100, 100, 0 , 0);
	     if(strcmp(line_binary,  "1110001111111111") == 0)
	    	 error = -2.5;
			 	// control_motor(400, 400, 0 , 0);
	     if(strcmp(line_binary,  "1100011111111111") == 0)
	    	 error = -3;
			 	// control_motor(100, 100, 0 , 0);
	     if(strcmp(line_binary,  "1000111111111111") == 0)
	    	 error = -3.5;
	     		 	// control_motor(100, 100, 0 , 0);
	     if(strcmp(line_binary,  "0001111111111111") == 0)
	    	 error = -4;
	     		 	// control_motor(100, 100, 0 , 0);
	    //-----------------------lệch phải 4 phân---------------------------------------//
		 if(strcmp(line_binary, "1111111000011111") == 0)
			   error = 1;
		 		// control_motor(100, 100, 1 , 1);// rẽ phải
		 if(strcmp(line_binary, "1111111100001111") == 0)
			   error = 1.5;
		 		// control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111110000111") == 0)
			   error = 2;
		 		// control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111111000011") == 0)
			   error = 2.5;
		 		 //control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111111100001") == 0)
			   error = 3;
		 		 //control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111111110000") == 0)
			  error = 3.5;
		 		// control_motor(100, 100, 1 , 1);
		 //--------------------lệch phải 3 phân------------------------------------------//
		 if(strcmp(line_binary, "1111111000111111") == 0)
			    error = 1;
		 		 //control_motor(100, 100, 1 , 1);// rẽ trái
		 if(strcmp(line_binary, "1111111100011111") == 0)
		 		// control_motor(100, 100, 1 , 1);
			    error = 1.5;
		 if(strcmp(line_binary, "1111111110001111") == 0)
		 		 //control_motor(100, 100, 1 , 1);
			    error = 2;
		 if(strcmp(line_binary, "1111111111000111") == 0)
		 		// control_motor(100, 100, 1 , 1);
			    error = 2.5;
		 if(strcmp(line_binary, "1111111111100011") == 0)
		 		// control_motor(100, 100, 1 , 1);
			    error = 3;
		 if(strcmp(line_binary, "1111111111110001") == 0)
		 		// control_motor(100, 100, 1 , 1);
			    error = 3.5;
		 if(strcmp(line_binary, "1111111111111000") == 0)
			    // control_motor(100, 100, 1 , 1);
			    error = 4;
	//-------------------------lệch trái 5 phân---------------------------//
		 if(strcmp(line_binary, "1111000001111111") == 0)
		 		 //control_motor(100, 100, 0 , 0);// rẽ phải
			 error = -1;
		 if(strcmp(line_binary, "1110000011111111") == 0)
		 		// control_motor(100, 100, 0 , 0);
			 error = -1.5;
		 if(strcmp(line_binary, "1100000111111111") == 0)
		 		// control_motor(100, 100, 0 , 0);
			 error = -2;
		 if(strcmp(line_binary, "1000001111111111") == 0)
		 		// control_motor(100, 100, 0 , 0);
			 error = -2.5;
		 if(strcmp(line_binary, "0000011111111111") == 0)
		 		// control_motor(100, 100, 0 , 0);
			 error = -3;
   //-----------------------lệch phải 5 phân------------------------------//
		 if(strcmp(line_binary, "1111111000001111") == 0)
		 		// control_motor(100, 100, 1 , 1);// rẽ trái
			 error = 1;
		 if(strcmp(line_binary, "1111111100000111") == 0)
			 error = 1.5;
		 		// control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111110000011") == 0)
			 error = 2;
		 		// control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111111000001") == 0)
			 error = 2.5;
		 		 //control_motor(100, 100, 1 , 1);
		 if(strcmp(line_binary, "1111111111100000") == 0)
		 {
			 error = 3;
		 }
         //control_motor(100, 100, 1 , 1);
		  pid_controller(&x, error);

		  if(dir == 1)
		  {
		  if(error == 0)
		  {
			 control_motor(200, 200, 0 , 1);
		  }
		 if(error == -1 || error == -1.5 || error == -2 || error == -2.5 || error == -3 || error == -3.5 || error == -4)
			 control_motor((&x)->SPEED_R, 0 , 0 , 1);
		 if(error == 1 || error == 1.5 || error == 2 || error == 2.5 || error == 3 || error == 3.5 || error == 4)
			 control_motor(0, (&x)->SPEED_L , 0 , 1);
		 if(strcmp(line_binary, "1111111111111111") == 0)
		 {
			 control_motor(0, 0, 0 , 0);
		 }
		 }
		 if(strcmp(line_binary, "1111110000000000") == 0)
		 {

			 control_motor(100, 100, 0 , 1);
			 HAL_Delay(1500);
			 control_motor(0, 100, 1 , 1);
			 HAL_Delay(1000);
			 while(1)
		  {
			        read_sensor_AGV();
			        warning_led(200);
			        if(strcmp(line_binary, "1111110000111111") == 0 ||strcmp(line_binary, "1111110000011111") == 0 )
	      {
			        break;
		  }
		  }

}
}
void pid_controller(pid *x , double error)
{
  x->error = error;

  x->I += x->error;

  x->D = (double)(x->error - x->pre_error);

  x->pre_error = x->error;

  x->pwm = (x->Kp*x->error) + (x->Ki*x->I) + (x->Kd*x->D);

  x->SPEED_L = (int)x->set_base_speed - x->pwm;
  x->SPEED_R = (int)x->set_base_speed + x->pwm;

  if(x->SPEED_L >= 700) x->SPEED_L = 700;//500
  if(x->SPEED_R >= 700) x->SPEED_R = 700;//500

 // return x->pwm;
}
void warning_led(uint16_t timer)
{
	if(timer != 0)
	{
	if((uint32_t)HAL_GetTick() - pre_time > timer)
	{
		pre_time = HAL_GetTick();
		GPIOE -> ODR ^= (1 << 15);
	}
	}
}
void return_robot()
{
	if((GPIOA -> IDR >> 4 & 1) == 0 && IR_status == 1)
	{
	HAL_Delay(20);
	IR_count++;
	}
	IR_status = GPIOA -> IDR >> 4 & 1;
	if(IR_count == 1)
	{

	while(1)
	{
	dir = 0;
	read_sensor_AGV();
	follow_line_robot_AGV();
	if(error == 0)
	{
	control_motor(100, 100, 1 , 0);
	}
	if(error == -1 || error == -1.5 || error == -2 || error == -2.5 || error == -3)
	control_motor(100,100,  0,  1);
	if(error == 1 || error == 1.5 || error == 2 || error == 2.5 || error == 3)
	control_motor(100,100 , 1 ,0);
	}
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // dùng uart3 , RS485_1_ISO
  // HAL_UART_Receive(&huart3, rx_scan, 2 , 100);// nhớ để trong while(1)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  (&htim3) -> Instance -> PSC = 31;
  (&htim3) -> Instance -> ARR = 999;//1khz
  (&x)->Kp = 55;
  (&x)->Ki = 0;
  (&x)->Kd = 10;
  (&x)->set_base_speed = 300;
  //HAL_UART_Receive_IT(&huart3,&rx,1);
  //HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  //	  sensor[0] = GPIOG -> IDR >> 14 & 1;
  //	  sensor[1] = GPIOG -> IDR >> 15 & 1;
  //	  sensor[2] = GPIOD -> IDR >> 7 & 1;
  //	  sensor[3] = GPIOD -> IDR >> 3 & 1;
  //	  sensor[4] = GPIOD -> IDR >> 4 & 1;
  //	  sensor[5] = GPIOF -> IDR >> 14 & 1;
  //	  sensor[6] = GPIOG -> IDR >> 5 & 1;
  //	  sensor[7] = GPIOG -> IDR >> 4 & 1;
  //	  sensor[8] = GPIOE -> IDR >> 13 & 1;
  //	  sensor[9] = GPIOE -> IDR >> 14 & 1;
  //	  sensor[10] = GPIOE -> IDR >> 11 & 1;
  //	  sensor[11] = GPIOE -> IDR >> 12 & 1;
  //	  sensor[12] = GPIOE -> IDR >> 9 & 1;
  //	  sensor[13] = GPIOE -> IDR >> 10 & 1;
  //	  sensor[14] = GPIOE -> IDR >> 8 & 1;
  //	  sensor[15] = GPIOG -> IDR >> 0 & 1;
  //EN = 0  LÀ CHẠY , 1 LÀ DỪNG
  // FR = 1 -> t , 0-> N
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//LM1 PD9 , LM2 : PF15
	HAL_UART_Receive(&huart3, rx_scan,sizeof(rx_scan) ,100);
	//return_robot();
	//read_sensor_AGV();
//	if((GPIOC -> IDR >> 5 & 1) == 0 && (status == 1))
//	{
//		HAL_Delay(20);
//		counter++;
//	}
//	status = GPIOC -> IDR >> 5 & 1;
//
//    if(counter % 2 == 0)
//    {
//    if((GPIOG -> IDR >> 1 & 1) == 1)
//	{
//		control_motor(0, 0, 0, 0);// stop robot
//	}else
//	{
//	if(dir == 1)
//	{
//	warning_led(200);
//	follow_line_robot_AGV();
//	}
//	}
//    }
//    if(counter % 2 == 1)
//    {
//    	GPIOE -> ODR &=~ (1 << 15);
//    	control_motor(0, 0, 0, 0);
//    }
	  //control_motor(400, 400, 0 , 1);// tiến
	  //control_motor(400, 400, 1, 0);//lùi
	  //control_motor(400, 400, 1, 1);//phải
	  //control_motor(400, 400, 0, 0);//trái
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 PF9 PF10
                           PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG4 PG5
                           PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11
                           PE12 PE13 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
