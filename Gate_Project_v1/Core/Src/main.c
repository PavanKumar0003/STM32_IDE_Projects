/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "Lcd.h"
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

UART_HandleTypeDef huart1;

osThreadId SensorStatusHandle;
osThreadId DisplayTimeDateHandle;
osThreadId GateAnnouncmentHandle;
osThreadId VolumeChangeHandle;
osSemaphoreId Sem01Handle;
osSemaphoreId Sem02Handle;
osSemaphoreId Sem03Handle;
osSemaphoreId Sem04Handle;
/* USER CODE BEGIN PV */
int IR_Value,PIR_Value,Hours_Int=0;
char Hours[2],Min[2],Sec[2]; //Time Stamp

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void Task1_SensorStatus(void const * argument);
void Task2_ReadRTC_WriteLCD_1sec(void const * argument);
void Task3_Gate_Announcement(void const * argument);
void Task4_Volume_Change(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendJQ6500Command(uint8_t commandType) {
    uint8_t command[] = {commandType};
    HAL_UART_Transmit(&huart1, command, sizeof(command), 100);
}

void Audio_Track_Morning(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x01); //File 1
	sendJQ6500Command(0xEF);
}

void Audio_Track_Afternoon(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x02); //File 2
	sendJQ6500Command(0xEF);
}

void Audio_Track_Evening(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x03); //File 3
	sendJQ6500Command(0xEF);
}

void Audio_Track_Welcome(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x04); //File 4
	sendJQ6500Command(0xEF);
}

void Audio_Track_ThankYou(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x05); //File 5
	sendJQ6500Command(0xEF);
}

void Audio_Track_CloseGate(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x06); //File 6
	sendJQ6500Command(0xEF);
}

void Audio_Track_ThankClose(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x04);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x00);
	sendJQ6500Command(0x07); //File 7
	sendJQ6500Command(0xEF);
}

void Audio_Track_VolumeSet(void){
	sendJQ6500Command(0x7E);
	sendJQ6500Command(0x03);
	sendJQ6500Command(0x06);
	sendJQ6500Command(0x1E); //Volume
	sendJQ6500Command(0xEF);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char apndValue[3];
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  RM_LCD_Init();
  RM_LCD_Clear();

  /*Project Welcome Screen*/
  sprintf(apndValue, "Gate Monitoring");  //Hours
  RM_LCD_Write_Str(0,0,apndValue);
  sprintf(apndValue, "V1.0");  //Hours
  RM_LCD_Write_Str(6,1,apndValue);
  HAL_Delay(2000);
  RM_LCD_Clear();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Sem01 */
  osSemaphoreDef(Sem01);
  Sem01Handle = osSemaphoreCreate(osSemaphore(Sem01), 1);

  /* definition and creation of Sem02 */
  osSemaphoreDef(Sem02);
  Sem02Handle = osSemaphoreCreate(osSemaphore(Sem02), 1);

  /* definition and creation of Sem03 */
  osSemaphoreDef(Sem03);
  Sem03Handle = osSemaphoreCreate(osSemaphore(Sem03), 1);

  /* definition and creation of Sem04 */
  osSemaphoreDef(Sem04);
  Sem04Handle = osSemaphoreCreate(osSemaphore(Sem04), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SensorStatus */
  osThreadDef(SensorStatus, Task1_SensorStatus, osPriorityNormal, 0, 128);
  SensorStatusHandle = osThreadCreate(osThread(SensorStatus), NULL);

  /* definition and creation of DisplayTimeDate */
  osThreadDef(DisplayTimeDate, Task2_ReadRTC_WriteLCD_1sec, osPriorityNormal, 0, 128);
  DisplayTimeDateHandle = osThreadCreate(osThread(DisplayTimeDate), NULL);

  /* definition and creation of GateAnnouncment */
  osThreadDef(GateAnnouncment, Task3_Gate_Announcement, osPriorityNormal, 0, 128);
  GateAnnouncmentHandle = osThreadCreate(osThread(GateAnnouncment), NULL);

  /* definition and creation of VolumeChange */
  osThreadDef(VolumeChange, Task4_Volume_Change, osPriorityNormal, 0, 128);
  VolumeChangeHandle = osThreadCreate(osThread(VolumeChange), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|Buzzer_Pin
                          |LCD_D3_Pin|LCD_RS_Pin|LCD_RW_Pin|LCD_EW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin Buzzer_Pin
                           LCD_D3_Pin LCD_RS_Pin LCD_RW_Pin LCD_EW_Pin */
  GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|Buzzer_Pin
                          |LCD_D3_Pin|LCD_RS_Pin|LCD_RW_Pin|LCD_EW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_Sensor_Pin PIR_Sensor_Pin */
  GPIO_InitStruct.Pin = IR_Sensor_Pin|PIR_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1_SensorStatus */
/**
  * @brief  Function implementing the SensorStatus thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task1_SensorStatus */
void Task1_SensorStatus(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  IR_Value = HAL_GPIO_ReadPin(GPIOB,IR_Sensor_Pin);
	  PIR_Value = HAL_GPIO_ReadPin(GPIOB,PIR_Sensor_Pin);
	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2_ReadRTC_WriteLCD_1sec */
/**
* @brief Function implementing the DisplayTimeDate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task2_ReadRTC_WriteLCD_1sec */
void Task2_ReadRTC_WriteLCD_1sec(void const * argument)
{
  /* USER CODE BEGIN Task2_ReadRTC_WriteLCD_1sec */
	char apndValue[3];
	uint8_t Time_Date[8];

  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(Sem01Handle,osWaitForever);
	  //*******DATE AND TIME DATA TO I2C*******//
	  HAL_I2C_Mem_Read(&hi2c1,(0x68<<1),0x00,1, (uint8_t *)&Time_Date,7,1000);

	 //******LCD_PRINT*******//
	 //Print Time->HH:MM:SS(Row-1)//RM_LCD_Goto(0,0);
	 sprintf(apndValue, "%02d",Time_Date[2]-6*(Time_Date[2]>>4));  //Hours
	 RM_LCD_Write_Str(0,0,apndValue);
	 apndValue[0]=0;
	 apndValue[1]=0;
	 strcpy(Hours,apndValue);  //Copy Hours into Hours variable

	 RM_LCD_Write_DATA(':');
	 sprintf(apndValue, "%02d",Time_Date[1]-6*(Time_Date[1]>>4));
	 RM_LCD_PutStr(apndValue);
	 strcpy(Min,apndValue);		//Copy Mins into Min variable

	 RM_LCD_Write_DATA(':');
	 sprintf(apndValue, "%02d",Time_Date[0]-6*(Time_Date[0]>>4));
	 RM_LCD_PutStr(apndValue);
	 strcpy(Sec,apndValue);  //Copy Seconds into Sec variable

	 //Print DATE->(Row-2)//RM_LCD_PutStr("DATE:");
	 RM_LCD_Goto(0,1);
	 sprintf(apndValue, "%02d",Time_Date[4]-6*(Time_Date[4]>>4));
	 RM_LCD_PutStr(apndValue);

	 RM_LCD_Write_DATA('/');
	 sprintf(apndValue, "%02d",Time_Date[5]-6*(Time_Date[5]>>4));
	 RM_LCD_PutStr(apndValue);

	 RM_LCD_Write_DATA('/');
	 sprintf(apndValue, "%02d",Time_Date[6]-6*(Time_Date[6]>>4));
	 RM_LCD_PutStr(apndValue);

	 osDelay(10);
	 osSemaphoreRelease(Sem02Handle);
  }
  /* USER CODE END Task2_ReadRTC_WriteLCD_1sec */
}

/* USER CODE BEGIN Header_Task3_Gate_Announcement */
/**
* @brief Function implementing the GateAnnouncment thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task3_Gate_Announcement */
void Task3_Gate_Announcement(void const * argument)
{
  /* USER CODE BEGIN Task3_Gate_Announcement */
  /* Infinite loop */
	char apndValue[3];
	Audio_Track_VolumeSet();
  for(;;)
  {
	  osSemaphoreWait(Sem02Handle,osWaitForever);
	  if(PIR_Value == 0){    //-> Gate Open and IR Trigger

		  /*Out going Section*/
		  if(IR_Value == 0){ //-> IR

			  /*Thank you Printing*/
			  RM_LCD_Clear();
			  osDelay(1000);
			  RM_LCD_Write_Str(0,0,"Gate Announce");
			  RM_LCD_Goto(5,1);
			  RM_LCD_Write_Str(5,1,"Thank You!");
			  Audio_Track_ThankYou();  //Thank you Announce
			  osDelay(1000);
			  RM_LCD_Clear();
			  osDelay(1000);

			  while(!PIR_Value){ 	//during GATE OPEN
				  RM_LCD_Write_Str(5,1,"Close T");
				  Audio_Track_CloseGate();  //Close Gate Announce
				  osDelay(1000);
				  RM_LCD_Clear();
				  osDelay(1000);
			  }
			  RM_LCD_Clear();
		  }
	  }

	  //-> Gate Open
	  if(PIR_Value==0){
		  /*In Coming Section*/
		  Hours_Int = atoi(Hours);
		  RM_LCD_Clear();
		  osDelay(1000);

		  RM_LCD_Write_Str(0,0,"Gate Announce");
		  RM_LCD_Write_Str(0,1,"Good Evening!");

//		  if(Hours>=4 && Hours < 12){
//			  RM_LCD_Write_Str(0,1,"Good Morning!");
//			  Audio_Track_Morning();
//		  }
//		  else if(Hours>=12 && Hours < 16){
//			  RM_LCD_Write_Str(0,1,"Good Afternoon!");
//			  Audio_Track_Afternoon();
//		  }
//		  else if(Hours>=16 && Hours < 24){
//			  RM_LCD_Write_Str(0,1,"Good Evening!");
//			  Audio_Track_Evening();
//		  }

		  Audio_Track_Evening();  //Evening Announce
		  Audio_Track_Welcome();  //Welcome Announce
		  osDelay(1000);
		  RM_LCD_Clear();
		  osDelay(3000);

		  while(!PIR_Value){ 	//during GATE OPEN
			  RM_LCD_Write_Str(5,1,"Close A");
			  Audio_Track_CloseGate();  //Close Gate Announce
			  osDelay(1000);
			  RM_LCD_Clear();
			  osDelay(1000);
		  }
		  RM_LCD_Clear();
	  }

	  osDelay(10);
	  osSemaphoreRelease(Sem01Handle);
  }
  /* USER CODE END Task3_Gate_Announcement */
}

/* USER CODE BEGIN Header_Task4_Volume_Change */
/**
* @brief Function implementing the VolumeChange thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task4_Volume_Change */
void Task4_Volume_Change(void const * argument)
{
  /* USER CODE BEGIN Task4_Volume_Change */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);

  }
  /* USER CODE END Task4_Volume_Change */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
