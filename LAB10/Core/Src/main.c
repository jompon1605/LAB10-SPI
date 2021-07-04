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
#include <math.h>
#include <stdio.h>
#include <string.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char RxDataBuffer[32] = { 0 };
int state = 0;
double Frequency = 1.0;
double High = 3.3; //max 3.3 V
double Low = 0.0; //min 0 V
int DutyCycle = 100;
int Slope = 0;
int WaveForm = 0;
double WaveGenOut = 0.0;
double Theta = 0;
int check = 0;
int check2 = 0;

char Word1[] = {" Choose Wave Form\r\n SawTooth Wave : Press a\r\n Sine Wave : Press b\r\n Square : Press c \r\n"};
char Word2[] = {" SawTooth Wave Menu\r\n Set Frequency : Press f\r\n Set Highest Voltage : Press h\r\n Set Lowest Voltage : Press l\r\n Set Slope : Press s\r\n Back : Press x\r\n"};
char Word3[] = {" Sine Wave Menu\r\n Set Frequency : Press f\r\n Set Highest Voltage : Press h\r\n Set Lowest Voltage : Press l\r\n Back : Press x\r\n"};
char Word4[] = {" Square Wave Menu\r\n Set Frequency : Press f\r\n Set Highest Voltage : Press h\r\n Set Lowest Voltage : Press l\r\n Set Duty Cycle : Press d\r\n Back : Press x\r\n"};
char Word5[] = {" Set Frequency Menu\r\n Increase Frequency 0.1Hz : Press +\r\n Decrease Frequency 0.1Hz : Press -\r\n Back : Press x\r\n"};
char Word6[] = {" Set Highest Voltage Menu\r\n Increase Highest Voltage 0.1V : Press +\r\n Decrease Highest Voltage 0.1V : Press -\r\n Back : Press x\r\n"};
char Word7[] = {" Set Lowest Voltage Menu\r\n Increase Lowest Voltage 0.1V : Press +\r\n Decrease Lowest Voltage 0.1V : Press -\r\n Back : Press x\r\n"};
char Word8[] = {" Set Slope Menu\r\n Slope Up : Press 1\r\n Slope Down : Press 2\r\n Back : Press x\r\n"};
char Word9[] = {" Set Duty Cycle\r\n Increase Duty Cycle For 5% : Press +\r\n Decrease Duty Cycle For 5% : Press -\r\n Back : Press x\r\n"};
char Word11[] = {" Slope Up | Increasing Slope\r\n"};
char Word12[] = {" Slope Down | Decreasing Slope\r\n"};
char Word13[] = {" Back\r\n"};
char WordError[] = {"Error : Highest Voltage Can't Less Than Lowest Voltage\r\n"};

char ShowFrequency[32] = {0}; //Frequency
char ShowDutyCycle[32] = {0}; //Duty cycle
char ShowVoltage[64] = {0}; //High | Low



uint16_t ADCin = 0;
uint64_t _micro = 0;
uint16_t dataOut = 0;
uint8_t DACConfig = 0b0011;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

int16_t UARTRecieveIT();
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);

	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);
		int16_t inputchar = UARTRecieveIT();
		switch(state)
		{
			case 0: //menu
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)Word1, strlen(Word1), 1000);
				state = 1;
				break;
			}
			case 1: //choose wave form
			{
				if(inputchar == 'a') //sawtooth wave
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)Word2, strlen(Word2), 1000);
					state = 2;
					WaveForm = 1;
					break;
				}
				else if(inputchar == 'b') //sine wave
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)Word3, strlen(Word3), 1000);
					state = 3;
					WaveForm = 2;
					break;
				}
				else if(inputchar == 'c') //square wave
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)Word4, strlen(Word4), 1000);
					state = 4;
					WaveForm = 3;
					break;
				}
				break;
			}
			case 2: //sawtooth wave menu
			{
				if(inputchar == 'f') //frequency
				{
					state = 5;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word5, strlen(Word5), 1000);
					break;
				}
				else if(inputchar == 'h') //set high
				{
					state = 6;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word6, strlen(Word6), 1000);
					break;
				}
				else if(inputchar == 'l') //set low
				{
					state = 7;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word7, strlen(Word7), 1000);
					break;
				}
				else if(inputchar == 's') //slope
				{
					state = 8;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word8, strlen(Word8), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					state = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
					break;
				}
				break;
			}
			case 3: //sine wave menu
			{
				if(inputchar == 'f') //frequency
				{
					state = 5;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word5, strlen(Word5), 1000);
					break;
				}
				else if(inputchar == 'h') //set high
				{
					state = 6;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word6, strlen(Word6), 1000);
					break;
				}
				else if(inputchar == 'l') //set low
				{
					state = 7;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word7, strlen(Word7), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					state = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
					break;
				}
				break;
			}
			case 4: //square wave menu
			{
				if(inputchar == 'f') //frequency
				{
					state = 5;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word5, strlen(Word5), 1000);
					break;
				}
				else if(inputchar == 'h') //set high
				{
					state = 6;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word6, strlen(Word6), 1000);
					break;
				}
				else if(inputchar == 'l') //set low
				{
					state = 7;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word7, strlen(Word7), 1000);
					break;
				}
				else if(inputchar == 'd') //duty cycle
				{
					state = 9;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word9, strlen(Word9), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					state = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
					break;
				}
				break;
			}
			case 5: //frequency menu
			{
				if(inputchar == '+')
				{
					Frequency += 0.1;
					if(Frequency > 10)
					{
						Frequency = 10;
					}
					sprintf(ShowFrequency, "Current Frequency : %f Hz\r\n", Frequency);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowFrequency, strlen(ShowFrequency), 1000);
					break;
				}
				else if(inputchar == '-')
				{
					Frequency -= 0.1;
					if(Frequency < 0)
					{
						Frequency = 0;
					}
					sprintf(ShowFrequency, "Current Frequency : %f Hz\r\n", Frequency);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowFrequency, strlen(ShowFrequency), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					if(WaveForm == 1)
					{
						state = 2;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word2, strlen(Word2), 1000);
					}
					else if(WaveForm == 2)
					{
						state = 3;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word3, strlen(Word3), 1000);
					}
					if(WaveForm == 3)
					{
						state = 4;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word4, strlen(Word4), 1000);
					}
					break;
				}
				break;
			}
			case 6: //high menu
			{
				if(inputchar == '+')
				{
					High += 0.1;
					if(High > 3.3)
					{
						High = 3.3;
					}
					sprintf(ShowVoltage, " High Voltage : %f V | Low Voltage : %f V  \r\n" , High , Low);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowVoltage, strlen(ShowVoltage), 1000);
					break;
				}
				else if(inputchar == '-')
				{
					High -= 0.1;
					if(High < 0)
					{
						High = 0;
					}
					if(High < Low)
					{
						High += 0.1;
						HAL_UART_Transmit(&huart2, (uint8_t*)WordError, strlen(WordError), 1000);
					}
					sprintf(ShowVoltage, " High Voltage : %f V | Low Voltage : %f V \r\n" , High , Low);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowVoltage, strlen(ShowVoltage), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					if(WaveForm == 1)
					{
						state = 2;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word2, strlen(Word2), 1000);
					}
					else if(WaveForm == 2)
					{
						state = 3;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word3, strlen(Word3), 1000);
					}
					if(WaveForm == 3)
					{
						state = 4;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word4, strlen(Word4), 1000);
					}
					break;
				}
				break;
			}
			case 7: //low menu
			{
				if(inputchar == '+')
				{
					Low += 0.1;
					if(Low > High + 0.1)
					{
						Low -= 0.1;
						HAL_UART_Transmit(&huart2, (uint8_t*)WordError, strlen(WordError), 1000);
					}
					sprintf(ShowVoltage, " High Voltage : %f V | Low Voltage : %f V \r\n" , High , Low);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowVoltage, strlen(ShowVoltage), 1000);
					break;
				}
				else if(inputchar == '-')
				{
					Low -= 0.1;
					if(Low < 0)
					{
						Low = 0;
					}
					sprintf(ShowVoltage, " High Voltage : %f V | Low Voltage : %f V \r\n" , High , Low);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowVoltage, strlen(ShowVoltage), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					if(WaveForm == 1)
					{
						state = 2;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word2, strlen(Word2), 1000);
					}
					else if(WaveForm == 2)
					{
						state = 3;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word3, strlen(Word3), 1000);
					}
					if(WaveForm == 3)
					{
						state = 4;
						HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
						HAL_UART_Transmit(&huart2, (uint8_t*)Word4, strlen(Word4), 1000);
					}
					break;
				}
				break;
			}
			case 8: //slope menu
			{
				if(inputchar == '1')
				{
					Slope = 1;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word11, strlen(Word11), 1000);
					break;
				}
				else if(inputchar == '2')
				{
					Slope = 2;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word12, strlen(Word12), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					state = 2;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*)Word2, strlen(Word2), 1000);
					break;
				}
				break;
			}
			case 9: //duty cycle menu
			{
				if(inputchar == '+')
				{
					DutyCycle += 5;
					if(DutyCycle > 100)
					{
						DutyCycle = 100;
					}
					sprintf(ShowDutyCycle, "Duty Cycle : %d \r\n", DutyCycle);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowDutyCycle, strlen(ShowDutyCycle), 1000);
					break;
				}
				else if(inputchar == '-')
				{
					DutyCycle -= 5;
					if(DutyCycle < 0)
					{
						DutyCycle = 0;
					}
					sprintf(ShowDutyCycle, "Duty Cycle : %d \r\n", DutyCycle);
					HAL_UART_Transmit(&huart2, (uint8_t*)ShowDutyCycle, strlen(ShowDutyCycle), 1000);
					break;
				}
				else if(inputchar == 'x') //back
				{
					state = 4;
					HAL_UART_Transmit(&huart2, (uint8_t*)Word13, strlen(Word13), 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*)Word4, strlen(Word4), 1000);
					break;
				}
				break;
			}
		}
		static uint64_t timestamp = 0;
		static uint64_t timestampGenWave = 0;


		if (micros() - timestamp > 100) //period 100uS -> frequency 10000 Hz # period(uS) don't forget to multi 10^6
		{
			if(WaveForm == 1) //sawtooth wave
			{
				if(Slope == 1) //Slope Up //from the beginning plus middle value that multiply time and divine period
				{
					if(micros() - timestampGenWave <= (1000000/Frequency))
					{
						WaveGenOut = Low + (High - Low)*(micros() - timestampGenWave)*Frequency/1000000;
					}
					else if(micros() - timestampGenWave > (1000000/Frequency))
					{
						timestampGenWave = micros();
					}
				}
				else if(Slope == 2) //Slope Down
				{
					if(micros() - timestampGenWave <= 1000000/Frequency)
					{
						WaveGenOut = High - (High - Low)*(micros() - timestampGenWave)*Frequency/1000000;
					}
					else if(micros() - timestampGenWave > 1000000/Frequency)
					{
						timestampGenWave = micros();
					}
				}
			}
			else if(WaveForm == 2) //sine wave // find theta that depend on time and frequency | give starter value and plus A*sin(theta)
			{
				if(micros() - timestampGenWave <= 1000000/Frequency)
				{
					Theta = -90 + 2*3.14*(micros() - timestampGenWave)*Frequency/1000000;
					WaveGenOut = ((High - Low)/2 + Low) + ((High - Low)/2)*sin(Theta);
				}
				else
				{
					timestampGenWave = micros();
				}
			}
			else if(WaveForm == 3) //square wave // in dutycycle = high | out of duty cycle = low
			{
				if(micros() - timestampGenWave > (1000000/Frequency))
				{
					timestampGenWave = micros();
				}
				else if(micros() - timestampGenWave <= (1000000/Frequency)*DutyCycle/100)
				{
					WaveGenOut = High;
				}
				else if(micros() - timestampGenWave > (1000000/Frequency)*DutyCycle/100)
				{
					WaveGenOut = Low;
				}
			}
			dataOut = ((WaveGenOut/3.3)*4096); //WaveGenOut Value is a Voltage value #devine by 3.3 and multiply 4096 to change to range of IC
			timestamp = micros();
			if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LOAD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micro += 4294967295;
	}
}

int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

inline uint64_t micros()
{
	return htim2.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
