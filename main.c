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
#include "adc.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MC_SixStep.h"
#include "DRV830X_Driver.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PHASE_SHIFT_USER (int16_t)(0)

#define NUM_POLES_PAIR 10
#define HALL_ROT_STATE NUM_POLES_PAIR * 6
#define PWM_FREQ 16000

#define MAX_CURRENT 2000
#define MAX_CURRENT_NEG -2000
#define VOLTAGE_TO_CURRENT_COEF MAX_CURRENT / 2048
#define TIM1_PWM_PERIOD 5311

#define CURRENT_SET_COEF MAX_CURRENT / 32768
#define SPEED_SET_COEF 1200 / 32768

#define CURRENT_P_DIVIDER 128
#define CURRENT_I_DIVIDER 32
#define CURRENT_P_COEF 3000 / CURRENT_P_DIVIDER
#define CURRENT_I_COEF 3000 / CURRENT_I_DIVIDER

#define SPEED_P_DIVIDER 1024
#define SPEED_I_DIVIDER 8192
#define SPEED_P_COEF 15000 / SPEED_P_DIVIDER
#define SPEED_I_COEF 500 / SPEED_I_DIVIDER

#define PI_TO_PHASE_VOLTAGE_COEF TIM1_PWM_PERIOD / 32768

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t ADC_DATA_1 = 0;
uint16_t ADC_DATA_2 = 0;
int16_t SetCurrent = 0;
int16_t SetSpeed = 0;

volatile uint32_t CountHallTime = 0;
uint16_t ms_count = 0;
uint16_t Accel = 0;

bool CountHallReadyFlag = false;
bool HallSwitchFlag = false;
bool HallTimeCntFlag = false;
uint8_t HallSeq = 0;

int32_t PI_Kp_Out_Current = 0;
int32_t PI_Ip_Out_Current = 0;
int32_t PI_Error_Current = 0;
int32_t PI_Ip_Current_Acc = 0;
int32_t PI_Out_Current = 0;

int32_t PI_Kp_Out_Speed = 0;
int32_t PI_Ip_Out_Speed = 0;
int32_t PI_Error_Speed = 0;
int32_t PI_Ip_Speed_Acc = 0;
int32_t PI_Out_Speed = 0;

uint16_t PhaseVoltageSet = 0;

static volatile int32_t CurrentMeasured;
static volatile int32_t CurrentMeasuredFiltr;

MC_SixStep_t MC_SixStep_M1 = {
		.State = 0,
		.CntPh = 0,
		.PWMperiod = TIM1_PWM_PERIOD,
		.ADCShuntOffset = 0,
		.MeasuredCurrent = 0,
		.MeasuredCurrentFiltr = 0,
		.Direction = 0,
		.PrevDirection = 0,
		.PhaseShift = PHASE_SHIFT_USER,
		.HallState = 0,
		.PrevHallState = 0,
		.Step = 0,
		.NextStep = 0,
		.NextStepFlag = false,
		.HallStepFlag = false,
		.ElAngle = 0,
		.StepElAngle = 0,
		.IncrementElAngle = 0,
		.MeasuredElAngle = 0,
		.SpeedCnt = 0,
		.SpeedCntFiltr = 0,
		.SpeedRPM = 0,
		.CurrentSet = 0,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* **************** Прерывание ШИМ ******************* */
/*******************************************************/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM1) {
/* *********** Чтение значения АЦП канала измерения тока *************** */
    	ADC_DATA_2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    	MC_SixStep_M1.MeasuredCurrent = (ADC_DATA_2 - MC_SixStep_M1.ADCShuntOffset) * VOLTAGE_TO_CURRENT_COEF;
    	MC_SixStep_M1.MeasuredCurrentFiltr = (MC_SixStep_M1.MeasuredCurrent*32767L/100*5 + MC_SixStep_M1.MeasuredCurrentFiltr*32767L/100*95)/32768;
/* *********** Расчет времени смены состояний Холлов*************** */
		if (HallTimeCntFlag == true){
			CountHallTime++;
		}
		if (HallSwitchFlag == true) {
			HallTimeCntFlag = true;
/*********** Чтение состояния с Холлов  ***********/
			MC_SixStep_ReadHallState(&MC_SixStep_M1);
/*********** Определение угла по Холлам ***********/
			MC_SixStep_M1.PrevDirection = MC_SixStep_M1.Direction;
			MC_SixStep_HallAngleCalc(&MC_SixStep_M1);

			if (CountHallTime != 0) {
				if (MC_SixStep_M1.Direction == 1) {
					if (MC_SixStep_M1.PrevDirection == 1 && MC_SixStep_M1.State == START) {
						HallSeq++;
					}
					else {
						HallSeq = 0;
					}
					MC_SixStep_M1.SpeedCnt = 65536 / CountHallTime;
				}
				else if (MC_SixStep_M1.Direction == -1) {
					if (MC_SixStep_M1.PrevDirection == -1 && MC_SixStep_M1.State == START) {
						HallSeq++;
					}
					else {
						HallSeq = 0;
					}
					MC_SixStep_M1.SpeedCnt = -(65536 / CountHallTime);
				}
				CountHallTime = 0;
			}
		}
		else if (CountHallTime > 128){
			CountHallTime = 0;
			HallTimeCntFlag = false;
			MC_SixStep_M1.SpeedCnt = 0;
			MC_SixStep_M1.State = START;
		}

/* **************** Расчет угла ротора **************** */
		MC_SixStep_HallAngleToELAngle(&MC_SixStep_M1);

/*********** Определение уставки направления вращения ***********/
		if (MC_SixStep_M1.State == START) {
			if (SetSpeed >= 0) {
				MC_SixStep_M1.StepElAngle = MC_SixStep_M1.ElAngle + S16_120_PHASE_SHIFT;
			}
			else {
				MC_SixStep_M1.StepElAngle = MC_SixStep_M1.ElAngle - S16_120_PHASE_SHIFT;
			}
		}
		else if (MC_SixStep_M1.State == RUN) {
			if (SetSpeed >= 0) {
				MC_SixStep_M1.StepElAngle = MC_SixStep_M1.ElAngle + S16_90_PHASE_SHIFT;
			}
			else {
				MC_SixStep_M1.StepElAngle = MC_SixStep_M1.ElAngle - S16_90_PHASE_SHIFT;
			}
		}
/* *************** Расчет нового шага ***************** */
		MC_SixStep_M1.NextStepFlag = MC_SixStep_ElAngleToStep(&MC_SixStep_M1);
/*********** Установка нового  шага ***********/
		MC_SixStep_LoadNextStep(&MC_SixStep_M1);
/* ******************* ПИ-регулятор тока ******************* */
		SetCurrent = PI_Out_Speed * CURRENT_SET_COEF;
		PI_Error_Current = SetCurrent - MC_SixStep_M1.MeasuredCurrentFiltr;

		PI_Kp_Out_Current = PI_Error_Current * CURRENT_P_COEF;

		PI_Ip_Current_Acc += PI_Error_Current * CURRENT_I_COEF;
		if (PI_Ip_Current_Acc > 32767 << 15) {
			PI_Ip_Current_Acc = 32767 << 15;
		} else if (PI_Ip_Current_Acc < -32768 << 15) {
			PI_Ip_Current_Acc = -32768 << 15;
		}

		PI_Ip_Out_Current = PI_Ip_Current_Acc >> 15;

		if (PI_Ip_Out_Current > 32767) {
			PI_Ip_Out_Current = 32767;
		} else if (PI_Ip_Out_Current < -32768) {
			PI_Ip_Out_Current = -32768;
		}

		if (Accel == 0) {
			PI_Ip_Current_Acc = 0;
			PI_Kp_Out_Current = 0;
			PI_Ip_Out_Current = 0;
		}

		PI_Out_Current = PI_Kp_Out_Current + PI_Ip_Out_Current;

		if (PI_Out_Current > 32767) {
			PI_Out_Current = 32767;
		} else if (PI_Out_Current < -32767) {
			PI_Out_Current = -32767;
		}

		MC_SixStep_M1.CurrentSet = PI_Out_Current;
/* *************** Расчет заполнения ШИМ ******************* */
		PhaseVoltageSet = abs(MC_SixStep_M1.CurrentSet) * PI_TO_PHASE_VOLTAGE_COEF;

		if (PhaseVoltageSet > TIM1_PWM_PERIOD) {
			PhaseVoltageSet = TIM1_PWM_PERIOD;
		}

		MC_SixStep_SetPhaseVoltage(&MC_SixStep_M1, PhaseVoltageSet);
//		MC_SixStep_SetPhaseVoltage(&MC_SixStep_M1, TIM1_PWM_PERIOD/2);

		HallSwitchFlag = false;
    }
}

/* **************** Прерывание по событию от Холлов ******************* */
/************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		HallSwitchFlag = true;
	}
}

/* **************** Прерывание 1мс ******************* */
/* *************************************************** */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
// ************	 Получение данных с ручек управления ************
		ADC_DATA_1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);
		Accel = SetPointCalc(&MC_SixStep_M1, ADC_DATA_1);
		SetSpeed = Accel * SPEED_SET_COEF;
		if (SetSpeed != 0) {
			if (MC_SixStep_M1.State == IDLE) {
				MC_SixStep_M1.State = START;
				HallSeq = 0;
				PI_Ip_Speed_Acc = 0;
				PI_Ip_Current_Acc = 0;
				MC_SixStep_EnablePWM();
			}
			else if (MC_SixStep_M1.State == START && MC_SixStep_M1.SpeedCnt > 0 && HallSeq > 10) {
				MC_SixStep_M1.State = RUN;
			}
// ************	 ПИ-регулятор скорости ************
			PI_Error_Speed = SetSpeed - MC_SixStep_M1.SpeedRPM;
			PI_Kp_Out_Speed = PI_Error_Speed * SPEED_P_COEF;

			PI_Ip_Speed_Acc += PI_Error_Speed;
			if (PI_Ip_Speed_Acc > 32767 << 15) {
				PI_Ip_Speed_Acc = 32767 << 15;
			} else if (PI_Ip_Speed_Acc < -32768 << 15) {
				PI_Ip_Speed_Acc = -32768 << 15;
			}

			PI_Ip_Out_Speed = PI_Ip_Speed_Acc * SPEED_I_COEF;

			if (PI_Ip_Out_Speed > 32767) {
				PI_Ip_Out_Speed = 32767;
			} else if (PI_Ip_Out_Speed < -32767) {
				PI_Ip_Out_Speed = -32767;
			}
			PI_Out_Speed = PI_Kp_Out_Speed + PI_Ip_Out_Speed;

			if (PI_Out_Speed > 32767) {
				PI_Out_Speed = 32767;
			} else if (PI_Out_Speed < -32767) {
				PI_Out_Speed = -32767;
			}
		}
// ************************************************
		else {
			if (MC_SixStep_M1.State == START || MC_SixStep_M1.State == RUN) {
				MC_SixStep_DisablePWM();
				MC_SixStep_M1.State = IDLE;
				PI_Ip_Speed_Acc = 0;
				PI_Ip_Current_Acc = 0;
				SetCurrent = 0;
				SetSpeed = 0;
				PI_Ip_Speed_Acc = 0;

				PI_Kp_Out_Speed = 0;
				PI_Ip_Out_Speed = 0;
				PI_Out_Speed = 0;
				PI_Error_Speed = 0;

				PI_Kp_Out_Current = 0;
				PI_Ip_Out_Current = 0;
				PI_Out_Current = 0;
				PI_Error_Current = 0;

				MC_SixStep_M1.SpeedCnt = 0;
				MC_SixStep_M1.SpeedCntFiltr = 0;
			}
		}
// ******** Фильтр скорости, пересчет в RPM ********
		MC_SixStep_M1.SpeedCntFiltr = Moving_Average_Simple(MC_SixStep_M1.SpeedCnt);
		MC_SixStep_M1.SpeedRPM = MC_SixStep_M1.SpeedCntFiltr * 65536 / 1000 * HALL_ROT_STATE / PWM_FREQ;
// ************************************************
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_SPI3_Init();
  MX_ADC2_Init();
  MX_OPAMP2_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

  DRV8303_Unit();

  HAL_OPAMP_Start(&hopamp2);

/******* ADC offset calc *******/
//  HAL_ADCEx_InjectedStart(&hadc2);
//  HAL_ADC_PollForConversion(&hadc2, 10);
//  ADC_DATA_2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//  HAL_ADCEx_InjectedStop(&hadc2);
  MC_SixStep_M1.ADCShuntOffset = 2005;
/* *************************** */

  HAL_ADCEx_InjectedStart_IT(&hadc2);

  HAL_ADC_Start(&hadc1);

  MC_SixStep_DisablePWM();
  MC_SixStep_UnitPWM();

  HAL_TIMEx_HallSensor_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
