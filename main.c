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

#define MAX_CURRENT 7000
#define MAX_CURRENT_NEG -7000
#define VOLTAGE_TO_CURRENT_COEF MAX_CURRENT / 2048
#define TIM1_PWM_PERIOD 5311

#define CURRENT_SET_COEF MAX_CURRENT / 32768
#define SPEED_SET_COEF 1200 / 32768

#define CURRENT_P_DIVIDER 512
#define CURRENT_I_DIVIDER 512
#define CURRENT_P_COEF 1500 / CURRENT_P_DIVIDER
#define CURRENT_I_COEF 3000 / CURRENT_I_DIVIDER

#define SPEED_P_DIVIDER 512
#define SPEED_I_DIVIDER 4096
#define SPEED_P_COEF 22000 / SPEED_P_DIVIDER
#define SPEED_I_COEF 150 / SPEED_I_DIVIDER

#define PI_TO_PHASE_VOLTAGE_COEF TIM1_PWM_PERIOD / 32768

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

OPAMP_HandleTypeDef hopamp2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

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
		.CntPh = 0,
		.PWMperiod = TIM1_PWM_PERIOD,
		.ADCShuntOffset = 0,
		.MeasuredCurrent = 0,
		.MeasuredCurrentFiltr = 0,
		.Direction = 0,
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
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* **************** Прерывание ШИМ ******************* */
/*******************************************************/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM1) {
		CountHallTime++;
/* ******************* ПИ-регулятор тока ******************* */
		SetCurrent = PI_Out_Speed * CURRENT_SET_COEF;
		PI_Error_Current = SetCurrent - MC_SixStep_M1.MeasuredCurrentFiltr;

		PI_Kp_Out_Current = PI_Error_Current * CURRENT_P_COEF;

		PI_Ip_Current_Acc += PI_Error_Current * CURRENT_I_COEF;
		if (PI_Ip_Current_Acc > 32767 << 15) {
			PI_Ip_Current_Acc = 32767 << 15;
		}
		else if (PI_Ip_Current_Acc < -32768 << 15) {
			PI_Ip_Current_Acc = -32768 << 15;
		}

		PI_Ip_Out_Current = PI_Ip_Current_Acc >> 15;

		if (PI_Ip_Out_Current > 32767) {
			PI_Ip_Out_Current = 32767;
		}
		else if (PI_Ip_Out_Current < -32768) {
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
		}
		else if (PI_Out_Current < -32767) {
			PI_Out_Current = -32767;
		}

		MC_SixStep_M1.CurrentSet = PI_Out_Current;

/* *************** Расчет заполнения ШИМ ******************* */
		PhaseVoltageSet = abs(MC_SixStep_M1.CurrentSet) * PI_TO_PHASE_VOLTAGE_COEF;

		if (PhaseVoltageSet > TIM1_PWM_PERIOD) {
			PhaseVoltageSet = TIM1_PWM_PERIOD;
		}

		MC_SixStep_SetPhaseVoltage(&MC_SixStep_M1, PhaseVoltageSet);
/* **************** Расчет угла ротора **************** */
		MC_SixStep_HallAngleToELAngle(&MC_SixStep_M1);
/* *************** Расчет нового шага ***************** */
		MC_SixStep_M1.NextStepFlag = MC_SixStep_ElAngleToStep(&MC_SixStep_M1);
/*********** Определение направления вращения ***********/
		if (SetSpeed >= 0) {
			MC_SixStep_M1.StepElAngle = MC_SixStep_M1.ElAngle + S16_90_PHASE_SHIFT;
		}
		else {
			MC_SixStep_M1.StepElAngle = MC_SixStep_M1.ElAngle - S16_90_PHASE_SHIFT;
		}
/*********** Установка нового шага ***********/
		MC_SixStep_LoadNextStep(&MC_SixStep_M1);
    }
}

/* **************** Прерывание по готовности АЦП ******************* */
/*********************************************************************/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
//	GPIOC->BSRR |= 1<<4; // Set
//	GPIOC->BRR |= 1<<4; //Reset
    if(hadc->Instance == ADC2) {
    	ADC_DATA_2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    	CurrentMeasured = (ADC_DATA_2 - MC_SixStep_M1.ADCShuntOffset) * VOLTAGE_TO_CURRENT_COEF;
		CurrentMeasuredFiltr = (CurrentMeasured*32767L/100*5 + CurrentMeasuredFiltr*32767L/100*95)/32768;

		MC_SixStep_M1.MeasuredCurrent = CurrentMeasured;
		MC_SixStep_M1.MeasuredCurrentFiltr = CurrentMeasuredFiltr;
    }
}

/* **************** Прерывание 1мс ******************* */
/* *************************************************** */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM7) {

		ADC_DATA_1 = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);
		Accel = SetPointCalc(&MC_SixStep_M1, ADC_DATA_1);
		SetSpeed = Accel * SPEED_SET_COEF;
// ************	 ПИ-регулятор скорости ************
		PI_Error_Speed = SetSpeed - MC_SixStep_M1.SpeedRPM;
		PI_Kp_Out_Speed = PI_Error_Speed * SPEED_P_COEF;

		PI_Ip_Speed_Acc += PI_Error_Speed;
		if (PI_Ip_Speed_Acc > 32767 << 15) {
			PI_Ip_Speed_Acc = 32767 << 15;
		}
		else if (PI_Ip_Speed_Acc < -32768 << 15) {
			PI_Ip_Speed_Acc = -32768 << 15;
		}

		PI_Ip_Out_Speed = PI_Ip_Speed_Acc * SPEED_I_COEF;

		if (PI_Ip_Out_Speed > 32767) {
			PI_Ip_Out_Speed = 32767;
		}
		else if (PI_Ip_Out_Speed < -32767) {
			PI_Ip_Out_Speed = -32767;
		}

		if (Accel == 0) {
			PI_Ip_Speed_Acc = 0;
			PI_Kp_Out_Speed = 0;
			PI_Ip_Out_Speed = 0;
		}
		PI_Out_Speed = PI_Kp_Out_Speed + PI_Ip_Out_Speed;

		if (PI_Out_Speed > 32767) {
			PI_Out_Speed = 32767;
		}
		else if (PI_Out_Speed < -32767) {
			PI_Out_Speed = -32767;
		}
// ************************************************
// ******** Фиьтр скорости, пересчет в RPM ********
		if (HallSwitchFlag == true) {
			HallSwitchFlag = false;
			MC_SixStep_M1.SpeedCntFiltr = Moving_Average_Simple(MC_SixStep_M1.SpeedCnt);
			MC_SixStep_M1.SpeedRPM = MC_SixStep_M1.SpeedCntFiltr * 65536 / 1000 * HALL_ROT_STATE / PWM_FREQ;
		}
		else if (HallSwitchFlag == false && CountHallTime > 2048) {
			MC_SixStep_M1.SpeedCnt = 0;
			MC_SixStep_M1.SpeedCntFiltr = Moving_Average_Simple(MC_SixStep_M1.SpeedCnt);
			MC_SixStep_M1.SpeedRPM = MC_SixStep_M1.SpeedCntFiltr * 65536 / 1000 * HALL_ROT_STATE / PWM_FREQ;
		}
// ************************************************
    }
}

/* **************** Прерывание по событию от Холлов ******************* */
/************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
/*********** Определение угла по Холлам ***********/
		MC_SixStep_HallAngleCalc(&MC_SixStep_M1);
/* *********** Расчет времени смены состояний Холлов*************** */
		if (MC_SixStep_M1.Direction == 1) {
			MC_SixStep_M1.SpeedCnt = 65536 / CountHallTime;
		}
		else if (MC_SixStep_M1.Direction == -1) {
			MC_SixStep_M1.SpeedCnt = -(65536 / CountHallTime);
		}
		CountHallTime = 0;
		HallSwitchFlag = true;
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

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 1;

  MC_SixStep_M1.HallState = (((GPIOB->IDR & GPIO_IDR_IDR_6) >> 6) << 2) | (((GPIOB->IDR & GPIO_IDR_IDR_7) >> 7) << 1) | ((GPIOB->IDR & GPIO_IDR_IDR_8) >> 8);
  MC_SixStep_HallAngleCalc(&MC_SixStep_M1);
  MC_SixStep_ElAngleToStep(&MC_SixStep_M1);
  MC_SixStep_LoadNextStep(&MC_SixStep_M1);

  HAL_OPAMP_Start(&hopamp2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

/******* ADC offset calc *******/
//  HAL_ADCEx_InjectedStart(&hadc2);
//  HAL_ADC_PollForConversion(&hadc2, 10);
//  ADC_DATA_2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//  HAL_ADCEx_InjectedStop(&hadc2);
  MC_SixStep_M1.ADCShuntOffset = 2005;
/* *************************** */

  MC_SixStep_M1.ElAngle = MC_SixStep_M1.MeasuredElAngle;
  MC_SixStep_M1.NextStepFlag = MC_SixStep_ElAngleToStep(&MC_SixStep_M1);

  HAL_TIMEx_HallSensor_Start_IT(&htim4);

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);

  /*
  	Bit 0 CCPC: Capture/compare preloaded control
  	0:CCxE, CCxNE and OCxM bits are not preloaded
  	1:CCxE, CCxNE and OCxM bits are preloaded, after having been written, they are updated
  	only when a commutation event (COM) occurs (COMG bit set or rising edge detected on
  	tim_trgi, depending on the CCUS bit).
  */
  	TIM1->CR2 |= (uint16_t) TIM_CR2_CCPC;

  	HAL_ADCEx_InjectedStart_IT(&hadc2);

  	HAL_TIM_Base_Start_IT(&htim7);

  	HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

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
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 5311;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 15;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_EN_Pin DRV_EN_Pin */
  GPIO_InitStruct.Pin = PWR_EN_Pin|DRV_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_NSS_GPIO_Port, &GPIO_InitStruct);

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
