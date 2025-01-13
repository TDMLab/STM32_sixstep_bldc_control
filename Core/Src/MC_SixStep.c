/*
 * MC_SixStep.c
 *
 *  Created on: Dec 18, 2024
 *      Author: TDM
 */
#include "MC_SixStep.h"
#include "stdint.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

#define STATE_1 (uint8_t) 1
#define STATE_2 (uint8_t) 2
#define STATE_3 (uint8_t) 3
#define STATE_4 (uint8_t) 4
#define STATE_5 (uint8_t) 5
#define STATE_6 (uint8_t) 6

#define NEGATIVE (int8_t)-1
#define POSITIVE (int8_t) 1

#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)
#define S16_90_PHASE_SHIFT  (int16_t)(65536/4)
#define S16_120_PHASE_SHIFT (int16_t)(65536/3)

#define CCMR1_STEP1_HSMOD         	(uint16_t)(0x4868)
#define CCMR1_STEP2_HSMOD        	(uint16_t)(0x0868)
#define CCMR1_STEP3_HSMOD  	        (uint16_t)(0x6808)
#define CCMR1_STEP4_HSMOD  	        (uint16_t)(0x6848)
#define CCMR1_STEP5_HSMOD  	        (uint16_t)(0x0848)
#define CCMR1_STEP6_HSMOD  	        (uint16_t)(0x4808)

#define CCMR2_STEP1_HSMOD         	(uint16_t)(0x6808)
#define CCMR2_STEP2_HSMOD        	(uint16_t)(0x6848)
#define CCMR2_STEP3_HSMOD  	        (uint16_t)(0x6848)
#define CCMR2_STEP4_HSMOD  	        (uint16_t)(0x6808)
#define CCMR2_STEP5_HSMOD  	        (uint16_t)(0x6868)
#define CCMR2_STEP6_HSMOD  	        (uint16_t)(0x6868)

#define CCMR1_CW_STEP1_MIDALIGN    	(uint16_t)(0x4868)
#define CCMR1_CW_STEP2_MIDALIGN   	(uint16_t)(0x4868)
#define CCMR1_CW_STEP3_MIDALIGN   	(uint16_t)(0x6868)
#define CCMR1_CW_STEP4_MIDALIGN   	(uint16_t)(0x6848)
#define CCMR1_CW_STEP5_MIDALIGN  	(uint16_t)(0x6848)
#define CCMR1_CW_STEP6_MIDALIGN  	(uint16_t)(0x4848)

#define CCMR2_CW_STEP1_MIDALIGN    	(uint16_t)(0x6868)
#define CCMR2_CW_STEP2_MIDALIGN   	(uint16_t)(0x6848)
#define CCMR2_CW_STEP3_MIDALIGN   	(uint16_t)(0x6848)
#define CCMR2_CW_STEP4_MIDALIGN   	(uint16_t)(0x6848)
#define CCMR2_CW_STEP5_MIDALIGN  	(uint16_t)(0x6868)
#define CCMR2_CW_STEP6_MIDALIGN  	(uint16_t)(0x6868)

#define CCER_POLARITY_MIDSTEP       UH_POLARITY | UL_POLARITY | VH_POLARITY | VL_POLARITY | WH_POLARITY | WL_POLARITY

#define CCER_UH_VH_UL_VL            (uint16_t)(0x1055)
#define CCER_UH_WH_UL_WL            (uint16_t)(0x1505)
#define CCER_VH_WH_VL_WL            (uint16_t)(0x1550)
#define CCER_UH_VH_WH_UL_VL_WL      (uint16_t)(0x1555)


void MC_SixStep_SetPhaseVoltage(MC_SixStep_t *pHandle, uint16_t DutyCycle)
{
  pHandle->CntPh = DutyCycle;
  TIM1->CCR1 = DutyCycle;
  TIM1->CCR2 = DutyCycle;
  TIM1->CCR3 = DutyCycle;
}

void MC_SixStep_SetNextStep(MC_SixStep_t *pHandle)
{
  if(pHandle->NextStep != pHandle->Step){
  switch (pHandle->NextStep)
  {
  case 6:
    // UH, VL
	TIM1->CCER = CCER_UH_VH_UL_VL;
	TIM1->CCMR1 = CCMR1_STEP1_HSMOD;
	TIM1->CCMR2 = CCMR2_STEP1_HSMOD;
    break;
  case 1:
    // UH, WL
	TIM1->CCER = CCER_UH_WH_UL_WL;
	TIM1->CCMR1 = CCMR1_STEP2_HSMOD;
	TIM1->CCMR2 = CCMR2_STEP2_HSMOD;
    break;
  case 2:
    // VH, WL
	TIM1->CCER = CCER_VH_WH_VL_WL;
	TIM1->CCMR1 = CCMR1_STEP3_HSMOD;
	TIM1->CCMR2 = CCMR2_STEP3_HSMOD;
    break;
  case 3:
    // VH, UL
	TIM1->CCER = CCER_UH_VH_UL_VL;
	TIM1->CCMR1 = CCMR1_STEP4_HSMOD;
	TIM1->CCMR2 = CCMR2_STEP4_HSMOD;
    break;
  case 4:
    // WH, UL
	TIM1->CCER = CCER_UH_WH_UL_WL;
	TIM1->CCMR1 = CCMR1_STEP5_HSMOD;
	TIM1->CCMR2 = CCMR2_STEP5_HSMOD;
    break;
  case 5:
    // WH, VL
	TIM1->CCER = CCER_VH_WH_VL_WL;
	TIM1->CCMR1 = CCMR1_STEP6_HSMOD;
	TIM1->CCMR2 = CCMR2_STEP6_HSMOD;
    break;
  default:
    break;
  }
  TIM1->EGR |= (uint16_t) TIM_EGR_COMG;
  pHandle->Step = pHandle->NextStep;
  }
}

void MC_SixStep_AngleCalc(MC_SixStep_t *pHandle) {
	pHandle->PrevHallState = pHandle->HallState;

	/* HALL Sensor 120 degrees */
	pHandle->HallState = (((GPIOB->IDR & GPIO_IDR_IDR_8) >> 8) << 2)
			| (((GPIOB->IDR & GPIO_IDR_IDR_7) >> 7) << 1)
			| ((GPIOB->IDR & GPIO_IDR_IDR_6) >> 6);
	/* HALL Sensor 60 degrees */
	/*
	pHandle->HallState = ((((GPIOB->IDR & GPIO_IDR_IDR_7) ^ 1U) >> 7) << 2)
			| (((GPIOB->IDR & GPIO_IDR_IDR_8) >> 8) << 1)
			| ((GPIOB->IDR & GPIO_IDR_IDR_6) >> 6);
	 */

	switch (pHandle->HallState) {
	case STATE_1: {
		if (pHandle->PrevHallState == STATE_3) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift;
		}
		else if (pHandle->PrevHallState == STATE_5) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift
					+ S16_60_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_5: {
		if (pHandle->PrevHallState == STATE_1) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift
					+ S16_60_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_4) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift
					+ S16_120_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_4: {
		if (pHandle->PrevHallState == STATE_5) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift
					+ S16_120_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_6) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift
					+ S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_6: {
		if (pHandle->PrevHallState == STATE_4) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift
					+ S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_2) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift
					- S16_120_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_2: {
		if (pHandle->PrevHallState == STATE_6) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift
					- S16_120_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_3) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift
					- S16_60_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_3: {
		if (pHandle->PrevHallState == STATE_2) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift
					- S16_60_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_1) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle =
					(int16_t) (pHandle->PhaseShift);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	}
}

bool MC_SixStep_ElAngleToStep(MC_SixStep_t *pHandle) {
	uint8_t Step;
	if ((pHandle->MeasuredElAngle >= (int16_t) ( S16_60_PHASE_SHIFT / 2))
			&& (pHandle->MeasuredElAngle
					< (int16_t) ( S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2)))
		Step = 1;
	else if ((pHandle->MeasuredElAngle
			>= (int16_t) ( S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2))
			&& (pHandle->MeasuredElAngle
					< (int16_t) ( S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2)))
		Step = 2;
	else if ((pHandle->MeasuredElAngle
			>= (int16_t) ( S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2))
			|| (pHandle->MeasuredElAngle
					< (int16_t) (- S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2)))
		Step = 3;
	else if ((pHandle->MeasuredElAngle
			>= (int16_t) (- S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2))
			&& (pHandle->MeasuredElAngle
					< (int16_t) (- S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2)))
		Step = 4;
	else if ((pHandle->MeasuredElAngle
			>= (int16_t) (- S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2))
			&& (pHandle->MeasuredElAngle < (int16_t) (- S16_60_PHASE_SHIFT / 2)))
		Step = 5;
	else if ((pHandle->MeasuredElAngle >= (int16_t) (- S16_60_PHASE_SHIFT / 2))
			&& (pHandle->MeasuredElAngle < (int16_t) ( S16_60_PHASE_SHIFT / 2)))
		Step = 6;
	else {
	}
	pHandle->NextStep = Step;
	if (pHandle->NextStep != pHandle->Step)
		return true;
	else
		return false;
}


uint16_t SetPointCalc(MC_SixStep_t *pHandle, uint16_t ADC_DATA) {
	int32_t SetPoint;
	SetPoint = (ADC_DATA - 1050) * 16;
	if (SetPoint < 0) {
		SetPoint = 0;
	}
	else if (SetPoint > 32767) {
		SetPoint = 32767;
	}
	return SetPoint;
}

void DRV8303_Unit(void) {
// ***************** Сброс и включение DRV8303 ***********************
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

// ******************* �?нициализация DRV8303 *************************
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	uint16_t SPI_DATA_SEND[1];
	uint16_t SPI_DATA_READ[1];

	SPI_DATA_SEND[0] = 0b0001000000110000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b1001000000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b0000000000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi3, (uint8_t*) SPI_DATA_READ, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b0001100000110000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b1001100000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b0000000000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi3, (uint8_t*) SPI_DATA_READ, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}


int16_t Moving_Average_Simple(int16_t Zk) {
  #define WindowLength 8
  static int16_t delayLine[WindowLength];
  static uint16_t pointer;
  static int32_t average_Out;
  if (++pointer >= WindowLength) pointer = 0;
  average_Out -= delayLine[pointer];
  average_Out += Zk;
  delayLine[pointer] = Zk;
  return (average_Out / WindowLength);
}
