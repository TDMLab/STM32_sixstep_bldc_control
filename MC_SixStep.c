/*
 * MC_SixStep.c
 *
 *  Created on: Dec 18, 2024
 *      Author: TDM
 */
#include "main.h"
#include "MC_SixStep.h"
#include "stdint.h"
#include "stdbool.h"

extern TIM_HandleTypeDef htim1;

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


void MC_SixStep_SetPhaseVoltage(MC_SixStep_t *pHandle, uint16_t DutyCycle) {
	pHandle->CntPh = DutyCycle;
	TIM1->CCR1 = DutyCycle;
	TIM1->CCR2 = DutyCycle;
	TIM1->CCR3 = DutyCycle;
}

void MC_SixStep_LoadNextStep(MC_SixStep_t *pHandle) {
	if (pHandle->NextStepFlag == true) {
		pHandle->NextStepFlag = false;
		switch (pHandle->NextStep) {
		case 1:
			TIM1->CCER = CCER_VH_WH_VL_WL;
			TIM1->CCMR1 = CCMR1_STEP6_HSMOD;
			TIM1->CCMR2 = CCMR2_STEP6_HSMOD;
			break;
		case 2:
			TIM1->CCER = CCER_UH_VH_UL_VL;
			TIM1->CCMR1 = CCMR1_STEP1_HSMOD;
			TIM1->CCMR2 = CCMR2_STEP1_HSMOD;
			break;
		case 3:
			TIM1->CCER = CCER_UH_WH_UL_WL;
			TIM1->CCMR1 = CCMR1_STEP2_HSMOD;
			TIM1->CCMR2 = CCMR2_STEP2_HSMOD;
			break;
		case 4:
			TIM1->CCER = CCER_VH_WH_VL_WL;
			TIM1->CCMR1 = CCMR1_STEP3_HSMOD;
			TIM1->CCMR2 = CCMR2_STEP3_HSMOD;
			break;
		case 5:
			TIM1->CCER = CCER_UH_VH_UL_VL;
			TIM1->CCMR1 = CCMR1_STEP4_HSMOD;
			TIM1->CCMR2 = CCMR2_STEP4_HSMOD;
			break;
		case 6:
			TIM1->CCER = CCER_UH_WH_UL_WL;
			TIM1->CCMR1 = CCMR1_STEP5_HSMOD;
			TIM1->CCMR2 = CCMR2_STEP5_HSMOD;
			break;
		default:
			break;
		}
		TIM1->EGR |= (uint16_t) TIM_EGR_COMG;
	}
}

void MC_SixStep_DisablePWM(void) {
	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void MC_SixStep_EnablePWM(void) {
	__HAL_TIM_MOE_ENABLE(&htim1);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4 | TIM_IT_UPDATE);
}

void MC_SixStep_UnitPWM(void) {
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 1;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	  /*
	  	Bit 0 CCPC: Capture/compare preloaded control
	  	0:CCxE, CCxNE and OCxM bits are not preloaded
	  	1:CCxE, CCxNE and OCxM bits are preloaded, after having been written, they are updated
	  	only when a commutation event (COM) occurs (COMG bit set or rising edge detected on
	  	tim_trgi, depending on the CCUS bit).
	  */
	  	TIM1->CR2 |= (uint16_t) TIM_CR2_CCPC;
	  	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
//	  	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4 | TIM_IT_UPDATE);

}


bool MC_SixStep_ReadHallState(MC_SixStep_t *pHandle) {
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
	return true;
}


void MC_SixStep_HallAngleCalc(MC_SixStep_t *pHandle) {
	switch (pHandle->HallState) {
	case STATE_1: {
		if (pHandle->PrevHallState == STATE_3) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift;
		}
		else if (pHandle->PrevHallState == STATE_5) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift + S16_60_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_5: {
		if (pHandle->PrevHallState == STATE_1) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_4) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift + S16_120_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_4: {
		if (pHandle->PrevHallState == STATE_5) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_120_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_6) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_6: {
		if (pHandle->PrevHallState == STATE_4) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_2) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift - S16_120_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_2: {
		if (pHandle->PrevHallState == STATE_6) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift - S16_120_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_3) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift - S16_60_PHASE_SHIFT);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	case STATE_3: {
		if (pHandle->PrevHallState == STATE_2) {
			pHandle->Direction = POSITIVE;
			pHandle->MeasuredElAngle = pHandle->PhaseShift - S16_60_PHASE_SHIFT;
		}
		else if (pHandle->PrevHallState == STATE_1) {
			pHandle->Direction = NEGATIVE;
			pHandle->MeasuredElAngle = (int16_t) (pHandle->PhaseShift);
		}
		else {
			/* Nothing to do */
		}
		break;
	}
	}
	if (pHandle->HallState != pHandle->PrevHallState ){
		pHandle->HallStepFlag = true;
	}
	else {
		pHandle->HallStepFlag = false;
	}
}

void MC_SixStep_HallAngleToELAngle(MC_SixStep_t *pHandle) {
	if (pHandle->HallStepFlag == true) {
		pHandle->HallStepFlag = false;
		pHandle->ElAngle = pHandle->MeasuredElAngle;
	}
	else if (pHandle->State == RUN) {
		pHandle->IncrementElAngle = pHandle->SpeedCntFiltr * 10500 / 65536;
		pHandle->ElAngle += pHandle->IncrementElAngle;
	}
}

bool MC_SixStep_ElAngleToStep(MC_SixStep_t *pHandle) {
	uint8_t Step = 0;
	if ((pHandle->StepElAngle >= (int16_t) ( S16_30_PHASE_SHIFT))
			&& (pHandle->StepElAngle < (int16_t) (S16_90_PHASE_SHIFT))) {
		Step = 1;
	}
	else if ((pHandle->StepElAngle >= (int16_t) (S16_90_PHASE_SHIFT))
			&& (pHandle->StepElAngle < (int16_t) (S16_120_PHASE_SHIFT + S16_30_PHASE_SHIFT))) {
		Step = 2;
	}
	else if ((pHandle->StepElAngle >= (int16_t) (S16_120_PHASE_SHIFT + S16_30_PHASE_SHIFT))
			|| (pHandle->StepElAngle < (int16_t) (- S16_120_PHASE_SHIFT - S16_30_PHASE_SHIFT))) {
		Step = 3;
	}
	else if ((pHandle->StepElAngle >= (int16_t) (- S16_120_PHASE_SHIFT - S16_30_PHASE_SHIFT))
			&& (pHandle->StepElAngle < (int16_t) (- S16_90_PHASE_SHIFT))) {
		Step = 4;
	}
	else if ((pHandle->StepElAngle >= (int16_t) (- S16_90_PHASE_SHIFT))
			&& (pHandle->StepElAngle < (int16_t) (- S16_30_PHASE_SHIFT))) {
		Step = 5;
	}
	else if ((pHandle->StepElAngle >= (int16_t) (- S16_30_PHASE_SHIFT))
			&& (pHandle->StepElAngle < (int16_t) (S16_30_PHASE_SHIFT))) {
		Step = 6;
	}
	pHandle->NextStep = Step;
	if (pHandle->NextStep != pHandle->Step)
		return true;
	else
		return false;
}

uint16_t SetPointCalc(MC_SixStep_t *pHandle, uint16_t ADC_DATA) {
	#define ADC_OFFSET 1050
	#define ADC_GAIN 10

	int32_t SetPoint;
	SetPoint = (ADC_DATA - 1090) * ADC_GAIN;
	if (SetPoint < 0) {
		SetPoint = 0;
	}
	else if (SetPoint > 32767) {
		SetPoint = 32767;
	}
	return SetPoint;
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
