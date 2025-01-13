/*
 * MC_SixStep.h
 *
 *  Created on: Dec 18, 2024
 *      Author: TDM
 */

#include "stdint.h"
#include "stdbool.h"

typedef struct {
  uint16_t CntPh;
  uint16_t StartCntPh;
  uint16_t ADCTriggerCnt;
  uint16_t PWMperiod;
  int16_t ADCShuntOffset;
  int16_t MeasuredCurrent;
  int16_t MeasuredCurrentFiltr;
  int8_t Direction;
  int16_t MeasuredElAngle;
  int16_t PhaseShift;
  uint8_t HallState;
  uint8_t PrevHallState;
  int16_t AlignFlag;
  uint8_t NextStep;
  uint8_t Step;
  int16_t hElAngle;
  int32_t SpeedCnt;
  int32_t SpeedCntFiltr;
} MC_SixStep_t;

#ifndef INC_MC_SIXSTEP_H_
#define INC_MC_SIXSTEP_H_

void MC_SixStep_SetPhaseVoltage(MC_SixStep_t *pHandle, uint16_t DutyCycle);
void MC_SixStep_SetNextStep(MC_SixStep_t *pHandle);
void MC_SixStep_AngleCalc(MC_SixStep_t *pHandle);
bool MC_SixStep_ElAngleToStep(MC_SixStep_t *pHandle);
uint16_t SetPointCalc(MC_SixStep_t *pHandle, uint16_t ADC_DATA);
void DRV8303_Unit(void);
int16_t Moving_Average_Simple(int16_t Zk);

#endif /* INC_MC_SIXSTEP_H_ */
