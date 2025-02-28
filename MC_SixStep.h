/*
 * MC_SixStep.h
 *
 *  Created on: Dec 18, 2024
 *      Author: TDM
 */

#include "stdint.h"
#include "stdbool.h"

typedef struct {
  uint8_t State;
  uint16_t CntPh;
  uint16_t PWMperiod;
  int16_t ADCShuntOffset;
  int16_t MeasuredCurrent;
  int16_t MeasuredCurrentFiltr;
  int8_t Direction;
  int8_t PrevDirection;
  int16_t PhaseShift;
  uint8_t HallState;
  uint8_t PrevHallState;
  uint8_t Step;
  uint8_t NextStep;
  bool NextStepFlag;
  bool HallStepFlag;
  int16_t ElAngle;
  int16_t StepElAngle;
  int32_t IncrementElAngle;
  int16_t MeasuredElAngle;
  int32_t SpeedCnt;
  int32_t SpeedCntFiltr;
  int16_t SpeedRPM;
  int16_t CurrentSet;
} MC_SixStep_t;

#define STATE_1 (uint8_t) 1
#define STATE_2 (uint8_t) 2
#define STATE_3 (uint8_t) 3
#define STATE_4 (uint8_t) 4
#define STATE_5 (uint8_t) 5
#define STATE_6 (uint8_t) 6

#define NEGATIVE (int8_t)-1
#define POSITIVE (int8_t) 1

#define IDLE 0
#define START 1
#define RUN 2

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)
#define S16_90_PHASE_SHIFT  (int16_t)(65536/4)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)
#define S16_30_PHASE_SHIFT  (int16_t)(65536/12)

#ifndef INC_MC_SIXSTEP_H_
#define INC_MC_SIXSTEP_H_

void MC_SixStep_SetPhaseVoltage(MC_SixStep_t *pHandle, uint16_t DutyCycle);
void MC_SixStep_LoadNextStep(MC_SixStep_t *pHandle);
bool MC_SixStep_ReadHallState(MC_SixStep_t *pHandle);
void MC_SixStep_HallAngleCalc(MC_SixStep_t *pHandle);
void MC_SixStep_HallAngleToELAngle(MC_SixStep_t *pHandle);
bool MC_SixStep_ElAngleToStep(MC_SixStep_t *pHandle);
uint16_t SetPointCalc(MC_SixStep_t *pHandle, uint16_t ADC_DATA);
int16_t Moving_Average_Simple(int16_t Zk);
void MC_SixStep_UnitPWM(void);
void MC_SixStep_DisablePWM(void);
void MC_SixStep_EnablePWM(void);

#endif /* INC_MC_SIXSTEP_H_ */
