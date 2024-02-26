#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "drv_can.h"
#include "PID.h"
void Motor_Control(void);
void Set_MotorTarget(int LFTarget,int LBTarget,int RFTarget,int RBTarget);
void All_MotorPID_Init(void);
void All_MotorFilter_Init(void);
void All_SetMotorNowSpeed(void);
void All_MotorPID_TorqueChange(void);
void Torque_Limit(PID_CANSHU p,int x);
void All_TorqueLimit(void);
void getValues(float Vx ,float Vy, float Vz);
#endif
