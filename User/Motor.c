#include "Motor.h"

void Motor_Control()
{
		All_MotorPID_TorqueChange();
	
		CAN1_0x200_Tx_Data[0] = LFMotor_Torque>>8;
		CAN1_0x200_Tx_Data[1] = LFMotor_Torque;
	
		CAN1_0x200_Tx_Data[2] = LBMotor_Torque>>8;
		CAN1_0x200_Tx_Data[3] = LBMotor_Torque;
	
		CAN1_0x200_Tx_Data[4] = RFMotor_Torque>>8;
		CAN1_0x200_Tx_Data[5] = RFMotor_Torque;
	
		CAN1_0x200_Tx_Data[6] = RBMotor_Torque>>8;
		CAN1_0x200_Tx_Data[7] = RBMotor_Torque;
	
		CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
	
}

void Set_MotorTarget(int LFTarget,int LBTarget,int RFTarget,int RBTarget)
{
	
	LFMotor_Target = LFTarget;
	LBMotor_Target = LBTarget;
	RFMotor_Target = RFTarget;
	RBMotor_Target = RBTarget;
	Set_Target(LFMotor_Target,&PID_canshu_LFMotor);
	Set_Target(LBMotor_Target,&PID_canshu_LBMotor);
	Set_Target(RFMotor_Target,&PID_canshu_RFMotor);
	Set_Target(RBMotor_Target,&PID_canshu_RBMotor);
}
void All_MotorPID_Init()
{
	float kp=0.265,ki=2.5, kd=0.015,kf=0.1;
	PID_INIT(kp,ki,kd,kf,1000,1200,12000,5000,10000,10000,0,0.002,&PID_canshu_LFMotor);
	PID_INIT(kp,ki,kd,kf,1000,1200,12000,5000,10000,10000,0,0.002,&PID_canshu_LBMotor);
	PID_INIT(kp,ki,kd,kf,1000,1200,12000,5000,10000,10000,0,0.002,&PID_canshu_RFMotor);
	PID_INIT(kp,ki,kd,kf,1000,1200,12000,5000,10000,10000,0,0.002,&PID_canshu_RBMotor);
	
}
void All_MotorFilter_Init()
{
	CAN_Filter_Mask_Config(&hcan1,CAN_FILTER(13)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE,0x201,0x7ff);
	CAN_Filter_Mask_Config(&hcan1,CAN_FILTER(12)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE,0x202,0x7ff);
	CAN_Filter_Mask_Config(&hcan1,CAN_FILTER(11)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE,0x203,0x7ff);
	CAN_Filter_Mask_Config(&hcan1,CAN_FILTER(10)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE,0x204,0x7ff);
	
}
void All_SetMotorNowSpeed()
{
	Set_Now(Lf_Motor_Data.Omeag,&PID_canshu_LFMotor);
	Set_Now(Lb_Motor_Data.Omeag,&PID_canshu_LBMotor);
	Set_Now(Rf_Motor_Data.Omeag,&PID_canshu_RFMotor);
	Set_Now(Rb_Motor_Data.Omeag,&PID_canshu_RBMotor);
	
}
void All_MotorPID_TorqueChange()
{
	float lfmotorchange=0.0f;
	float lbmotorchange=0.0f;
	float rfmotorchange=0.0f;
	float rbmotorchange=0.0f;
	
	PID_Computing(&PID_canshu_LFMotor);
	PID_Computing(&PID_canshu_LBMotor);
	PID_Computing(&PID_canshu_RFMotor);
	PID_Computing(&PID_canshu_RBMotor);
	
	lfmotorchange=Get_Out(PID_canshu_LFMotor);
	lbmotorchange=Get_Out(PID_canshu_LBMotor);
	rfmotorchange=Get_Out(PID_canshu_RFMotor);
	rbmotorchange=Get_Out(PID_canshu_RBMotor);
	
	LFMotor_Torque+=lfmotorchange;
	LBMotor_Torque+=lbmotorchange;
	RFMotor_Torque+=rfmotorchange;
	RBMotor_Torque+=rbmotorchange;
	All_TorqueLimit();
	
}

void Torque_Limit(PID_CANSHU p,int x)
{
	if(p.Target>0)
	{
		switch(x)
		{
			case 1:
				if(LFMotor_Torque>2500)
					LFMotor_Torque=2500;
//				else if(LFMotor_Torque<0)
//					LFMotor_Torque=0;
				break;
				
				case 2:
				if(LBMotor_Torque>2500)
					LBMotor_Torque=2500;
//				else if(LBMotor_Torque<0)
//					LBMotor_Torque=0;
				break;
				
				case 3:
				if(RFMotor_Torque>2500)
					RFMotor_Torque=2500;
//				else if(RFMotor_Torque<0)
//					RFMotor_Torque=0;
				break;
				
				case 4:
				if(RBMotor_Torque>2500)
					RBMotor_Torque=2500;
//				else if(RBMotor_Torque<0)
//					RBMotor_Torque=0;
				break;
				
		}
		
	}
	else
	{
		switch(x)
		{
			case 1:
				if(LFMotor_Torque<-2500)
					LFMotor_Torque=-2500;
//				else if(LFMotor_Torque>0)
//					LFMotor_Torque=0;
				break;
				
				case 2:
				if(LBMotor_Torque<-2500)
					LBMotor_Torque=-2500;
//				else if(LBMotor_Torque>0)
//					LBMotor_Torque=0;
				break;
				
				case 3:
				if(RFMotor_Torque<-2500)
					RFMotor_Torque=-2500;
//				else if(RFMotor_Torque>0)
//					RFMotor_Torque=0;
				break;
				
				case 4:
				if(RBMotor_Torque<-2500)
					RBMotor_Torque=-2500;
//				else if(RBMotor_Torque>0)
//					RBMotor_Torque=0;
				break;
				
		}
		
	}
	

	
}
void All_TorqueLimit()
{
	Torque_Limit(PID_canshu_LFMotor,1);
	Torque_Limit(PID_canshu_LBMotor,2);
	Torque_Limit(PID_canshu_RFMotor,3);
	Torque_Limit(PID_canshu_RBMotor,4);
}

void getValues(float Vx ,float Vy, float Vz)
{
	float circleLenth = 2*pi*Lenth;
	RFMotor_Target = (float)((-Vx*cosa-Vy*cosa+Vz*Lenth)/circleLenth*reduction_ratio*60.0);
	LFMotor_Target = (float)((+Vx*cosa-Vy*cosa+Vz*Lenth)/circleLenth*reduction_ratio*60.0);
	LBMotor_Target = (float)((+Vx*cosa+Vy*cosa+Vz*Lenth)/circleLenth*reduction_ratio*60.0);
	RBMotor_Target = (float)((-Vx*cosa+Vy*cosa+Vz*Lenth)/circleLenth*reduction_ratio*60.0);
	Set_MotorTarget(LFMotor_Target,LBMotor_Target,RFMotor_Target,RBMotor_Target);
}
