#include "mathPID.h"
#include "main.h"
#include "PID.h"
//float D_T;
//float Dead_Zone;
//float Pre_Now = 0.0f;
//float Pre_Target = 0.0f;
//float Pre_Out = 0.0f;
//float Pre_Error = 0.0f;
//float Out = 0.0f;
//float K_P = 0.0f;
//float K_I = 0.0f;
//float K_D = 0.0f;
//float K_F = 0.0f;
//float I_Out_Max = 0;
//float Out_Max = 0;
//float I_Variable_Speed_A = 0.0f;
//float I_Variable_Speed_B = 0.0f;
//float I_Separate_Threshold = 0.0f;
//float Target = 0.0f;
//float Now = 0.0f;
//float Integral_Error = 0.0f;
//int D_First;
PID_CANSHU PID_canshu_LFMotor;
PID_CANSHU PID_canshu_LBMotor;
PID_CANSHU PID_canshu_RFMotor;
PID_CANSHU PID_canshu_RBMotor;

void PID_INIT(float __K_P, float __K_I, float __K_D,float __K_F,
							float __I_Out_Max , float __Out_Max,
							float __Dead_Zone, 
							float __I_Variable_Speed_A , 
							float __I_Variable_Speed_B, 
							float __I_Separate_Threshold,
							int PID_D_First,
							float __D_T,
							PID_CANSHU* PIDcanshu)
{
	
//		K_P = __K_P;
//		K_I = __K_I;
//    K_D = __K_D;
//    K_F = __K_F;
//		D_T = __D_T;
//    I_Out_Max = __I_Out_Max;
//    Out_Max = __Out_Max;
//    Dead_Zone = __Dead_Zone;
//    I_Variable_Speed_A = __I_Variable_Speed_A;
//    I_Variable_Speed_B = __I_Variable_Speed_B;
//    I_Separate_Threshold = __I_Separate_Threshold;
//    D_First = PID_D_First;
		PIDcanshu->K_P = __K_P;
		PIDcanshu->K_I = __K_I;
		PIDcanshu->K_D = __K_D;
		PIDcanshu->K_F = __K_F;
		PIDcanshu->D_T = __D_T;
		PIDcanshu->Out_Max = __I_Out_Max;
		PIDcanshu->Out_Max = __Out_Max;
		PIDcanshu->Dead_Zone = __Dead_Zone;
		PIDcanshu->I_Variable_Speed_A = __I_Variable_Speed_A;
		PIDcanshu->I_Variable_Speed_B = __I_Variable_Speed_B;
		PIDcanshu->I_Separate_Threshold = __I_Separate_Threshold;
		PIDcanshu->D_First = PID_D_First;
	
	
}
float Get_Integral_Error(PID_CANSHU p)
{
    return (p.Integral_Error);
}
float Get_Out(PID_CANSHU p)
{
    return (p.Out);
}
void Set_K_P(float __K_P,PID_CANSHU* p)
{
//    K_P = __K_P;
	p->K_P = __K_P;
}

void Set_K_I(float __K_I,PID_CANSHU* p)
{
//    K_I = __K_I;
	p->K_I = __K_I;
}

void Set_K_D(float __K_D,PID_CANSHU* p)
{
//    K_D = __K_D;
	p->K_D = __K_D;
}

void Set_K_F(float __K_F,PID_CANSHU* p)
{
//    K_F = __K_F;
	p->K_F = __K_F;
}

void Set_I_Out_Max(float __I_Out_Max,PID_CANSHU* p)
{
//    I_Out_Max = __I_Out_Max;
	p->I_Out_Max = __I_Out_Max;
}

void Set_Out_Max(float __Out_Max,PID_CANSHU* p)
{
//    Out_Max = __Out_Max;
	p->Out_Max = __Out_Max;
}

void Set_I_Variable_Speed_A(float __I_Variable_Speed_A,PID_CANSHU* p)
{
//    I_Variable_Speed_A = __I_Variable_Speed_A;
	p->I_Variable_Speed_A = __I_Variable_Speed_A;
}

void Set_I_Variable_Speed_B(float __I_Variable_Speed_B,PID_CANSHU* p)
{
//    I_Variable_Speed_B = __I_Variable_Speed_B;
	p->I_Variable_Speed_B = __I_Variable_Speed_B;
}

void Set_I_Separate_Threshold(float __I_Separate_Threshold,PID_CANSHU* p)
{
//    I_Separate_Threshold = __I_Separate_Threshold;
	p->I_Separate_Threshold = __I_Separate_Threshold;
}

void Set_Target(float __Target,PID_CANSHU* p)
{
//    Target = __Target;
	p->Target = __Target;
}

void Set_Now(float __Now,PID_CANSHU* p)
{
//    Now = __Now;
	p->Now = __Now;
}

void Set_Integral_Error(float __Integral_Error,PID_CANSHU* p)
{
//    Integral_Error = __Integral_Error;
	p->Integral_Error = __Integral_Error;
}

void PID_Computing(PID_CANSHU* p)
	{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    //误差
    float error;
    //绝对值误差
    float abs_error;
    //线性变速积分
//    float speed_ratio;

//    error = Target - Now;
		error = p->Target - p->Now;
    abs_error = Math_Abs(error);

    //判断死区
    if (abs_error > p->Dead_Zone)
    {
        p->Target = p->Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    //计算p项

//    p_out = K_P * error;
		p_out = p->K_P*error;

    //计算i项

////    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_A == 0.0f)
//		if	(p->I_Variable_Speed_A == 0.0f && p->I_Variable_Speed_B == 0.0f)
//    {
//        //非变速积分
//        speed_ratio = 1.0f;
//    }
//    else
//    {
//        //变速积分
////        if (abs_error <= I_Variable_Speed_B)
//				if (abs_error <= p->I_Variable_Speed_B)
//        {
//            speed_ratio = 1.0f;
//        }
////        else if (I_Variable_Speed_B < abs_error && abs_error < I_Variable_Speed_A + I_Variable_Speed_B)
//				else if (p->I_Variable_Speed_B < abs_error && abs_error < p->I_Variable_Speed_A + p->I_Variable_Speed_B)
//        {  
////					speed_ratio = (I_Variable_Speed_A + I_Variable_Speed_B - abs_error) / I_Variable_Speed_A;
//					speed_ratio = (p->I_Variable_Speed_A + p->I_Variable_Speed_B - abs_error) / p->I_Variable_Speed_A;
//					
//        }
////        else if (abs_error >= I_Variable_Speed_B)
//				else if (abs_error >= p->I_Variable_Speed_B)
//        {
//            speed_ratio = 0.0f;
//        }
//    }
//    //积分限幅
////    if (I_Out_Max != 0.0f)
//		if (p->I_Out_Max != 0.0f)
//    {
////        Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
//			Math_Constrain(&(p->Integral_Error), -(p->I_Out_Max / p->K_I), (p->I_Out_Max / p->K_I));
//    }
////    if (I_Separate_Threshold == 0.0f)
//		if (p->I_Separate_Threshold == 0.0f)
//    {
//        //没有积分分离
////        Integral_Error += speed_ratio * D_T * error;
//			p->Integral_Error += speed_ratio * p->D_T * error;
////        i_out = K_I * Integral_Error;
//			i_out = p->K_I * p->Integral_Error;
//    }
//    else
//    {
//        //积分分离使能
////        if (abs_error < I_Separate_Threshold)
//			if (abs_error < p->I_Separate_Threshold)
//        {
////            Integral_Error += speed_ratio * D_T * error;
//					p->Integral_Error += speed_ratio * p->D_T * error;
////            i_out = K_I * Integral_Error;
//					i_out = p->K_I * p->Integral_Error;
//        }
//        else
//        {
////            Integral_Error = 0.0f;
//					p->Integral_Error = 0.0f;
//            i_out = 0.0f;
//        }
//    }
i_out = p->K_I*(error-2*p->Pre_Error+p->Pre_pre_Error);

    //计算d项

//    if (D_First == PID_D_First_DISABLE)
		if (p->D_First == PID_D_First_DISABLE)
    {
        //没有微分先行
//        d_out = K_D * (error - Pre_Error) / D_T;
			d_out = p->K_D * (error - p->Pre_Error) / p->D_T;
    }
    else
    {
        //微分先行使能
//        d_out = K_D * (Out - Pre_Out) / D_T;
			 d_out = p->K_D * (p->Out - p->Pre_Out) / p->D_T;
    }

    //计算前馈

//    f_out = (Target - Pre_Target) * K_F;
		f_out = (p->Target - p->Pre_Target) * p->K_F;

    //计算总共的输出

//   Out = p_out + i_out + d_out + f_out;
		p->Out = p_out + i_out + d_out + f_out;
    //输出限幅
//    if (Out_Max != 0.0f)
		if (p->Out_Max != 0.0f)
    {
//        Math_Constrain(&Out, -Out_Max, Out_Max);
			Math_Constrain(&(p->Out), -(p->Out_Max), p->Out_Max);
    }

    //善后工作
//    Pre_Now = Now;
		p->Pre_Now = p->Now;
//    Pre_Target = Target;
		 p->Pre_Target = p->Target;
//    Pre_Out = Out;
		 p->Pre_Out = p->Out;
//    Pre_Error = error;
		p->Pre_pre_Error=p->Pre_Error;
		p->Pre_Error = error;
		
}
