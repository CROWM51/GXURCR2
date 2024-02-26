#ifndef __PID_H
#define __PID_H
//微分先行
#define PID_D_First_DISABLE 0
#define PID_D_First_ENABLE 1

//变量定义：

//常量
typedef struct canshu
{
	
float D_T;
float Dead_Zone;
float Pre_Now ;
float Pre_Target ;
float Pre_Out;
float Pre_Error;
float Pre_pre_Error;
float Out ;
float K_P ;
float K_I ;
float K_D ;
float K_F;
float I_Out_Max ;
float Out_Max ;
float I_Variable_Speed_A ;
float I_Variable_Speed_B ;
float I_Separate_Threshold;
float Target;
float Now;
float Integral_Error;
int D_First;
	
}PID_CANSHU;
 extern float Dead_Zone;
 extern float D_T;
extern PID_CANSHU PID_canshu_LFMotor;
extern PID_CANSHU PID_canshu_LBMotor;
extern PID_CANSHU PID_canshu_RFMotor;
extern PID_CANSHU PID_canshu_RBMotor;
//内部变量

////死区, Error在其绝对值内不输出
//   

//		//之前的当前值
//		extern float Pre_Now ;
//    //之前的目标值
//		extern float Pre_Target;
//    //之前的输出值
//		extern float Pre_Out;
//    //前向误差
//		extern float Pre_Error;
//		
//		//读变量

//    //输出值
//    extern float Out;

//    //写变量

//    // PID的P
//    extern float K_P;
//    // PID的I
//    extern float K_I;
//    // PID的D
//    extern float K_D ;
//    //前馈
//    extern float K_F;
//		//积分限幅, 0为不限制
//    extern float I_Out_Max;
//    //输出限幅, 0为不限制
//    extern float Out_Max;

//    //变速积分定速内段阈值, 0为不限制
//    extern float I_Variable_Speed_A;
//    //变速积分变速区间, 0为不限制
//    extern float I_Variable_Speed_B;
//    //积分分离阈值，需为正数, 0为不限制
//    extern float I_Separate_Threshold;

//    //目标值
//    extern float Target;
//    //当前值
//    extern float Now;

//    //读写变量

//    //积分值
//    extern float Integral_Error;
//		extern int D_First;
		
		
//函数定义：
void PID_INIT(float __K_P, float __K_I, float __K_D,float __K_F,
							float __I_Out_Max , float __Out_Max,
							float __Dead_Zone, 
							float __I_Variable_Speed_A , 
							float __I_Variable_Speed_B, 
							float __I_Separate_Threshold,
							int PID_D_First,
							float __D_T,
							PID_CANSHU* PIDcanshu);//pid初始化函数
float Get_Integral_Error(PID_CANSHU p);
float Get_Out(PID_CANSHU p);
void Set_K_P(float __K_P,PID_CANSHU* p);
void Set_K_I(float __K_I,PID_CANSHU* p);
void Set_K_D(float __K_D,PID_CANSHU* p);
void Set_K_F(float __K_F,PID_CANSHU* p);
void Set_I_Out_Max(float __I_Out_Max,PID_CANSHU* p);
void Set_Out_Max(float __Out_Max,PID_CANSHU* p);
void Set_I_Variable_Speed_A(float __I_Variable_Speed_A,PID_CANSHU* p);
void Set_I_Variable_Speed_B(float __I_Variable_Speed_B,PID_CANSHU* p);
void Set_I_Separate_Threshold(float __I_Separate_Threshold,PID_CANSHU* p);
void Set_Now(float __Now,PID_CANSHU* p);
void Set_Target(float __Target,PID_CANSHU* p);
void Set_Integral_Error(float __Integral_Error,PID_CANSHU* p);
void PID_Computing(PID_CANSHU* p);

#endif
