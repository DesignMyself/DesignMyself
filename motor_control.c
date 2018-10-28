#include "pwm.h"
#include "motor1.h"
#include "HDMI.H"
#include  <stdlib.h>


/* Private variables ---------------------------------------------------------*/
 motor motor1;
 motor motor2;
 uint16_t Line_speed=5;
 float speed=0.0;
 float Motor2_Cspeed=0;
 uint8_t sign_speed=1;
 extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
extern float ADC1_value;
extern float ADC2_value;
extern float motor_speed_r_min;
extern 	rt_uint8_t Dis_Para[30];//串口3获得屏幕发过来的数据
extern Display D1;//用于main中的任务，把参数发送给屏幕
extern Display D2;//用于main中的任务，把参数发送给屏幕
PID	Motor1_PID;
PID Motor2_PID;
 

/********电机1PID参数初始化************/
void Motor1_PID_Init(){
	
	Motor1_PID.P_PID=12.0f;
	Motor1_PID.I_PID=2.0f;
	Motor1_PID.D_PID=0.0f;
	Motor1_PID.Last_Error=0.0f;
	Motor1_PID.Current_Error=0.0f;
	Motor1_PID.D_Error=0.0f;
	Motor1_PID.errsum=0.0f;
	Motor1_PID.Inc_PID=0;
	Motor1_PID.PID_OPEN=1;
	
	
}
/********电机2PID参数初始化************/
void Motor2_PID_Init(){
	
	Motor2_PID.P_PID=1.3f;
	Motor2_PID.I_PID=0.02f;
	Motor2_PID.D_PID=0.0f;
	Motor2_PID.Last_Error=0.0f;
	Motor2_PID.Current_Error=0.0f;
	Motor2_PID.errsum=0.0f;
	Motor2_PID.D_Error=0.0f;
	Motor2_PID.Inc_PID=0;
	Motor2_PID.changetime=20;
	Motor2_PID.PID_OPEN=0;
	
}

/********电机2参数初始化*************/
void motor2_Init()
{
	motor2.Ahead_Distance=0;
	motor2.Back_Distance=0;
	motor2.cw=0;
	motor2.pwm=0;
	motor2.state=0;//0——关闭电机;1——开启电机，手动模式;2——开启电机，自动前进一段距离模式;3——开启电机，自动后退一段距离;4——开启电机，先前进一段距离，再后退一段距离，
	motor2.Target_Speed=15;
	motor2.autorun_state=0;
	motor2.summotor_ahead=0;
	motor2.summotor_back=0;
	motor2.open=PA12;
	motor2.brake=PA10;
	motor2.ccw=PA11;
	motor2.state_record=0;
	motor2.direction=0;//0代表没有方向，按键没被按下，1向前，2向后，3两个按键都被按下
	motor2.pwm_nopid=150;
}/********电机1参数初始化*************/
void motor1_Init()
{
	motor1.Ahead_Distance=0;
	motor1.Back_Distance=0;
	motor1.cw=0;
	motor1.pwm=0;
	motor1.state=0;//0——关闭电机;1——开启电机，手动模式;2——开启电机，自动前进一段距离模式;3——开启电机，自动后退一段距离;4——开启电机，先前进一段距离，再后退一段距离，
	motor1.Target_Speed=2;
	motor1.autorun_state=0;
	motor1.summotor_ahead=0;
	motor1.summotor_back=0;
	motor1.open=PB0;
	motor1.brake=PB12;
	motor1.ccw=PB1;
	motor1.state_record=0;
	motor1.direction=0;//0代表没有方向，按键没被按下，1向左，2向右，3两个按键都被按下
	motor1.pwm_nopid=80;
}
/**
  * 函数功能：增量式PID速度环计算
  * 输入参数：Get_Speed    由编码器得到的速度值 
  *           Target_Speed    目标值
  * 返 回 值：经过PID运算得到的增量值
  * 说    明：增量式 PID 速度环控制设计,计算得到的结果仍然是速度值
  */
float PID_Motor_Control(PID  *Motor_PID,float Get_Speed,float Target_Speed,float changetime)//第一个为PID的参数，第二个为测量到的速度，第三个为目标速度，第四个为变化的时间
{
	float IError=0,Increase_PWM=0;
	IError=Target_Speed-Get_Speed;//单位时间内误差
	Motor_PID->errsum=IError*changetime;//单位时间内误差累计
	Motor_PID->D_Error=(IError-Motor_PID->Last_Error)/changetime;//单位时间内误差变化率
	if((IError<0.5f)&&(IError>-0.5f))
	{
		
		IError=0;
//		Motor1_PID.Last_Error=0;
//		Motor1_PID.Pre_Error=0;
	}
	Increase_PWM=(Motor_PID->P_PID*IError)+(Motor_PID->I_PID*Motor_PID->errsum)+(Motor_PID->D_PID*Motor_PID->D_Error);
	Motor_PID->Last_Error=IError;
	return(Increase_PWM);
	
	
	
}
/*********************************/
void key_scan()
{
	
		if(rt_pin_read(PB4)==1)
			{
				motor2.autorun_state=0;//用于判断是否是自动行走模式，此时按键没被按下
				motor1.autorun_state=0;
			}
			if(rt_pin_read(PB4)==0)
			{
				motor2.autorun_state=1;//用于判断是否是自动行走模式，此时按键被按下
				motor1.autorun_state=1;
			}
			
				
				/*********小车前后判断************/
				if(rt_pin_read(PC12)==0&&rt_pin_read(PC13)==1)
					{

						motor2.direction=1;//小车向前
						
					}
				if(rt_pin_read(PC13)==0&&rt_pin_read(PC12)==1)
					{

						motor2.direction=2;//小车向后
						
					
					}
				if(rt_pin_read(PC13)==1&&rt_pin_read(PC12)==1)
					{

						motor2.direction=0;//小车停止，此时累计的距离数据会清0
					
					}
				if(rt_pin_read(PC13)==0&&rt_pin_read(PC12)==0)
					{

						motor2.direction=3;//小车停止
					
					}
					/********X轴左右判断********************/
				if(rt_pin_read(PC5)==0&&rt_pin_read(PC4)==1&&rt_pin_read(PB5)!=0)//小车向左运行，行程开关没有到达左终点，左的按键处于打开
					{

						motor1.direction=1;//向左运动
						
					}
				if(rt_pin_read(PC4)==0&&rt_pin_read(PC5)==1&&rt_pin_read(PB3)!=0)//小车向右运行，行程开关没有到达右终点，右的按键处于关闭,
					{

						motor1.direction=2;//向右运动
						
					}
					if(rt_pin_read(PC5)==0&&rt_pin_read(PC4)==1&&rt_pin_read(PB5)==0)//小车向左运行，行程开关到达左终点
					{

						motor1.direction=0;//停止运动 
						D2.message="X轴到达左终点";
						
					}
					if(rt_pin_read(PC4)==0&&rt_pin_read(PC5)==1&&rt_pin_read(PB3)==0)//小车向右运行，行程开关没有到达右终点，右的按键处于关闭,
					{

						motor1.direction=0;//停止运动
						D2.message="X轴到达右终点";
						
					}
					if(rt_pin_read(PC4)==0&&rt_pin_read(PC5)==0)//两个按键同时被按下
					{

						motor1.direction=3;//停止运动 
						D2.message="X轴停止运动，两按钮被同时按下";
						
					}
					if(rt_pin_read(PC4)==1&&rt_pin_read(PC5)==1)//两个按键同时被按下
					{

						motor1.direction=0;//停止运动
						D2.message="X轴停止运动";
						
					}
}
/*************
* 函数功能：对小车的电机进行控制
  * 输入参数：
  * 返 回 值：
  * 说    明：
  ***************/

void motor2_control()
{
		if(motor2.direction==1&&motor2.autorun_state==0&&Dis_Para[2]!=0xc1)//发送前进命令的标志
				{
									motor2.cw=1;
									motor2.state=1;//开启手动模式
									D1.message="小车在前进";
					}
		if(motor2.direction==2&&motor2.autorun_state==0&&Dis_Para[2]!=0xc1)//发送后退的标志
					{
									motor2.cw=0;
									motor2.state=1;//开启手动模式
									D1.message="小车在后退";
							}
		if(((motor2.direction==0)&&motor2.autorun_state==0))//发送停止的标志
					{
									
									motor2.state=0;//开启手动模式
									D1.message="小车停止运行";
							}
		if(((motor2.direction==3)&&motor2.autorun_state==0)||Dis_Para[2]==0xc1)//发送停止的标志
					{
									
									motor2.state=0;//开启手动模式
									D1.message="小车停止运行";
							}
		if(motor2.direction==1&&motor2.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor2.Ahead_Distance<D1.Input_Ahead_Distance))//发送自动前进命令的标志
							{
									motor2.cw=1;
									//开启自动模式
									motor2.state=2;//关闭手动模式
									
									D1.message="小车在自动前进输入的距离";
							}
							
		if(motor2.direction==2&&motor2.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor2.Back_Distance<D1.Input_Back_Distance))//发送自动后退的标志
								{
									motor2.cw=0;
									//开启自动模式
									motor2.state=3;//关闭手动模式
									D1.message="小车在自动后退输入的距离";
									
							}
		if(((motor2.direction==0)&&motor2.autorun_state==1))//发送停止的标志
					{
									
									motor2.state=0;//开启手动模式
							
									motor2.summotor_ahead=0;//行程数据清零
									motor2.summotor_back=0;//行程数据清零
							
									D1.message="小车停止运行";
					}	
		if(((motor2.direction==3)&&motor2.autorun_state==1)||Dis_Para[2]==0xc1)//发送停止的标志
					{
									
									motor2.state=0;//开启手动模式
									D1.message="小车停止运行，前后按钮被同时按下";
									
							}
			if(motor2.state==1)//开启电机2
			{
				rt_pin_write(motor2.open,1);//open motor 1
				rt_pin_write(motor2.brake,0);//打开刹车
				motor2.state_record=1;
			}

			if(motor2.state==0)//关闭电机2
			{
				rt_pin_write(motor2.open,0);//close motor 1
//				rt_pin_write(motor2.ccw,1);//motor1 cw ahead
				rt_pin_write(motor2.brake,1);
				//rt_pin_write(PA12,1);//motor1 cw ahead
				motor2.pwm=0;
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);//小车电机的PWM
				Motor2_PID.Last_Error=0.0f;
				Motor2_PID.Current_Error=0.0f;
				Motor2_PID.errsum=0.0f;
				Motor2_PID.D_Error=0.0f;
				Motor2_PID.Inc_PID=0;
				
				
			}
//			if(motor2.recover==1)//用来恢复当时的运行状态
//			{
//				
//				motor2.state=motor2.state_record;
//				
//			}
			/******自动前进输入的距离*********/

					Auto_Ahead_Distance_JP(&motor2,&D1,2);
						D1.test=motor2.state_record;
				/******自动后退输入的距离*********/
					Auto_Back_Distance_JP(&motor2,&D1,3);
			if(motor2.cw==1)//电机2正向行驶
			{
				rt_pin_write(motor2.ccw,0);//motor1 cw ahead
				
				
				
			}
			if(motor2.cw==0)//电机2反向行驶
			{
				rt_pin_write(motor2.ccw,1);//motor1 cw back
					
			}
			
	
}
/*************
* 函数功能：对X轴进行控制
  * 输入参数：
  * 返 回 值：
  * 说    明：
  ***************/

void motor1_control()
{
		if(motor1.direction==1&&Dis_Para[2]!=0xc1)//发送向左命令的标志&&motor1.autorun_state==0
				{
									motor1.cw=0;
									motor1.state=1;//开启手动模式
									D2.message="X轴向右运动";

					}
		if(motor1.direction==2&&Dis_Para[2]!=0xc1)//发送向右的标志&&motor1.autorun_state==0
					{
									motor1.cw=1;
									motor1.state=1;//开启手动模式
									D2.message="X轴向左运动";

							}
		if(motor1.direction==0)//发送停止的标志&&motor1.autorun_state==0
					{
									
									motor1.state=0;//开启手动模式

							}
		if((motor1.direction==3)||Dis_Para[2]==0xc1)//发送停止的标志&&motor1.autorun_state==0)
					{
									
									motor1.state=0;//开启手动模式
									

							}
//		if(motor1.direction==1&&motor1.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor1.Ahead_Distance<D2.Input_Ahead_Distance))//发送自动向左命令的标志
//							{
//									motor1.cw=0;
//									//开启自动模式
//									motor1.state=2;//关闭手动模式
//									motor1.pwm=motor1.pwm_nopid;
//							D2.message="X轴在自动往左运行预定距离";
//							}
//							
//		if(motor1.direction==2&&motor1.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor1.Back_Distance<D2.Input_Back_Distance))//发送自动向右的标志
//								{
//									motor1.cw=1;
//									
//									motor1.state=3;//关闭手动模式
//									motor1.pwm=motor1.pwm_nopid;
//									D2.message="X轴在自动往右运行预定距离";

//									
//							}
//		if(((motor1.direction==0)&&motor1.autorun_state==1))//发送停止的标志
//					{
//									
//									motor1.state=0;//开启手动模式
//							
//									motor1.summotor_ahead=0;//行程数据清零
//									motor1.summotor_back=0;//行程数据清零
//							
//									D2.message="X轴停止运行";
//					}	
//		if(((motor1.direction==3)&&motor1.autorun_state==1)||Dis_Para[2]==0xc1)//发送停止的标志
//					{
//									
//									motor1.state=0;//开启手动模式
//									D2.message="X轴停止运行";
//									
//							}
			if(motor1.state==1)//开启电机2
			{
				rt_pin_write(motor1.open,1);//open motor 1
				motor1.pwm=500;
			}

			if(motor1.state==0)//关闭电机2
			{
				rt_pin_write(motor1.open,0);//close motor 1
				motor1.cw=1;
				motor1.pwm=0;
				Motor1_PID.Last_Error=0.0f;
				Motor1_PID.Current_Error=0.0f;
				Motor1_PID.errsum=0.0f;
				Motor1_PID.D_Error=0.0f;
				Motor1_PID.Inc_PID=0;
				
				
			}
//			if(motor2.recover==1)//用来恢复当时的运行状态
//			{
//				
//				motor2.state=motor2.state_record;
//				
//			}
			/******自动向左输入的距离*********/

					Auto_Ahead_Distance_JP(&motor1,&D2,2);
						
				/******自动向右输入的距离*********/
					Auto_Back_Distance_JP(&motor1,&D2,3);
			if(motor1.cw==1)//电机2向左
			{
				rt_pin_write(motor1.ccw,0);//motor1 cw ahead
				
				
				
			}
			if(motor1.cw==0)//电机2向右
			{
				rt_pin_write(motor1.ccw,1);//motor1 cw back
					
			}
			
	
}
static struct rt_thread  * PWM_Para_thread;
static struct rt_thread  * PID_thread;
static void PID_entry(void *parameter)
{
		
		
		
	
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,motor2.pwm);//小车电机的PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,motor1.pwm);//横向小车的PWM
							
	while(1)
	{
						
				
						key_scan();
						motor1_control();
			
							__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,motor1.pwm);
						
							
					if(motor2.state>0&&Motor2_PID.PID_OPEN==1)//使用PID模式，恒速输出
					{
						motor2.pwm+=PID_Motor_Control(&Motor2_PID,motor2.speed,motor2.Target_Speed,Motor2_PID.changetime);//电机2PID累加
								motor2.pwm=abs(motor2.pwm);
						
							__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,motor2.pwm);
						
						
					}
					if(motor2.state>0&&Motor2_PID.PID_OPEN==0)//使用PWM模式，速度不恒定
					{		
							__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,motor2.pwm_nopid);
						
					}		
					rt_thread_delay(Motor2_PID.changetime);
			}
	
}
static void Motor_Contorl_entry1(void *parameter)
{	
	
	
	while(1)
	{
			
			motor2_control();
			rt_thread_delay(2);
		}
		
				
}
int PWM_Init()
{
	
	
		 MX_GPIO_Init();//初始化GPIO时钟
		MX_TIM4_Init();
		MX_TIM12_Init();//初始化PWM
		Motor1_PID_Init();
		Motor2_PID_Init();
		motor2_Init();
		motor1_Init();
		motor1.pwm=50;
		motor2.pwm=150;
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//启动PWM通道
		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//启动PWM通道
		
		 PWM_Para_thread= rt_thread_create("PWMLED",//任务的名称
														Motor_Contorl_entry1,//任务的主程序入口
														RT_NULL,//任务的传入参数
														1024,//堆栈的大小
														4,//优先级大小
														10);//时间片，在同一优先级时使用
		if(PWM_Para_thread!= RT_NULL)
		{
			
			rt_thread_startup(PWM_Para_thread);
			
		}
		
	PID_thread= rt_thread_create("PID",//任务的名称
														PID_entry,//任务的主程序入口
														RT_NULL,//任务的传入参数
														1024,//堆栈的大小
														8,//优先级大小
														10);//时间片，在同一优先级时使用
		if(PID_thread!= RT_NULL)
		{
			
			rt_thread_startup(PID_thread);
			
		}
		
		return 0;
	
	
	
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/* 如果设置了RT_SAMPLES_AUTORUN，则加入到初始化线程中自动运行 */
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(PWM_Init);
#endif
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(PWM_Init, PWM_Init_Sample);
