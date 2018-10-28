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
extern 	rt_uint8_t Dis_Para[30];//����3�����Ļ������������
extern Display D1;//����main�е����񣬰Ѳ������͸���Ļ
extern Display D2;//����main�е����񣬰Ѳ������͸���Ļ
PID	Motor1_PID;
PID Motor2_PID;
 

/********���1PID������ʼ��************/
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
/********���2PID������ʼ��************/
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

/********���2������ʼ��*************/
void motor2_Init()
{
	motor2.Ahead_Distance=0;
	motor2.Back_Distance=0;
	motor2.cw=0;
	motor2.pwm=0;
	motor2.state=0;//0�����رյ��;1��������������ֶ�ģʽ;2��������������Զ�ǰ��һ�ξ���ģʽ;3��������������Զ�����һ�ξ���;4���������������ǰ��һ�ξ��룬�ٺ���һ�ξ��룬
	motor2.Target_Speed=15;
	motor2.autorun_state=0;
	motor2.summotor_ahead=0;
	motor2.summotor_back=0;
	motor2.open=PA12;
	motor2.brake=PA10;
	motor2.ccw=PA11;
	motor2.state_record=0;
	motor2.direction=0;//0����û�з��򣬰���û�����£�1��ǰ��2���3����������������
	motor2.pwm_nopid=150;
}/********���1������ʼ��*************/
void motor1_Init()
{
	motor1.Ahead_Distance=0;
	motor1.Back_Distance=0;
	motor1.cw=0;
	motor1.pwm=0;
	motor1.state=0;//0�����رյ��;1��������������ֶ�ģʽ;2��������������Զ�ǰ��һ�ξ���ģʽ;3��������������Զ�����һ�ξ���;4���������������ǰ��һ�ξ��룬�ٺ���һ�ξ��룬
	motor1.Target_Speed=2;
	motor1.autorun_state=0;
	motor1.summotor_ahead=0;
	motor1.summotor_back=0;
	motor1.open=PB0;
	motor1.brake=PB12;
	motor1.ccw=PB1;
	motor1.state_record=0;
	motor1.direction=0;//0����û�з��򣬰���û�����£�1����2���ң�3����������������
	motor1.pwm_nopid=80;
}
/**
  * �������ܣ�����ʽPID�ٶȻ�����
  * ���������Get_Speed    �ɱ������õ����ٶ�ֵ 
  *           Target_Speed    Ŀ��ֵ
  * �� �� ֵ������PID����õ�������ֵ
  * ˵    ��������ʽ PID �ٶȻ��������,����õ��Ľ����Ȼ���ٶ�ֵ
  */
float PID_Motor_Control(PID  *Motor_PID,float Get_Speed,float Target_Speed,float changetime)//��һ��ΪPID�Ĳ������ڶ���Ϊ���������ٶȣ�������ΪĿ���ٶȣ����ĸ�Ϊ�仯��ʱ��
{
	float IError=0,Increase_PWM=0;
	IError=Target_Speed-Get_Speed;//��λʱ�������
	Motor_PID->errsum=IError*changetime;//��λʱ��������ۼ�
	Motor_PID->D_Error=(IError-Motor_PID->Last_Error)/changetime;//��λʱ�������仯��
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
				motor2.autorun_state=0;//�����ж��Ƿ����Զ�����ģʽ����ʱ����û������
				motor1.autorun_state=0;
			}
			if(rt_pin_read(PB4)==0)
			{
				motor2.autorun_state=1;//�����ж��Ƿ����Զ�����ģʽ����ʱ����������
				motor1.autorun_state=1;
			}
			
				
				/*********С��ǰ���ж�************/
				if(rt_pin_read(PC12)==0&&rt_pin_read(PC13)==1)
					{

						motor2.direction=1;//С����ǰ
						
					}
				if(rt_pin_read(PC13)==0&&rt_pin_read(PC12)==1)
					{

						motor2.direction=2;//С�����
						
					
					}
				if(rt_pin_read(PC13)==1&&rt_pin_read(PC12)==1)
					{

						motor2.direction=0;//С��ֹͣ����ʱ�ۼƵľ������ݻ���0
					
					}
				if(rt_pin_read(PC13)==0&&rt_pin_read(PC12)==0)
					{

						motor2.direction=3;//С��ֹͣ
					
					}
					/********X�������ж�********************/
				if(rt_pin_read(PC5)==0&&rt_pin_read(PC4)==1&&rt_pin_read(PB5)!=0)//С���������У��г̿���û�е������յ㣬��İ������ڴ�
					{

						motor1.direction=1;//�����˶�
						
					}
				if(rt_pin_read(PC4)==0&&rt_pin_read(PC5)==1&&rt_pin_read(PB3)!=0)//С���������У��г̿���û�е������յ㣬�ҵİ������ڹر�,
					{

						motor1.direction=2;//�����˶�
						
					}
					if(rt_pin_read(PC5)==0&&rt_pin_read(PC4)==1&&rt_pin_read(PB5)==0)//С���������У��г̿��ص������յ�
					{

						motor1.direction=0;//ֹͣ�˶� 
						D2.message="X�ᵽ�����յ�";
						
					}
					if(rt_pin_read(PC4)==0&&rt_pin_read(PC5)==1&&rt_pin_read(PB3)==0)//С���������У��г̿���û�е������յ㣬�ҵİ������ڹر�,
					{

						motor1.direction=0;//ֹͣ�˶�
						D2.message="X�ᵽ�����յ�";
						
					}
					if(rt_pin_read(PC4)==0&&rt_pin_read(PC5)==0)//��������ͬʱ������
					{

						motor1.direction=3;//ֹͣ�˶� 
						D2.message="X��ֹͣ�˶�������ť��ͬʱ����";
						
					}
					if(rt_pin_read(PC4)==1&&rt_pin_read(PC5)==1)//��������ͬʱ������
					{

						motor1.direction=0;//ֹͣ�˶�
						D2.message="X��ֹͣ�˶�";
						
					}
}
/*************
* �������ܣ���С���ĵ�����п���
  * ���������
  * �� �� ֵ��
  * ˵    ����
  ***************/

void motor2_control()
{
		if(motor2.direction==1&&motor2.autorun_state==0&&Dis_Para[2]!=0xc1)//����ǰ������ı�־
				{
									motor2.cw=1;
									motor2.state=1;//�����ֶ�ģʽ
									D1.message="С����ǰ��";
					}
		if(motor2.direction==2&&motor2.autorun_state==0&&Dis_Para[2]!=0xc1)//���ͺ��˵ı�־
					{
									motor2.cw=0;
									motor2.state=1;//�����ֶ�ģʽ
									D1.message="С���ں���";
							}
		if(((motor2.direction==0)&&motor2.autorun_state==0))//����ֹͣ�ı�־
					{
									
									motor2.state=0;//�����ֶ�ģʽ
									D1.message="С��ֹͣ����";
							}
		if(((motor2.direction==3)&&motor2.autorun_state==0)||Dis_Para[2]==0xc1)//����ֹͣ�ı�־
					{
									
									motor2.state=0;//�����ֶ�ģʽ
									D1.message="С��ֹͣ����";
							}
		if(motor2.direction==1&&motor2.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor2.Ahead_Distance<D1.Input_Ahead_Distance))//�����Զ�ǰ������ı�־
							{
									motor2.cw=1;
									//�����Զ�ģʽ
									motor2.state=2;//�ر��ֶ�ģʽ
									
									D1.message="С�����Զ�ǰ������ľ���";
							}
							
		if(motor2.direction==2&&motor2.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor2.Back_Distance<D1.Input_Back_Distance))//�����Զ����˵ı�־
								{
									motor2.cw=0;
									//�����Զ�ģʽ
									motor2.state=3;//�ر��ֶ�ģʽ
									D1.message="С�����Զ���������ľ���";
									
							}
		if(((motor2.direction==0)&&motor2.autorun_state==1))//����ֹͣ�ı�־
					{
									
									motor2.state=0;//�����ֶ�ģʽ
							
									motor2.summotor_ahead=0;//�г���������
									motor2.summotor_back=0;//�г���������
							
									D1.message="С��ֹͣ����";
					}	
		if(((motor2.direction==3)&&motor2.autorun_state==1)||Dis_Para[2]==0xc1)//����ֹͣ�ı�־
					{
									
									motor2.state=0;//�����ֶ�ģʽ
									D1.message="С��ֹͣ���У�ǰ��ť��ͬʱ����";
									
							}
			if(motor2.state==1)//�������2
			{
				rt_pin_write(motor2.open,1);//open motor 1
				rt_pin_write(motor2.brake,0);//��ɲ��
				motor2.state_record=1;
			}

			if(motor2.state==0)//�رյ��2
			{
				rt_pin_write(motor2.open,0);//close motor 1
//				rt_pin_write(motor2.ccw,1);//motor1 cw ahead
				rt_pin_write(motor2.brake,1);
				//rt_pin_write(PA12,1);//motor1 cw ahead
				motor2.pwm=0;
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);//С�������PWM
				Motor2_PID.Last_Error=0.0f;
				Motor2_PID.Current_Error=0.0f;
				Motor2_PID.errsum=0.0f;
				Motor2_PID.D_Error=0.0f;
				Motor2_PID.Inc_PID=0;
				
				
			}
//			if(motor2.recover==1)//�����ָ���ʱ������״̬
//			{
//				
//				motor2.state=motor2.state_record;
//				
//			}
			/******�Զ�ǰ������ľ���*********/

					Auto_Ahead_Distance_JP(&motor2,&D1,2);
						D1.test=motor2.state_record;
				/******�Զ���������ľ���*********/
					Auto_Back_Distance_JP(&motor2,&D1,3);
			if(motor2.cw==1)//���2������ʻ
			{
				rt_pin_write(motor2.ccw,0);//motor1 cw ahead
				
				
				
			}
			if(motor2.cw==0)//���2������ʻ
			{
				rt_pin_write(motor2.ccw,1);//motor1 cw back
					
			}
			
	
}
/*************
* �������ܣ���X����п���
  * ���������
  * �� �� ֵ��
  * ˵    ����
  ***************/

void motor1_control()
{
		if(motor1.direction==1&&Dis_Para[2]!=0xc1)//������������ı�־&&motor1.autorun_state==0
				{
									motor1.cw=0;
									motor1.state=1;//�����ֶ�ģʽ
									D2.message="X�������˶�";

					}
		if(motor1.direction==2&&Dis_Para[2]!=0xc1)//�������ҵı�־&&motor1.autorun_state==0
					{
									motor1.cw=1;
									motor1.state=1;//�����ֶ�ģʽ
									D2.message="X�������˶�";

							}
		if(motor1.direction==0)//����ֹͣ�ı�־&&motor1.autorun_state==0
					{
									
									motor1.state=0;//�����ֶ�ģʽ

							}
		if((motor1.direction==3)||Dis_Para[2]==0xc1)//����ֹͣ�ı�־&&motor1.autorun_state==0)
					{
									
									motor1.state=0;//�����ֶ�ģʽ
									

							}
//		if(motor1.direction==1&&motor1.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor1.Ahead_Distance<D2.Input_Ahead_Distance))//�����Զ���������ı�־
//							{
//									motor1.cw=0;
//									//�����Զ�ģʽ
//									motor1.state=2;//�ر��ֶ�ģʽ
//									motor1.pwm=motor1.pwm_nopid;
//							D2.message="X�����Զ���������Ԥ������";
//							}
//							
//		if(motor1.direction==2&&motor1.autorun_state==1&&Dis_Para[2]!=0xc1&&(motor1.Back_Distance<D2.Input_Back_Distance))//�����Զ����ҵı�־
//								{
//									motor1.cw=1;
//									
//									motor1.state=3;//�ر��ֶ�ģʽ
//									motor1.pwm=motor1.pwm_nopid;
//									D2.message="X�����Զ���������Ԥ������";

//									
//							}
//		if(((motor1.direction==0)&&motor1.autorun_state==1))//����ֹͣ�ı�־
//					{
//									
//									motor1.state=0;//�����ֶ�ģʽ
//							
//									motor1.summotor_ahead=0;//�г���������
//									motor1.summotor_back=0;//�г���������
//							
//									D2.message="X��ֹͣ����";
//					}	
//		if(((motor1.direction==3)&&motor1.autorun_state==1)||Dis_Para[2]==0xc1)//����ֹͣ�ı�־
//					{
//									
//									motor1.state=0;//�����ֶ�ģʽ
//									D2.message="X��ֹͣ����";
//									
//							}
			if(motor1.state==1)//�������2
			{
				rt_pin_write(motor1.open,1);//open motor 1
				motor1.pwm=500;
			}

			if(motor1.state==0)//�رյ��2
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
//			if(motor2.recover==1)//�����ָ���ʱ������״̬
//			{
//				
//				motor2.state=motor2.state_record;
//				
//			}
			/******�Զ���������ľ���*********/

					Auto_Ahead_Distance_JP(&motor1,&D2,2);
						
				/******�Զ���������ľ���*********/
					Auto_Back_Distance_JP(&motor1,&D2,3);
			if(motor1.cw==1)//���2����
			{
				rt_pin_write(motor1.ccw,0);//motor1 cw ahead
				
				
				
			}
			if(motor1.cw==0)//���2����
			{
				rt_pin_write(motor1.ccw,1);//motor1 cw back
					
			}
			
	
}
static struct rt_thread  * PWM_Para_thread;
static struct rt_thread  * PID_thread;
static void PID_entry(void *parameter)
{
		
		
		
	
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,motor2.pwm);//С�������PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,motor1.pwm);//����С����PWM
							
	while(1)
	{
						
				
						key_scan();
						motor1_control();
			
							__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,motor1.pwm);
						
							
					if(motor2.state>0&&Motor2_PID.PID_OPEN==1)//ʹ��PIDģʽ���������
					{
						motor2.pwm+=PID_Motor_Control(&Motor2_PID,motor2.speed,motor2.Target_Speed,Motor2_PID.changetime);//���2PID�ۼ�
								motor2.pwm=abs(motor2.pwm);
						
							__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,motor2.pwm);
						
						
					}
					if(motor2.state>0&&Motor2_PID.PID_OPEN==0)//ʹ��PWMģʽ���ٶȲ��㶨
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
	
	
		 MX_GPIO_Init();//��ʼ��GPIOʱ��
		MX_TIM4_Init();
		MX_TIM12_Init();//��ʼ��PWM
		Motor1_PID_Init();
		Motor2_PID_Init();
		motor2_Init();
		motor1_Init();
		motor1.pwm=50;
		motor2.pwm=150;
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//����PWMͨ��
		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);//����PWMͨ��
		
		 PWM_Para_thread= rt_thread_create("PWMLED",//���������
														Motor_Contorl_entry1,//��������������
														RT_NULL,//����Ĵ������
														1024,//��ջ�Ĵ�С
														4,//���ȼ���С
														10);//ʱ��Ƭ����ͬһ���ȼ�ʱʹ��
		if(PWM_Para_thread!= RT_NULL)
		{
			
			rt_thread_startup(PWM_Para_thread);
			
		}
		
	PID_thread= rt_thread_create("PID",//���������
														PID_entry,//��������������
														RT_NULL,//����Ĵ������
														1024,//��ջ�Ĵ�С
														8,//���ȼ���С
														10);//ʱ��Ƭ����ͬһ���ȼ�ʱʹ��
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

/* ���������RT_SAMPLES_AUTORUN������뵽��ʼ���߳����Զ����� */
#if defined (RT_SAMPLES_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
    INIT_APP_EXPORT(PWM_Init);
#endif
/* ������ msh �����б��� */
MSH_CMD_EXPORT(PWM_Init, PWM_Init_Sample);
