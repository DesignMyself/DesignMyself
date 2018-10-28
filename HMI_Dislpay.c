#include "HDMI.H"
#include "app_uSart.h"
/******串口屏结束字符********/
void HMI_End(uint8_t uart_value)
{
	uart_putchar_f(uart_value,0xff);
	uart_putchar_f(uart_value,0xff);
	uart_putchar_f(uart_value,0xff);
}


/********************选择串口发送单个字符****************/
void uart_putchar_f(uint8_t uart_value,uint8_t value)
{
	if(uart_value==3)
				{
					uart3_putchar(value);
				}
			if(uart_value==1)
				{
						uart1_putchar(value);
				}
	
	
}

/********************大彩屏的接收开始指令****************/
void BEGIN_CMD(uint8_t uart_value) 
{
	if(uart_value==3)
	{
		uart3_putchar(0XEE);
	}
	else if(uart_value==1)
	{
		uart1_putchar(0XEE);
		
	}
}
/********************选择串口发送单个字符****************/
void TX_8(uint8_t uart_value,uint8_t P1)
{
		if(uart_value==3)
	{
		uart3_putchar((P1)&0xFF);
	}
	else if(uart_value==2)
	{
		uart1_putchar((P1)&0xFF);
		
	}
	 
}

/*************选择串口发送字符串**********/
void send_string(uint8_t uart_value,char *str2)
{
	
		if(uart_value==3)
				{
					rt_kprintf(str2);
				}
			if(uart_value==1)
				{
						uart1_putstring(str2);
				}
	
}

/********************选择串口发送2个字节****************/
void TX_16(uint8_t uart_value,uint8_t P1)
{
	
	TX_8(uart_value,(P1)>>8);
	TX_8(uart_value,P1);
	
}
///***精度为0.1*****/
//void HDMI0_1(char *str1,uint32_t num)
//{
//	uint8_t ch1[8];
//	uint8_t i=0;
//	ch1[0]=num/1000000+48;
//	ch1[1]=num%1000000/100000+48;
//	ch1[2]=num%1000000%100000/10000+48;
//	ch1[3]=num%1000000%100000%10000/1000+48;
//	ch1[4]=num%1000000%100000%10000%1000/100+48;
//	ch1[5]=num%1000000%100000%10000%1000%100/10+48;
//	ch1[6]=44;
//	ch1[7]=0+48;
//	rt_kprintf(str1);
//	uart_putchar(34);//双引号输入ASII码，括号内为十进制
//	for(i=0;i<8;i++)
//	{
//		
//		uart_putchar(ch1[i]);
//		
//	}
//		uart_putchar(34);//双引号输入ASII码，括号内为十进制
//	HMI_End();
//}


/********正数精度为0.01*************/
void HDMI0_2(uint8_t uart_value,char *str1,float num)
{
	num=(uint32_t)num;
	send_string(uart_value,str1);
	//uart_putchar_f(uart_value,34);//双引号输入ASII码，括号内为十进制
	Value_Asii(uart_value,num);
	//uart_putchar_f(uart_value,34);//双引号输入ASII码
	HMI_End(uart_value);
}


void HDMI_val(uint8_t uart_value,char *str1,uint8_t num)
{
	send_string(uart_value,str1);
	uart_putchar_f(uart_value,num);
	HMI_End(uart_value);
}
/**********负数精度为0.01***************/
/****显示负数精度2位********/
//void HDMI_0_2(char *str1,uint32_t num)
//{
//	uint8_t ch1[9];
//	uint8_t i=0;
//	ch1[0]=45;
//	ch1[1]=num/1000000+48;
//	ch1[2]=num%1000000/100000+48;
//	ch1[3]=num%1000000%100000/10000+48;
//	ch1[4]=num%1000000%100000%10000/1000+48;
//	ch1[5]=num%1000000%100000%10000%1000/100+48;
//	ch1[6]=44;
//	ch1[7]=num%1000000%100000%10000%1000%100/10+48;
//	ch1[8]=num%1000000%100000%10000%1000%100%10+48;
//	rt_kprintf(str1);
//	uart_putchar(34);//双引号输入ASII码
//	for( i=0;i<9;i++)
//	{
//		
//		uart_putchar(ch1[i]);
//		
//	}
//	uart_putchar(34);//双引号输入ASII码
//	HMI_End();
//}


void END_CMD(uint8_t uart_value)
{
	TX_8(uart_value,0XFF);
	TX_8(uart_value,0XFC);
	TX_8(uart_value,0XFF);
	TX_8(uart_value,0XFF);
	
	
}

/*****************把数值转为ASII****/
void Value_Asii(uint8_t uart_value,float value)
{

	uint8_t ch1[12];
	uint8_t i=0;
	uint32_t num;
	uint16_t num1;
	num=(uint32_t)value;
	if((value-num!=0.00f))
		{
			
			num1=(value-num)*1000;
			ch1[0]=num/1000000+48;
			ch1[1]=num%1000000/100000+48;
			ch1[2]=num%1000000%100000/10000+48;
			ch1[3]=num%1000000%100000%10000/1000+48;
			ch1[4]=num%1000000%100000%10000%1000/100+48;
			ch1[5]=num%1000000%100000%10000%1000%100/10+48;
			ch1[6]=num%1000000%100000%10000%1000%100%10+48;
			ch1[7]=46;
			ch1[8]=num1/100+48;
			ch1[9]=num1%100/10+48;
			ch1[10]=num1%1000%10+48;
			if(uart_value==3)
				{
						for(i=0;i<11;i++)
					{
						
							uart3_putchar(ch1[i]);
						
					}
				}
			if(uart_value==1)
				{
						for(i=0;i<11;i++)
					{
						
							uart1_putchar(ch1[i]);
						
					}
				}
		}
	else
	{
			ch1[0]=num/1000000+48;
			ch1[1]=num%1000000/100000+48;
			ch1[2]=num%1000000%100000/10000+48;
			ch1[3]=num%1000000%100000%10000/1000+48;
			ch1[4]=num%1000000%100000%10000%1000/100+48;
			ch1[5]=num%1000000%100000%10000%1000%100/10+48;
			ch1[6]=num%1000000%100000%10000%1000%100%10+48;
			if(uart_value==3)
				{
						for(i=0;i<7;i++)
					{
						
							uart3_putchar(ch1[i]);
						
					}
				}
			if(uart_value==1)
				{
						for(i=0;i<7;i++)
					{
						
							uart1_putchar(ch1[i]);
						
					}
				}
	}	
}

/***************大彩显示信息************/
void SetProgressValue(rt_uint16_t screen_id,rt_uint16_t control_id,char *str,uint8_t uart_value)
{
	BEGIN_CMD(uart_value);
	TX_8(uart_value,0xB1);
	TX_8(uart_value,0x10);
	TX_16(uart_value,screen_id);
	TX_16(uart_value,control_id);
	rt_kprintf(str);
	END_CMD(uart_value);
}
/***************大彩显示句子和数值************/
void SetProgress_mess_value(rt_uint16_t screen_id,rt_uint16_t control_id,char *str1,float value,uint8_t uart_value)
{
//	char * val;
//	*val=(char)value;
	BEGIN_CMD(uart_value);
	TX_8(uart_value,0xB1);
	TX_8(uart_value,0x10);
	TX_16(uart_value,screen_id);
	TX_16(uart_value,control_id);
	send_string( uart_value,str1);
	Value_Asii(uart_value,value);
	//rt_kprintf(val);
	END_CMD(uart_value);
}
