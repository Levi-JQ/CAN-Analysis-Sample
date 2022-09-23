#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "can.h"
#include <math.h>

//定义电机数据：
float postion,speed,torque;
float P_MIN = -12.5;
float P_MAX = 12.5;
float V_MIN = -20.94;
float V_MAX = 20.94;
float T_MIN = -24.8;
float T_MAX = 24.8;
float Kp_MIN = 0;
float Kp_MAX = 500;
float Kd_MIN = 0;
float Kd_MAX = 5;
float Test_Pos = 0.0;
//

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件


//发包时所有的数都要经以下函数转化成整型数之后再发给电机。
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits /// 
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) / span)));
}


//收包时所有的数都要经以下函数转化成浮点型。
float uint_to_float(int x_int, float x_min, float x_max, int bits) 
{
    /// converts unsigned int to float, given range and number of bits /// 
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}



//发送例程代码
/*
msg是指针，指向待发送数据canbuf[8]，即main()中应有u8 canbuf[8];u8 (*msg)[8];msg=&canbuf
这个函数经过（1）（2）（3）生成了待发送数据canbuf[8]
*/
void pack_cmd(u8 (*msg)[8], float p_des, float v_des, float kp, float kd, float t_ff){ 
    //这些设定的电机参数，根据他们生成待发送数据
//	float P_MIN = -12.5;
//    float P_MAX = 12.5;
//    float V_MIN = -20.94;
//    float V_MAX = 20.94;
//    float T_MIN = -24.8;
//    float T_MAX = 24.8;
//    float Kp_MIN = 0;
//    float Kp_MAX = 500;
//    float Kd_MIN = 0;
//    float Kd_MAX = 5;
//    float Test_Pos = 0.0;
	int p_int,v_int,kp_int,kd_int,t_int;
	
	///（1） limit data to be within bounds ///
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    
    /// （2）convert floats to unsigned ints /// 
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    
    /// （3）pack ints into the can buffer /// 
    (*msg)[0] = p_int >> 8;  		//位置高 8 
    (*msg)[1] = p_int & 0xFF; 		//位置低 8 
    (*msg)[2] = v_int >> 4; 			//速度高 8 位 
    (*msg)[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);	//速度低 4 位 KP 高 4 位 
    (*msg)[4] = kp_int & 0xFF; 		//KP 低 8 位 
    (*msg)[5] = kd_int >> 4; 		//Kd 高 8 位 
    (*msg)[6] = ((kd_int & 0xF) << 4) | (kp_int >> 8); 	//KP 低 4 位扭矩高 4 位 
    (*msg)[7] = t_int & 0xff;			//扭矩低 8 位 
}


//接收例程代码
void unpack_reply(u8 (*msg)[8]) 
{
    /// unpack ints from can buffer /// 
	float p,v,i;
	int id,p_int,v_int,i_int;
    id = (*msg)[0]; //驱动 ID 号 
    p_int = ((*msg)[1] << 8) | (*msg)[2]; 			//电机位置数据 
    v_int = ((*msg)[3] << 4) | ((*msg)[4] >> 4); 		//电机速度数据
    i_int = (((*msg)[4] & 0xF) << 8) | (*msg)[5];		//电机扭矩数据
    /// convert ints to floats /// 
    p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    if (id == 1) {
        postion = p; 									 //根据 ID 号读取对应数据 
        speed = v;
        torque = i;
    }
}




//ALIENTEK 探索者STM32F407开发板 实验27
//CAN通信实验-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK

int main(void)
{ 
	u8 key;
	u8 i=0,t=0;
	u8 cnt=0;
	u8 canbuf[8];
	u8 res;
	u8 mode=1;//CAN工作模式;0,普通模式;1,环回模式
	//**************************添加开始
	
	u8 (*msg)[8];
	msg=&canbuf;
	//生成待发送数据canbuf
	pack_cmd(msg, 0, 0, 0, 0, 0);//后面数据待给出
	
	//**************************添加结束
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED 
 	LCD_Init();					//LCD初始化 
	KEY_Init(); 				//按键初始化  
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);//CAN初始化环回模式,波特率500Kbps    
 	POINT_COLOR=RED;//设置字体为红色 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"CAN TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/7");
	LCD_ShowString(30,130,200,16,16,"LoopBack Mode");	 
	LCD_ShowString(30,150,200,16,16,"KEY0:Send WK_UP:Mode");//显示提示信息		
    POINT_COLOR=BLUE;//设置字体为蓝色	   
	LCD_ShowString(30,170,200,16,16,"Count:");		  	//显示当前计数值	
	LCD_ShowString(30,190,200,16,16,"Send Data:");		//提示发送的数据	
	LCD_ShowString(30,250,200,16,16,"Receive Data:");	//提示接收到的数据		
 									  
while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)//KEY0按下,发送一次数据
		{
			for(i=0;i<8;i++)
			{
				//canbuf[i]=cnt+i;//填充发送缓冲区
				if(i<4)LCD_ShowxNum(30+i*32,210,canbuf[i],3,16,0X80);	  //显示数据
				else LCD_ShowxNum(30+(i-4)*32,230,canbuf[i],3,16,0X80);	//显示数据
 			}
			res=CAN1_Send_Msg(canbuf,8);//发送8个字节 
			if(res)LCD_ShowString(30+80,190,200,16,16,"Failed");		//提示发送失败
			else LCD_ShowString(30+80,190,200,16,16,"OK    ");	 		//提示发送成功								   
		}else if(key==WKUP_PRES)//WK_UP按下，改变CAN的工作模式
		{	   
			mode=!mode;
			CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,mode);	//CAN普通模式初始化,普通模式,波特率500Kbps
  			POINT_COLOR=RED;//设置字体为红色 
			if(mode==0)//普通模式，需要2个开发板
			{
				LCD_ShowString(30,130,200,16,16,"Nnormal Mode ");	    
			}else //回环模式,一个开发板就可以测试了.
			{
 				LCD_ShowString(30,130,200,16,16,"LoopBack Mode");
			}
 			POINT_COLOR=BLUE;//设置字体为蓝色 
		}		 
		key=CAN1_Receive_Msg(canbuf);
		/*
		CAN1_Receive_Msg(canbuf)执行后，canbuf中变成了接收到的数据，那么对canbuf再用接收例程代码
		中的void unpack_reply(u8 (*msg)[8]) 函数进行处理，就可以得到postion;speed;torque等数据
		*/
		
		//***********添加开始
		msg=&canbuf;
		unpack_reply(msg);
		//***********添加结束
		
		if(key)//接收到有数据
		{			
			LCD_Fill(30,270,160,310,WHITE);//清除之前的显示
 			for(i=0;i<key;i++)
			{									    
				if(i<4)LCD_ShowxNum(30+i*32,270,canbuf[i],3,16,0X80);	//显示数据
				else LCD_ShowxNum(30+(i-4)*32,290,canbuf[i],3,16,0X80);	//显示数据
 			}
		}
		t++; 
		delay_ms(10);
		if(t==20)
		{
			LED0=!LED0;//提示系统正在运行	
			t=0;
			cnt++;
			LCD_ShowxNum(30+48,170,cnt,3,16,0X80);	//显示数据
		}		   
	} 
	
}



















