// TMotorCan.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单
//这是tb1的改动zheshiTB!
// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

int main()
{
 
    std::cout << "Hello World!\n";
}


//发送例程代码
void pack_cmd(CANMessage* msg, float p_des, float v_des, float kp, float kd, float t_ff){
    /// limit data to be within bounds /// 
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
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    
    /// convert floats to unsigned ints /// 
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    
    /// pack ints into the can buffer /// 
    msg->data[0] = p_int >> 8;  		//位置高 8 
    msg->data[1] = p_int & 0xFF; 		//位置低 8 
    msg->data[2] = v_int >> 4; 			//速度高 8 位 
    msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);	//速度低 4 位 KP 高 4 位 
    msg->data[4] = kp_int & 0xFF; 		//KP 低 8 位 
    msg->data[5] = kd_int >> 4; 		//Kd 高 8 位 
    msg->data[6] = ((kd_int & 0xF) << 4) | (kp_int >> 8); 	//KP 低 4 位扭矩高 4 位 
    msg->data[7] = t_int & 0xff;			//扭矩低 8 位 
}


//接收例程代码
void unpack_reply(CANMessage msg) 
{
    /// unpack ints from can buffer /// 
    int id = msg.data[0]; //驱动 ID 号 
    int p_int = (msg.data[1] << 8) | msg.data[2]; 			//电机位置数据 
    int v_int = (msg.data[3] << 4) | (msg.data[4] >> 4); 		//电机速度数据
    int i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];		//电机扭矩数据
    /// convert ints to floats /// 
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    if (id == 1) {
        postion = p; 									 //根据 ID 号读取对应数据 
        speed = v;
        torque = i;
    }
}

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

