#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.

////******************************************************************************

////定义结构体CANMessage（仿照库函数中can发送数据得结构体CanTxMsg来定义）
//typedef struct
//{
//  uint32_t StdId;  /*!< Specifies the standard identifier.
//                        This parameter can be a value between 0 to 0x7FF. */

//  uint32_t ExtId;  /*!< Specifies the extended identifier.
//                        This parameter can be a value between 0 to 0x1FFFFFFF. */

//  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
//                        will be transmitted. This parameter can be a value 
//                        of @ref CAN_identifier_type */

//  uint8_t RTR;     /*!< Specifies the type of frame for the message that will 
//                        be transmitted. This parameter can be a value of 
//                        @ref CAN_remote_transmission_request */

//  uint8_t DLC;     /*!< Specifies the length of the frame that will be 
//                        transmitted. This parameter can be a value between 
//                        0 to 8 */

//  //学长例程中只用到了结构体的成员data[8]
//  uint8_t data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
//                        to 0xFF. */
//} CANMessage;

////******************************************************************************

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
#endif

















