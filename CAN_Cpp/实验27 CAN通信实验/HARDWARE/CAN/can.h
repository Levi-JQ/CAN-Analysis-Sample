#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	0		//0,��ʹ��;1,ʹ��.

////******************************************************************************

////����ṹ��CANMessage�����տ⺯����can�������ݵýṹ��CanTxMsg�����壩
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

//  //ѧ��������ֻ�õ��˽ṹ��ĳ�Աdata[8]
//  uint8_t data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
//                        to 0xFF. */
//} CANMessage;

////******************************************************************************

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������

u8 CAN1_Receive_Msg(u8 *buf);							//��������
#endif

















