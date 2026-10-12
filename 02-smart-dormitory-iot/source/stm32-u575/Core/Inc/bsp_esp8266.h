/**
  ******************************************************************************
  * @file   bsp_esp8266.h
  * @brief  wifi模组ESP-12F的驱动头文件
  * 
  ******************************************************************************
  */
#ifndef __BSP_ESP8266_H__
#define __BSP_ESP8266_H__
//
#include "main.h"
//
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
//
#if defined (__CC_ARM)
#pragma anon_unions
#endif
//TCP测试 — 连接到 ESP8266(2) 的热点
#define User_ESP8266_SSID     "ESP-Slave-666"          			//ESP8266(2)的热点名称
#define User_ESP8266_PWD      "REPLACE_ME"     	 					//ESP8266(2)的热点密码

#define User_ESP8266_TCPServer_IP     "192.168.4.1"   		//ESP8266(2)的AP默认IP
#define User_ESP8266_TCPServer_PORT   "8266"      				//ESP8266(2)的TCP服务器端口
//

//ESP8266模式选择
typedef enum{
		STA,
		AP,
		STA_AP  
}ENUM_Net_ModeTypeDef;
//网络传输层协议，枚举类型
typedef enum{
		enumTCP,
		enumUDP,
} ENUM_NetPro_TypeDef;
//连接号，指定为该连接号可以防止其他计算机访问同一端口而发生错误
typedef enum{
		Multiple_ID_0 = 0,
		Multiple_ID_1 = 1,
		Multiple_ID_2 = 2,
		Multiple_ID_3 = 3,
		Multiple_ID_4 = 4,
		Single_ID_0 = 5,
} ENUM_ID_NO_TypeDef;
//
#define ESP8266_USART(fmt, ...)  USART_printf(&huart5, fmt, ##__VA_ARGS__)    
//
#define RX_BUF_MAX_LEN 1024       //最大字节数
//
extern struct STRUCT_USART_Fram   //数据帧结构体
{
		char Data_RX_BUF[RX_BUF_MAX_LEN];
		union 
		{
			volatile uint16_t InfAll;
			struct 
			{
				volatile uint16_t FramLength       :15;	// 14:0 
				volatile uint16_t FramFinishFlag   :1;	// 15 
			}InfBit;
		}; 	
}ESP8266_Fram_Record_Struct;
//初始化和TCP功能函数
void ESP8266_Init(UART_HandleTypeDef *huart, uint8_t *DataBuf,uint32_t bound);
void ESP8266_AT_Test(void);
bool ESP8266_Send_AT_Cmd(char *cmd,char *ack1,char *ack2,uint32_t time);
bool ESP8266_Net_Mode_Choose(ENUM_Net_ModeTypeDef enumMode);
bool ESP8266_JoinAP(char * pSSID, char * pPassWord);
bool ESP8266_Enable_MultipleId (FunctionalState enumEnUnvarnishTx);
bool ESP8266_Link_Server(ENUM_NetPro_TypeDef enumE, char * ip, char * ComNum, ENUM_ID_NO_TypeDef id);
bool ESP8266_SendString(FunctionalState enumEnUnvarnishTx, char * pStr, uint32_t ulStrLength, ENUM_ID_NO_TypeDef ucId);
bool ESP8266_UnvarnishSend(void);
void ESP8266_ExitUnvarnishSend(void);
uint8_t ESP8266_Get_LinkStatus(void);
void USART_printf(UART_HandleTypeDef * USARTx, char * Data, ...);
void ESP8266_STA_TCPClient_Test(void);
void ESP8266_ConnectToServer(void);
void ESP8266_ConnectStart(void);
void ESP8266_ConnectTask(void);
uint8_t ESP8266_IsConnected(void);
//
extern volatile uint8_t TcpClosedFlag;
#endif
