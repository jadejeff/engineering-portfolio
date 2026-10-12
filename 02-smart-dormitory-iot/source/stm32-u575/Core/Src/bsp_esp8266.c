/**
  ******************************************************************************
  * @file   bsp_esp8266.c
  * @brief  wifi模组ESP-12F的驱动程序
  * 
  ******************************************************************************
  */
#include "bsp_esp8266.h"
#include <stdarg.h>
#include "usart.h"
//
struct STRUCT_USART_Fram ESP8266_Fram_Record_Struct = { 0 };  //定义了一个数据帧结构体

typedef enum
{
    ESP_CONNECT_IDLE = 0,
    ESP_CONNECT_BOOT_WAIT,
    ESP_CONNECT_MODE_WAIT,
    ESP_CONNECT_AP_WAIT,
    ESP_CONNECT_MUX_WAIT,
    ESP_CONNECT_SERVER_WAIT,
    ESP_CONNECT_CIPMODE_WAIT,
    ESP_CONNECT_SEND_WAIT,
    ESP_CONNECT_RETRY_WAIT,
    ESP_CONNECT_READY
} ESP8266_ConnectState;

static ESP8266_ConnectState s_connect_state = ESP_CONNECT_IDLE;
static uint32_t s_connect_deadline;
static uint8_t s_connected;

static uint8_t ESP8266_TimeReached(uint32_t deadline)
{
    return ((int32_t)(HAL_GetTick() - deadline) >= 0) ? 1U : 0U;
}

static void ESP8266_ResetResponse(void)
{
    __disable_irq();
    ESP8266_Fram_Record_Struct.InfAll = 0U;
    ESP8266_Fram_Record_Struct.Data_RX_BUF[0] = '\0';
    __enable_irq();
}

static uint8_t ESP8266_ResponseContains(const char *text)
{
    uint16_t length = ESP8266_Fram_Record_Struct.InfBit.FramLength;

    if (length >= RX_BUF_MAX_LEN)
    {
        length = RX_BUF_MAX_LEN - 1U;
    }

    ESP8266_Fram_Record_Struct.Data_RX_BUF[length] = '\0';
    return (strstr(ESP8266_Fram_Record_Struct.Data_RX_BUF, text) != 0) ? 1U : 0U;
}

static void ESP8266_SendAsyncCommand(const char *command,
                                     ESP8266_ConnectState wait_state,
                                     uint32_t timeout_ms)
{
    ESP8266_ResetResponse();
    ESP8266_USART("%s\r\n", command);
    s_connect_state = wait_state;
    s_connect_deadline = HAL_GetTick() + timeout_ms;
}

static void ESP8266_ScheduleRetry(void)
{
    s_connected = 0U;
    s_connect_state = ESP_CONNECT_RETRY_WAIT;
    s_connect_deadline = HAL_GetTick() + 3000U;
}
//初始化波特率
void ESP8266_Init(UART_HandleTypeDef *huart, uint8_t *DataBuf,uint32_t bound)
{
	//设置波特率
  huart->Init.BaudRate = bound;
	//初始化配置
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    Error_Handler();
  }	
	//开启逐字节接收；不再通过空闲中断反复中止接收
	HAL_UART_Receive_IT(huart,(uint8_t *)DataBuf, 1);	//开启接收中断	
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
}
//对ESP8266模块发送AT指令
// cmd 待发送的指令
// ack1,ack2;期待的响应，为NULL表不需响应，两者为或逻辑关系
// time 等待响应时间
//返回1发送成功， 0失败
bool ESP8266_Send_AT_Cmd(char *cmd,char *ack1,char *ack2,uint32_t time)
{ 
    ESP8266_Fram_Record_Struct .InfBit .FramLength = 0;		//重新接收新的数据包
    ESP8266_USART("%s\r\n", cmd);
    if(ack1==0&&ack2==0)		//不需要接收数据
    {
    return true;
    }
    HAL_Delay(time);	//延时
    ESP8266_Fram_Record_Struct.Data_RX_BUF[ESP8266_Fram_Record_Struct.InfBit.FramLength ] = '\0';
		//
    printf("%s",ESP8266_Fram_Record_Struct .Data_RX_BUF);
		//
    if((ack1!=0) && (ack2!=0))
    {
        return ( ( bool ) strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, ack1 ) || 
                         ( bool ) strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, ack2 ) );
    }
    else if( ack1 != 0 )  //strstr(s1,s2);检测s2是否为s1的一部分，是返回该位置，否则返回false，它强制转换为bool类型了
        return ( ( bool ) strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, ack1 ) );
    else
        return ( ( bool ) strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, ack2 ) );
}
//发送恢复出厂默认设置指令将模块恢复成出厂设置
void ESP8266_AT_Test(void)
{
    char count=0;
    HAL_Delay(100); 
    while(count < 10)
    {
        if(ESP8266_Send_AT_Cmd("AT+RESTORE","OK",NULL,1000)) 
        {
            printf("OK\r\n");
            return;
        }
        ++ count;
    }
}
//选择ESP8266的工作模式
// enumMode 模式类型
//成功返回true，失败返回false
bool ESP8266_Net_Mode_Choose(ENUM_Net_ModeTypeDef enumMode)
{
    switch ( enumMode )
    {
        case STA:
            return ESP8266_Send_AT_Cmd ( "AT+CWMODE=1", "OK", "no change", 2500 ); 

        case AP:
            return ESP8266_Send_AT_Cmd ( "AT+CWMODE=2", "OK", "no change", 2500 ); 

        case STA_AP:
            return ESP8266_Send_AT_Cmd ( "AT+CWMODE=3", "OK", "no change", 2500 ); 

        default:
          return false;
    }       
}
//ESP8266连接外部的WIFI
//pSSID WiFi帐号
//pPassWord WiFi密码
//设置成功返回true 反之false
bool ESP8266_JoinAP(char * pSSID, char * pPassWord)
{
    char cCmd [120];
		//
    sprintf ( cCmd, "AT+CWJAP=\"%s\",\"%s\"", pSSID, pPassWord );
    return ESP8266_Send_AT_Cmd( cCmd, "OK", NULL, 5000 );
}
//ESP8266 透传使能
//enumEnUnvarnishTx  是否多连接，bool类型
//设置成功返回true，反之false
bool ESP8266_Enable_MultipleId(FunctionalState enumEnUnvarnishTx )
{
    char cStr [20];

    sprintf ( cStr, "AT+CIPMUX=%d", ( enumEnUnvarnishTx ? 1 : 0 ) );

    return ESP8266_Send_AT_Cmd ( cStr, "OK", 0, 500 );
}
//ESP8266 连接服务器
//enumE  网络类型
//ip ，服务器IP
//ComNum  服务器端口
//id，连接号，确保通信不受外界干扰
//设置成功返回true，反之fasle
bool ESP8266_Link_Server(ENUM_NetPro_TypeDef enumE, char * ip, char * ComNum, ENUM_ID_NO_TypeDef id)
{
    char cStr [100] = { 0 }, cCmd [120];

    switch (  enumE )
    {
        case enumTCP:
          sprintf ( cStr, "\"%s\",\"%s\",%s", "TCP", ip, ComNum );
          break;

        case enumUDP:
          sprintf ( cStr, "\"%s\",\"%s\",%s", "UDP", ip, ComNum );
          break;

        default:
            break;
    }

    if ( id < 5 )
        sprintf ( cCmd, "AT+CIPSTART=%d,%s", id, cStr);

    else
        sprintf ( cCmd, "AT+CIPSTART=%s", cStr );

    return ESP8266_Send_AT_Cmd ( cCmd, "OK", "ALREAY CONNECT", 4000 );
}
//透传使能
//设置成功返回true， 反之false
bool ESP8266_UnvarnishSend (void)
{
    if (!ESP8266_Send_AT_Cmd ( "AT+CIPMODE=1", "OK", 0, 500 ))
        return false;
    return 
        ESP8266_Send_AT_Cmd( "AT+CIPSEND", "OK", ">", 500 );
}
//ESP8266发送字符串
//enumEnUnvarnishTx是否使能透传模式
//pStr字符串
//ulStrLength字符串长度
//ucId 连接号
//设置成功返回true， 反之false
bool ESP8266_SendString(FunctionalState enumEnUnvarnishTx, char * pStr, uint32_t ulStrLength, ENUM_ID_NO_TypeDef ucId )
{
    char cStr [20];
    bool bRet = false;
		//
    if ( enumEnUnvarnishTx )
    {
        bRet = (HAL_UART_Transmit(&huart5,
                                  (uint8_t *)pStr,
                                  (uint16_t)ulStrLength,
                                  100U) == HAL_OK);

    }
    else
    {
        if ( ucId < 5 )
            sprintf (cStr, "AT+CIPSEND=%d,%d", ucId, ulStrLength + 2 );
        else
            sprintf (cStr, "AT+CIPSEND=%d", ulStrLength + 2 );

        if (!ESP8266_Send_AT_Cmd (cStr, ">", 0, 1000 ))
            return false;  // AT+CIPSEND 失败，ESP8266 未进入数据模式

        bRet = ESP8266_Send_AT_Cmd (pStr, "SEND OK", 0, 1000 );
  }
    return bRet;
}
//ESP8266退出透传模式
void ESP8266_ExitUnvarnishSend (void)
{
    HAL_Delay(1000);
    ESP8266_USART("+++");
    HAL_Delay( 500 );    
}
//ESP8266 检测连接状态
//返回0：获取状态失败
//返回2：获得ip
//返回3：建立连接 
//返回4：失去连接 
uint8_t ESP8266_Get_LinkStatus (void)
{
    if (ESP8266_Send_AT_Cmd( "AT+CIPSTATUS", "OK", 0, 500 ) )
    {
        if ( strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, "STATUS:2\r\n" ) )
            return 2;

        else if ( strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, "STATUS:3\r\n" ) )
            return 3;

        else if ( strstr ( ESP8266_Fram_Record_Struct .Data_RX_BUF, "STATUS:4\r\n" ) )
            return 4;       

    }

    return 0;
}
//
static char *itoa(int value, char *string, int radix )    //把一整数转换为字符串。
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;

    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */
//
void USART_printf(UART_HandleTypeDef * USARTx, char * Data, ... )
{
    char tx_buffer[320];
    int length;
    va_list ap;

    va_start(ap, Data);
    length = vsnprintf(tx_buffer, sizeof(tx_buffer), Data, ap);
    va_end(ap);

    if (length <= 0)
    {
        return;
    }

    if (length >= (int)sizeof(tx_buffer))
    {
        length = (int)sizeof(tx_buffer) - 1;
    }

    HAL_UART_Transmit(USARTx, (uint8_t *)tx_buffer, (uint16_t)length, 50U);
}


volatile uint8_t TcpClosedFlag = 0;
//
// Start the cooperative connection state machine. It never blocks the HMI loop.
void ESP8266_ConnectStart(void)
{
    s_connected = 0U;
    s_connect_state = ESP_CONNECT_BOOT_WAIT;
    s_connect_deadline = HAL_GetTick() + 500U;
}

uint8_t ESP8266_IsConnected(void)
{
    return s_connected;
}

void ESP8266_ConnectTask(void)
{
    char command[160];

    switch (s_connect_state)
    {
        case ESP_CONNECT_IDLE:
        case ESP_CONNECT_READY:
            return;

        case ESP_CONNECT_BOOT_WAIT:
            if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_SendAsyncCommand("AT+CWMODE=1",
                                         ESP_CONNECT_MODE_WAIT,
                                         1500U);
            }
            break;

        case ESP_CONNECT_MODE_WAIT:
            if (ESP8266_ResponseContains("OK") ||
                ESP8266_ResponseContains("no change"))
            {
                snprintf(command,
                         sizeof(command),
                         "AT+CWJAP=\"%s\",\"%s\"",
                         User_ESP8266_SSID,
                         User_ESP8266_PWD);
                ESP8266_SendAsyncCommand(command,
                                         ESP_CONNECT_AP_WAIT,
                                         8000U);
            }
            else if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_ScheduleRetry();
            }
            break;

        case ESP_CONNECT_AP_WAIT:
            if (ESP8266_ResponseContains("OK"))
            {
                ESP8266_SendAsyncCommand("AT+CIPMUX=0",
                                         ESP_CONNECT_MUX_WAIT,
                                         1500U);
            }
            else if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_ScheduleRetry();
            }
            break;

        case ESP_CONNECT_MUX_WAIT:
            if (ESP8266_ResponseContains("OK"))
            {
                snprintf(command,
                         sizeof(command),
                         "AT+CIPSTART=\"TCP\",\"%s\",%s",
                         User_ESP8266_TCPServer_IP,
                         User_ESP8266_TCPServer_PORT);
                ESP8266_SendAsyncCommand(command,
                                         ESP_CONNECT_SERVER_WAIT,
                                         6000U);
            }
            else if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_ScheduleRetry();
            }
            break;

        case ESP_CONNECT_SERVER_WAIT:
            if (ESP8266_ResponseContains("OK") ||
                ESP8266_ResponseContains("ALREADY CONNECT") ||
                ESP8266_ResponseContains("ALREAY CONNECT"))
            {
                ESP8266_SendAsyncCommand("AT+CIPMODE=1",
                                         ESP_CONNECT_CIPMODE_WAIT,
                                         1500U);
            }
            else if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_ScheduleRetry();
            }
            break;

        case ESP_CONNECT_CIPMODE_WAIT:
            if (ESP8266_ResponseContains("OK"))
            {
                ESP8266_SendAsyncCommand("AT+CIPSEND",
                                         ESP_CONNECT_SEND_WAIT,
                                         1500U);
            }
            else if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_ScheduleRetry();
            }
            break;

        case ESP_CONNECT_SEND_WAIT:
            if (ESP8266_ResponseContains(">") ||
                ESP8266_ResponseContains("OK"))
            {
                ESP8266_ResetResponse();
                s_connected = 1U;
                s_connect_state = ESP_CONNECT_READY;
                printf("[WiFi] Connected\r\n");
            }
            else if (ESP8266_TimeReached(s_connect_deadline))
            {
                ESP8266_ScheduleRetry();
            }
            break;

        case ESP_CONNECT_RETRY_WAIT:
            if (ESP8266_TimeReached(s_connect_deadline))
            {
                s_connect_state = ESP_CONNECT_BOOT_WAIT;
                s_connect_deadline = HAL_GetTick() + 100U;
            }
            break;

        default:
            ESP8266_ScheduleRetry();
            break;
    }
}

// Compatibility entry point: start connecting without blocking.
void ESP8266_ConnectToServer(void)
{
	ESP8266_ConnectStart();
}
//
void ESP8266_STA_TCPClient_Test(void)
{
    uint8_t res;
    char str[100]={0};
		printf("***************Restore the factory default mode***************\r\n");
    ESP8266_AT_Test();	//恢复出厂默认模式
		printf("***************The TCP mode is being configured***************\r\n");
    ESP8266_Net_Mode_Choose(STA);
    while(!ESP8266_JoinAP(User_ESP8266_SSID, User_ESP8266_PWD));
    ESP8266_Enable_MultipleId ( DISABLE );
    while(!ESP8266_Link_Server(enumTCP, User_ESP8266_TCPServer_IP, User_ESP8266_TCPServer_PORT, Single_ID_0));
	printf("***************The TCP mode is configured. Sending data...***************\r\n");
    while ( 1 )
    {
        sprintf (str,"123456" );                              //要发送的数据
        ESP8266_SendString ( DISABLE, str, strlen(str), Single_ID_0 );  //普通模式发送
        HAL_Delay(1000);
        if(TcpClosedFlag) //判断是否失去连接
        {
            printf("[INFO] TCP disconnected, reconnecting...\r\n");
            do
            {
                res = ESP8266_Get_LinkStatus();	//获取连接状态
            }
            while(!res);

            if(res == 4)	//确认失去连接，重连
            {
                while (!ESP8266_JoinAP(User_ESP8266_SSID, User_ESP8266_PWD ) );
                while (!ESP8266_Link_Server(enumTCP, User_ESP8266_TCPServer_IP, User_ESP8266_TCPServer_PORT, Single_ID_0 ) );
            }
            TcpClosedFlag = 0;    //清除断线标志
            printf("[INFO] Reconnected successfully\r\n");
        }
    }
}
//
