/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/app_thread.h
  * Description        : Header for Thread Application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_THREAD_H
#define APP_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include "tl.h"
#include "stm32wbxx_core_interface_def.h"
#include "tl_thread_hci.h"

/* OpenThread Library */
#include OPENTHREAD_CONFIG_FILE

/* USER CODE BEGIN Includes */

#define C_NODE_CAL_RESSOURCE					"nodeCal"
#define C_NODE_SPOT_RESSOURCE					"nodeSpot"
#define C_NODE_INFO_RESSOURCE					"nodeInfo"


extern const char nodeCalResource[15];
extern const char nodeSpotResource[15];

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

/* Thread application generic defines */
/*------------------------------------*/
typedef enum
{
  APP_THREAD_LIMITED,
  APP_THREAD_FULL,
} APP_THREAD_InitMode_t;

/* ipv6-addressing defines        */
/*------------------------------------*/
/* Key Point: A major difference between FTDs and MTDs are that FTDs subscribe to the ff03::2 multicast address.
 * MTDs do not. */

#define MULICAST_FTD_MED            "ff03::1"
#define MULICAST_FTD_BORDER_ROUTER  "ff03::2"

/* Application errors                 */
/*------------------------------------*/

/*
 *  List of all errors tracked by the Thread application
 *  running on M4. Some of these errors may be fatal
 *  or just warnings
 */
typedef enum
{
  ERR_REC_MULTI_MSG_FROM_M0,
  ERR_THREAD_SET_STATE_CB,
  ERR_THREAD_SET_CHANNEL,
  ERR_THREAD_SET_PANID,
  ERR_THREAD_IPV6_ENABLE,
  ERR_THREAD_START,
  ERR_THREAD_ERASE_PERSISTENT_INFO,
/* USER CODE BEGIN ERROR_APPLI_ENUM */

/* USER CODE END ERROR_APPLI_ENUM */
  ERR_THREAD_CHECK_WIRELESS
  } ErrAppliIdEnum_t;
/* USER CODE BEGIN ET */
#define REQUEST_ACK	1
#define NO_ACK		0

struct SystemCal
{
	otIp6Address 	ipv6;
	time_t 			epoch;
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern otIp6Address multicastAddr;

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void APP_THREAD_Init( void );
void APP_THREAD_Error(uint32_t ErrId, uint32_t ErrCode);
void APP_THREAD_RegisterCmdBuffer(TL_CmdPacket_t* p_buffer);
void APP_THREAD_ProcessMsgM0ToM4(void);
void APP_THREAD_Init_UART_CLI(void);
void APP_THREAD_TL_THREAD_INIT(void);
/* USER CODE BEGIN EF */
void APP_THREAD_SendCoapMsg(void *message, uint16_t msgSize, otIp6Address *ipv6_addr, const char *resource,
		uint8_t request_ack, otCoapCode coapCode, uint8_t msgID);
/* USER CODE END EF */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* APP_THREAD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
