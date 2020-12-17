/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : App/app_thread.c
  * Description        : Thread Application.
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

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "utilities_common.h"
#include "app_entry.h"
#include "dbg_trace.h"
#include "app_thread.h"
#include "stm32wbxx_core_interface_def.h"
#include "openthread_api_wb.h"
#include "shci.h"
#include "stm_logging.h"
#include "app_conf.h"
#include "stm32_lpm.h"
#include "cmsis_os.h"
#if (CFG_USB_INTERFACE_ENABLE != 0)
#include "vcp.h"
#include "vcp_conf.h"
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "spotlight_config.h"
//#include "calibration.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define C_SIZE_CMD_STRING       256U
#define C_PANID                 0x1234U
#define C_CHANNEL_NB            23U

/* FreeRtos stacks attributes */
const osThreadAttr_t ThreadMsgM0ToM4Process_attr = {
    .name = CFG_THREAD_MSG_M0_TO_M4_PROCESS_NAME,
    .attr_bits = CFG_THREAD_MSG_M0_TO_M4_PROCESS_ATTR_BITS,
    .cb_mem = CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_MEM,
    .cb_size = CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_SIZE,
    .stack_mem = CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_MEM,
    .priority = CFG_THREAD_MSG_M0_TO_M4_PROCESS_PRIORITY,
    .stack_size = CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_SIZE
};

const osThreadAttr_t ThreadCliProcess_attr = {
     .name = CFG_THREAD_CLI_PROCESS_NAME,
     .attr_bits = CFG_THREAD_CLI_PROCESS_ATTR_BITS,
     .cb_mem = CFG_THREAD_CLI_PROCESS_CB_MEM,
     .cb_size = CFG_THREAD_CLI_PROCESS_CB_SIZE,
     .stack_mem = CFG_THREAD_CLI_PROCESS_STACK_MEM,
     .priority = CFG_THREAD_CLI_PROCESS_PRIORITY,
     .stack_size = CFG_THREAD_CLI_PROCESS_STACK_SIZE
 };

/* USER CODE BEGIN PD */


/* RADIO SPECIFIC */
#define TRANSMIT_POWER							6 //in dbm
#define CHILD_SUPERVISION_INTERVAL				2 // default is 129 (how often a router broadcasts to its child to ensure its alive)
#define CHILD_SUPERVISION_TIMEOUT				3	// default is 190 (when child trying to find a new router)

#define NODE_DESCRIPTION  		"1"	  // max 11 bytes
#define NODE_TYPE				"Spot"
/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
static void APP_THREAD_CheckWirelessFirmwareInfo(void);
static void APP_THREAD_DeviceConfig(void);
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext);
static void APP_THREAD_TraceError(const char * pMess, uint32_t ErrCode);
#if (CFG_FULL_LOW_POWER == 0)
static void Send_CLI_To_M0(void);
#endif /* (CFG_FULL_LOW_POWER == 0) */
static void Send_CLI_Ack_For_OT(void);
static void HostTxCb( void );
static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);
#if (CFG_HW_LPUART1_ENABLED == 1)
extern void MX_LPUART1_UART_Init(void);
#endif
#if (CFG_HW_USART1_ENABLED == 1)
extern void MX_USART1_UART_Init(void);
#endif
#if (CFG_USB_INTERFACE_ENABLE != 0)
static uint32_t ProcessCmdString(uint8_t* buf , uint32_t len);
#else
#if (CFG_FULL_LOW_POWER == 0)
static void RxCpltCallback(void);
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* FreeRTos wrapper functions */
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument);
#if (CFG_FULL_LOW_POWER == 0)
static void APP_THREAD_FreeRTOSSendCLIToM0Task(void *argument);
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN PFP */
static void APP_THREAD_CoapNodeCalRequestHandler(void                * pContext,
													otCoapHeader        * pHeader,
													otMessage           * pMessage,
													const otMessageInfo * pMessageInfo);
static void APP_THREAD_CoapNodeSpotRequestHandler(void                * pContext,
													otCoapHeader        * pHeader,
													otMessage           * pMessage,
													const otMessageInfo * pMessageInfo);
//static void APP_THREAD_CoapNodeInfoRequestHandler(otCoapHeader *pHeader, otMessage *pMessage,
//		const otMessageInfo *pMessageInfo);
static void APP_THREAD_DummyReqHandler(void *p_context, otCoapHeader *pHeader, otMessage *pMessage,
		const otMessageInfo *pMessageInfo);
static void APP_THREAD_CoapSendDataResponse(otCoapHeader    * pRequestHeader,
    const otMessageInfo * pMessageInfo);
static void APP_THREAD_SendCoapUnicastRequest(char *message, uint8_t message_length, char *ipv6_addr, char *resource);

/* USER CODE END PFP */

/* Private variables -----------------------------------------------*/
#if (CFG_USB_INTERFACE_ENABLE != 0)
static uint8_t TmpString[C_SIZE_CMD_STRING];
static uint8_t VcpRxBuffer[sizeof(TL_CmdSerial_t)];        /* Received Data over USB are stored in this buffer */
static uint8_t VcpTxBuffer[sizeof(TL_EvtPacket_t) + 254U]; /* Transmit buffer over USB */
#else
#if (CFG_FULL_LOW_POWER == 0)
static uint8_t aRxBuffer[C_SIZE_CMD_STRING];
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_FULL_LOW_POWER == 0)
static uint8_t CommandString[C_SIZE_CMD_STRING];
#endif /* (CFG_FULL_LOW_POWER == 0) */
static __IO uint16_t indexReceiveChar = 0;
static __IO uint16_t CptReceiveCmdFromUser = 0;

static TL_CmdPacket_t *p_thread_otcmdbuffer;
static TL_EvtPacket_t *p_thread_notif_M0_to_M4;
static __IO uint32_t  CptReceiveMsgFromM0 = 0;
static volatile int FlagReceiveAckFromM0 = 0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_TH_Config_t ThreadConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ThreadNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadCliCmdBuffer;

static osThreadId_t OsTaskMsgM0ToM4Id;      /* Task managing the M0 to M4 messaging        */
#if (CFG_FULL_LOW_POWER == 0)
static osThreadId_t OsTaskCliId;            /* Task used to manage CLI comamnd             */
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN PV */
//static otCoapResource OT_Lights_Complex_Ressource = { C_LIGHTS_SIMPLE_RESSOURCE, APP_THREAD_DummyReqHandler,
//		(void*) APP_THREAD_CoapLightsSimpleRequestHandler, NULL };
//static otCoapResource OT_Lights_Simple_Ressource = { C_LIGHTS_COMPLEX_RESSOURCE, APP_THREAD_DummyReqHandler,
//		(void*) APP_THREAD_CoapLightsComplexRequestHandler, NULL };
//static otCoapResource OT_Border_Time_Ressource = { C_BORER_TIME_RESSOURCE, APP_THREAD_DummyReqHandler,
//		(void*) APP_THREAD_CoapBorderTimeRequestHandler, NULL };
//static otCoapResource OT_Node_Info_Ressource = { C_NODE_INFO_RESSOURCE, APP_THREAD_DummyReqHandler,
//		(void*) APP_THREAD_CoapNodeInfoRequestHandler, NULL };
//static otCoapResource OT_Node_Cal_Ressource = { C_NODE_CAL_RESSOURCE, APP_THREAD_DummyReqHandler,
//		(void*) APP_THREAD_CoapNodeCalRequestHandler, NULL };

static otCoapResource OT_Node_Cal_Ressource = {C_NODE_CAL_RESSOURCE, APP_THREAD_CoapNodeCalRequestHandler,"myCal", NULL};
static otCoapResource OT_Node_Spot_Ressource = {C_NODE_SPOT_RESSOURCE, APP_THREAD_CoapNodeSpotRequestHandler,"mySpot", NULL};




//static otCoapResource OT_Node_Info_Ressource = {C_NODE_INFO_RESSOURCE, APP_THREAD_CoapNodeInfoRequestHandler,"myInfo", NULL};

const otMasterKey masterKey = { 0x33, 0x33, 0x44, 0x44, 0x33, 0x33, 0x44, 0x44, 0x33, 0x33, 0x44, 0x44, 0x33, 0x33,
		0x44, 0x44 };
const otExtendedPanId extendedPanId = { 0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22 };
const char networkName[20] = "PatrickNetwork";
const char nodeInfoResource[15] = "nodeInfo";
const char nodeCalResource[15] = "nodeCal";
const char nodeSpotResource[15] = "nodeSpot";

static otMessage *pOT_Message = NULL;

struct sendIP_struct {
	char node_type[12];
	char description[12];
	uint8_t uid[8];
	uint16_t RLOC;
};

otError error = OT_ERROR_NONE;
static uint8_t OT_ReceivedCommand = 0;
static char OT_BufferSend[20] = "TEST PAYLOAD";
volatile otIp6Address *myAddress;
volatile otMessageInfo *tempMessageInfo;
volatile uint16_t myRloc16;
volatile otNetifAddress *unicastAddresses;

volatile bool isEnabledIpv6;
//volatile otNetifMulticastAddress * multicastAddresses;
//volatile otIp6Address *  meshLocalEID;
//volatile otIp6Address * linkLocalIPV6;
volatile otMessageInfo *receivedMessage;

//struct LogMessage logMessage;

struct SystemCal borderRouter = { 0 };
struct SystemCal receivedSystemCal = { 0 };

struct sendIP_struct msgSendMyIP = { .node_type = NODE_TYPE, .description = NODE_DESCRIPTION, .RLOC = 0, .uid = 0 };

otIp6Address multicastAddr;

//otNetifMulticastAddress* multicastAddresses;
//otIp6Address* meshLocalEID;
//otIp6Address* linkLocalIPV6;

volatile otNetifMulticastAddress* multicastAddresses;
volatile otIp6Address* meshLocalEID;
volatile otIp6Address* linkLocalIPV6;

otMessageInfo OT_MessageInfo = { 0 };
otCoapHeader OT_Header = { 0 };
/* USER CODE END PV */

/* Functions Definition ------------------------------------------------------*/

void APP_THREAD_Init( void )
{
  /* USER CODE BEGIN APP_THREAD_INIT_1 */

  /* USER CODE END APP_THREAD_INIT_1 */

  SHCI_CmdStatus_t ThreadInitStatus;

  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
  APP_THREAD_CheckWirelessFirmwareInfo();

#if (CFG_USB_INTERFACE_ENABLE != 0)
  VCP_Init(&VcpTxBuffer[0], &VcpRxBuffer[0]);
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

  /* Register cmdbuffer */
  APP_THREAD_RegisterCmdBuffer(&ThreadOtCmdBuffer);

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_THREAD, UTIL_LPM_DISABLE);

  /* Init config buffer and call TL_THREAD_Init */
  APP_THREAD_TL_THREAD_INIT();

  /* Configure UART for sending CLI command from M4 */
  APP_THREAD_Init_UART_CLI();

  /* Send Thread start system cmd to M0 */
  ThreadInitStatus = SHCI_C2_THREAD_Init();

  /* Prevent unused argument(s) compilation warning */
  UNUSED(ThreadInitStatus);

  /* USER CODE BEGIN APP_THREAD_INIT_TIMER */

  /* USER CODE END APP_THREAD_INIT_TIMER */

  /* Create the different FreeRTOS tasks requested to run this Thread application*/
  OsTaskMsgM0ToM4Id = osThreadNew(APP_THREAD_FreeRTOSProcessMsgM0ToM4Task, NULL,&ThreadMsgM0ToM4Process_attr);

  /* USER CODE BEGIN APP_THREAD_INIT_FREERTOS */

  /* USER CODE END APP_THREAD_INIT_FREERTOS */

  /* Configure the Thread device at start */
  APP_THREAD_DeviceConfig();

  /* USER CODE BEGIN APP_THREAD_INIT_2 */

  /* USER CODE END APP_THREAD_INIT_2 */
}

/**
  * @brief  Trace the error or the warning reported.
  * @param  ErrId :
  * @param  ErrCode
  * @retval None
  */
void APP_THREAD_Error(uint32_t ErrId, uint32_t ErrCode)
{
  /* USER CODE BEGIN APP_THREAD_Error_1 */

  /* USER CODE END APP_THREAD_Error_1 */
  switch(ErrId)
  {
  case ERR_REC_MULTI_MSG_FROM_M0 :
    APP_THREAD_TraceError("ERROR : ERR_REC_MULTI_MSG_FROM_M0 ", ErrCode);
    break;
  case ERR_THREAD_SET_STATE_CB :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_STATE_CB ",ErrCode);
    break;
  case ERR_THREAD_SET_CHANNEL :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_CHANNEL ",ErrCode);
    break;
  case ERR_THREAD_SET_PANID :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_PANID ",ErrCode);
    break;
  case ERR_THREAD_IPV6_ENABLE :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_IPV6_ENABLE ",ErrCode);
    break;
  case ERR_THREAD_START :
    APP_THREAD_TraceError("ERROR: ERR_THREAD_START ", ErrCode);
    break;
  case ERR_THREAD_ERASE_PERSISTENT_INFO :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_ERASE_PERSISTENT_INFO ",ErrCode);
    break;
  case ERR_THREAD_CHECK_WIRELESS :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_CHECK_WIRELESS ",ErrCode);
    break;
  /* USER CODE BEGIN APP_THREAD_Error_2 */

  /* USER CODE END APP_THREAD_Error_2 */
  default :
    APP_THREAD_TraceError("ERROR Unknown ", 0);
    break;
  }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief Thread initialization.
 * @param  None
 * @retval None
 */
static void APP_THREAD_DeviceConfig(void)
{
  otError error;
  error = otInstanceErasePersistentInfo(NULL);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_ERASE_PERSISTENT_INFO,error);
  }
  otInstanceFinalize(NULL);
  otInstanceInitSingle();
  error = otSetStateChangedCallback(NULL, APP_THREAD_StateNotif, NULL);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_STATE_CB,error);
  }
  error = otLinkSetChannel(NULL, C_CHANNEL_NB);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_CHANNEL,error);
  }
  error = otLinkSetPanId(NULL, C_PANID);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_PANID,error);
  }
  error = otIp6SetEnabled(NULL, true);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_IPV6_ENABLE,error);
  }
  error = otThreadSetEnabled(NULL, true);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_START,error);
  }

  /* USER CODE BEGIN DEVICECONFIG */
  error = otThreadSetEnabled(NULL, false);
  	if (error != OT_ERROR_NONE) {
  		APP_THREAD_Error(ERR_THREAD_START, error);
  	}


  	error = otPlatRadioSetTransmitPower(NULL, TRANSMIT_POWER);
  	if (error != OT_ERROR_NONE) {
  		APP_THREAD_Error(ERR_THREAD_SET_CHANNEL, error);
  	}

  	otThreadSetChildTimeout(NULL, CHILD_SUPERVISION_TIMEOUT);

  //	otChildSupervisionSetCheckTimeout(NULL, CHILD_SUPERVISION_TIMEOUT);
  //	otChildSupervisionSetInterval(NULL, CHILD_SUPERVISION_INTERVAL);

  //   error = otIp6AddressFromString("ff12::1", &multicastAddr);
  //   error = otIp6SubscribeMulticastAddress(NULL, &multicastAddr);

//  	error = otThreadSetMasterKey(NULL, &masterKey);
//  	if (error != OT_ERROR_NONE) {
//  		APP_THREAD_Error(ERR_THREAD_SET_CHANNEL, error);
//  	}

  	error = otThreadSetNetworkName(NULL, networkName);
  	if (error != OT_ERROR_NONE) {
  		APP_THREAD_Error(ERR_THREAD_SET_CHANNEL, error);
  	}
//  	error = otThreadSetExtendedPanId(NULL, &extendedPanId);
//  	if (error != OT_ERROR_NONE) {
//  		APP_THREAD_Error(ERR_THREAD_SET_CHANNEL, error);
//  	}

  	error = otIp6SetEnabled(NULL, true);
  	if (error != OT_ERROR_NONE) {
  		APP_THREAD_Error(ERR_THREAD_IPV6_ENABLE, error);
  	}

  	error = otThreadSetEnabled(NULL, true);
  	if (error != OT_ERROR_NONE) {
  		APP_THREAD_Error(ERR_THREAD_START, error);
  	}

  	error = otCoapStart(NULL, OT_DEFAULT_COAP_PORT);
  	//  error = otCoapAddResource(NULL, &OT_Light_Ressource);
//  	error = otCoapAddResource(NULL, &OT_Lights_Complex_Ressource);
//  	error = otCoapAddResource(NULL, &OT_Lights_Simple_Ressource);
//  	error = otCoapAddResource(NULL, &OT_Border_Time_Ressource);
//  	error = otCoapAddResource(NULL, &OT_Node_Info_Ressource);

  	error = otCoapAddResource(NULL, &OT_Node_Cal_Ressource);
  	error = otCoapAddResource(NULL, &OT_Node_Spot_Ressource);

  #ifdef BORDER_ROUTER_NODE
  	error = otCoapAddResource(NULL, &OT_Border_Log_Ressource);
  #endif
  #ifdef OTA_ENABLED
  	  error = otCoapAddResource(NULL, &OT_RessourceFuotaProvisioning);
  	  error = otCoapAddResource(NULL, &OT_RessourceFuotaParameters);
  	  error = otCoapAddResource(NULL, &OT_RessourceFuotaSend);
  #endif
  #ifndef DONGLE_CODE
//      error = otCoapAddResource(NULL, &OT_Toggle_Logging_Ressource);
  #endif
  	// set default multicast address for border router
  //    otIp6AddressFromString("ff03::1", &borderRouter.ipv6);
  	otIp6AddressFromString("ff03::1", &multicastAddr);
//  	memcpy(&borderRouter.ipv6, &multicastAddr, sizeof(multicastAddr));

  	// set UID in local state variable
  //	msgSendMyIP.uid = (uint32_t)  DBGMCU->IDCODE;
  //	 = UID64_BASE;
//  	stm32UID(msgSendMyIP.uid);
  /* USER CODE END DEVICECONFIG */
}

/**
 * @brief Thread notification when the state changes.
 * @param  aFlags  : Define the item that has been modified
 *         aContext: Context
 *
 * @retval None
 */
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pContext);

  /* USER CODE BEGIN APP_THREAD_STATENOTIF */

  /* USER CODE END APP_THREAD_STATENOTIF */

  if ((NotifFlags & (uint32_t)OT_CHANGED_THREAD_ROLE) == (uint32_t)OT_CHANGED_THREAD_ROLE)
  {
    switch (otThreadGetDeviceRole(NULL))
    {
    case OT_DEVICE_ROLE_DISABLED:
      /* USER CODE BEGIN OT_DEVICE_ROLE_DISABLED */

      /* USER CODE END OT_DEVICE_ROLE_DISABLED */
      break;
    case OT_DEVICE_ROLE_DETACHED:
      /* USER CODE BEGIN OT_DEVICE_ROLE_DETACHED */

      /* USER CODE END OT_DEVICE_ROLE_DETACHED */
      break;
    case OT_DEVICE_ROLE_CHILD:
      /* USER CODE BEGIN OT_DEVICE_ROLE_CHILD */

      /* USER CODE END OT_DEVICE_ROLE_CHILD */
      break;
    case OT_DEVICE_ROLE_ROUTER :
      /* USER CODE BEGIN OT_DEVICE_ROLE_ROUTER */

      /* USER CODE END OT_DEVICE_ROLE_ROUTER */
      break;
    case OT_DEVICE_ROLE_LEADER :
      /* USER CODE BEGIN OT_DEVICE_ROLE_LEADER */

      /* USER CODE END OT_DEVICE_ROLE_LEADER */
      break;
    default:
      /* USER CODE BEGIN DEFAULT */

      /* USER CODE END DEFAULT */
      break;
    }
  }
}

/**
  * @brief  Warn the user that an error has occurred.In this case,
  *         the LEDs on the Board will start blinking.
  *
  * @param  pMess  : Message associated to the error.
  * @param  ErrCode: Error code associated to the module (OpenThread or other module if any)
  * @retval None
  */
static void APP_THREAD_TraceError(const char * pMess, uint32_t ErrCode)
{
  /* USER CODE BEGIN TRACE_ERROR */

  /* USER CODE END TRACE_ERROR */
}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Thread
 *        and display associated informations
 * @param  None
 * @retval None
 */
static void APP_THREAD_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t* p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
  {
    APP_THREAD_Error((uint32_t)ERR_THREAD_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
  }
  else
  {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch(p_wireless_info->StackType)
    {
    case INFO_STACK_TYPE_THREAD_FTD :
      APP_DBG("FW Type : Thread FTD");
      break;
    case INFO_STACK_TYPE_THREAD_MTD :
      APP_DBG("FW Type : Thread MTD");
      break;
    case INFO_STACK_TYPE_BLE_THREAD_FTD_STATIC :
      APP_DBG("FW Type : Static Concurrent Mode BLE/Thread");
      break;
    default :
      /* No Thread device supported ! */
      APP_THREAD_Error((uint32_t)ERR_THREAD_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
      break;
    }
    APP_DBG("**********************************************************");
  }
}

/*************************************************************
 *
 * FREERTOS WRAPPER FUNCTIONS
 *
*************************************************************/
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    /* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */

    /* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */
    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
    APP_THREAD_ProcessMsgM0ToM4();
    /* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_2 */

    /* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_2 */
  }
}

#if (CFG_FULL_LOW_POWER == 0)
static void APP_THREAD_FreeRTOSSendCLIToM0Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    /* USER CODE BEGIN APP_THREAD_FREERTOS_SEND_CLI_TO_M0_1 */

    /* USER END END APP_THREAD_FREERTOS_SEND_CLI_TO_M0_1 */
    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
    Send_CLI_To_M0();
    /* USER CODE BEGIN APP_THREAD_FREERTOS_SEND_CLI_TO_M0_2 */

    /* USER END END APP_THREAD_FREERTOS_SEND_CLI_TO_M0_2 */
  }
}
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN FREERTOS_WRAPPER_FUNCTIONS */

/* USER CODE END FREERTOS_WRAPPER_FUNCTIONS */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

//static void APP_THREAD_DummyReqHandler(void *p_context, otCoapHeader *pHeader, otMessage *pMessage,
//		const otMessageInfo *pMessageInfo) {
//	tempMessageInfo = pMessageInfo;
//	receivedMessage = (otMessageInfo*) pMessage;
//}

static void APP_THREAD_CoapNodeSpotRequestHandler(void                * pContext,
													otCoapHeader        * pHeader,
													otMessage           * pMessage,
													const otMessageInfo * pMessageInfo) {
#ifdef DONGLE_CODE
		BSP_LED_Toggle(LED_RED);
#endif
		struct MeasMsg receivedMeasMsg;
		if (otMessageRead(pMessage, otMessageGetOffset(pMessage), &receivedMeasMsg, sizeof(receivedMeasMsg))
				== sizeof(receivedMeasMsg)) {


			if (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_PUT)
			{
				measMsgReceivedFromNode(&receivedMeasMsg);
				toggleLed(0,0,1);
			}

			if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE)
			{
				APP_THREAD_CoapSendDataResponse(pHeader, pMessageInfo);
			}

		}
}

struct CalMsg receivedCalMsg;
static void APP_THREAD_CoapNodeCalRequestHandler(void                * pContext,
													otCoapHeader        * pHeader,
													otMessage           * pMessage,
													const otMessageInfo * pMessageInfo) {
#ifdef DONGLE_CODE
		BSP_LED_Toggle(LED_RED);
#endif

		if (otMessageRead(pMessage, otMessageGetOffset(pMessage), &receivedCalMsg, sizeof(receivedCalMsg))
				== sizeof(receivedCalMsg)) {

			if (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_PUT)
			{
				toggleLed(1,1,1);
				if(receivedCalMsg.cal_fcn == CALIBRATION_START_COMMAND){
					// TODO: start calibration

				}else if(receivedCalMsg.cal_fcn == CALIBRATION_STOP_COMMAND){
					// TODO: stop calibration
				}
				toggleLed(0,0,1);
			}

			if (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_GET)
			{
				toggleLed(1,1,1);
				if(receivedCalMsg.cal_fcn == CALIBRATION_GET_MEAS){
					// send measurement to Spotlight
					// 		TODO this can be done in an ACK but doing it now in a separate PUT request to the nodeSpot resource
					sendPowerMeasurement(pMessageInfo->mPeerAddr, receivedCalMsg.angle_1, receivedCalMsg.angle_2);
				}
				toggleLed(1,0,0);
			}

			if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE)
			{
				APP_THREAD_CoapSendDataResponse(pHeader, pMessageInfo);
			}

		}
//
////		receivedMessage = (otMessageInfo*) pMessage;
//		otMessageRead(pMessage, otMessageGetOffset(pMessage), tempVar, sizeof(tempVar));
//
//		// send info if requested
//		testCode = otCoapHeaderGetCode(pHeader);
//		testType = otCoapHeaderGetType(pHeader);
//
//		if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_NON_CONFIRMABLE)
//		{
//			toggleLed(1,0,0);
//		}
//
//		if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_ACKNOWLEDGMENT)
//				{
//					toggleLed(0,1,0);
//				}
//
////		    if (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_PUT)
////		    {
////		      break;
////		    }
//
//		if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE)
//		{
//			toggleLed(0,0,1);
//		}

//		if (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_GET) {
////			APP_THREAD_SendDataResponse(&borderRouter, sizeof(borderRouter), pHeader, pMessageInfo);
////			APP_THREAD_SendDataResponse(&msgSendMyIP, sizeof(msgSendMyIP), pHeader, pMessageInfo);
////			APP_THREAD_SendMyInfo();
//
//			break;
//		}
//
//		if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE) {
//			APP_THREAD_SendDataResponse(NULL, 0, pHeader, pMessageInfo);
//		}

}

// Only get requests allowed for this resource
//struct sendIP_struct tempVar = {"test","test",0};
//char test_string[200] = "test";
//static void APP_THREAD_CoapNodeInfoRequestHandler(otCoapHeader *pHeader, otMessage *pMessage,
//		const otMessageInfo *pMessageInfo) {
//	do {
//#ifdef DONGLE_CODE
//		BSP_LED_Toggle(LED_RED);
//#endif
//
////		if (otMessageRead(pMessage, otMessageGetOffset(pMessage), &tempVar, sizeof(tempVar))
////				== sizeof(tempVar)) {
////			// if the message was a put request, copy message over to border router info struct
//////			if ((otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_PUT)
//////					|| (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_POST)) {
//////
//////				memcpy(&borderRouter, &receivedSystemCal, sizeof(receivedSystemCal));
//////
//////				// update the onboard RTC unix time
//////				updateRTC(borderRouter.epoch);
//////			}
////		}
//
////		receivedMessage = (otMessageInfo*) pMessage;
//
//		// send info if requested
//		if (otCoapHeaderGetCode(pHeader) == OT_COAP_CODE_GET) {
////			APP_THREAD_SendDataResponse(&borderRouter, sizeof(borderRouter), pHeader, pMessageInfo);
////			APP_THREAD_SendDataResponse(&msgSendMyIP, sizeof(msgSendMyIP), pHeader, pMessageInfo);
//			APP_THREAD_SendMyInfo();
//
//			break;
//		}
//
//		if (otCoapHeaderGetType(pHeader) == OT_COAP_TYPE_CONFIRMABLE) {
//			APP_THREAD_SendDataResponse(NULL, 0, pHeader, pMessageInfo);
//		}
//
//	} while (false);
//}

void APP_THREAD_SendMyInfo() {
//	msgSendMyIP.uid = DBGMCU->IDCODE;
//	const otIp6Address *test_ip = otThreadGetLinkLocalIp6Address(NULL);
//	if(test_ip) random_var++;
//	state = otIp6IsMulticastPromiscuousEnabled(NULL);
//	test_addr = otIp6GetMulticastAddresses(NULL);
//	test_ext_addr = otLinkGetExtendedAddress(NULL);
//	memcpy(&test_1, test_ext_addr, sizeof(otExtAddress));
////	test_1 = test_ext_addr[0];
////	test_2 = &test_ext_addr;
//
//	error = otThreadGetNextNeighborInfo(NULL, &test_neighbor_iterator, &test_info_neighbor);
	// TODO: does this need an ACK
	APP_THREAD_SendCoapMsg(&msgSendMyIP, sizeof(msgSendMyIP), &borderRouter.ipv6, nodeInfoResource, NO_ACK,
			OT_COAP_CODE_PUT, 1U);
//	APP_THREAD_SendCoapUnicastMsg(NULL, NULL, borderRouter.ipv6  , borderSyncResource, 1U);
}

/**
 * @brief This function acknowledges the data reception by sending an ACK
 *    back to the sender.
 * @param  pRequestHeader coap header
 * @param  pMessageInfo message info pointer
 * @retval None
 */
static void APP_THREAD_CoapSendDataResponse(otCoapHeader    * pRequestHeader,
    const otMessageInfo * pMessageInfo)
{
  otError  error = OT_ERROR_NONE;

  do{
    APP_DBG(" ********* APP_THREAD_CoapSendDataResponse ********* ");
    otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
    otCoapHeaderSetMessageId(&OT_Header, otCoapHeaderGetMessageId(pRequestHeader));
    otCoapHeaderSetToken(&OT_Header,
        otCoapHeaderGetToken(pRequestHeader),
        otCoapHeaderGetTokenLength(pRequestHeader));

    pOT_Message = otCoapNewMessage(NULL, &OT_Header);
    if (pOT_Message == NULL)
    {
      APP_DBG("WARNING : pOT_Message = NULL ! -> exit now");
      break;
    }
    error = otCoapSendResponse(NULL, pOT_Message, pMessageInfo);
    if (error != OT_ERROR_NONE && pOT_Message != NULL)
    {
      otMessageFree(pOT_Message);
//      APP_THREAD_Error(ERR_THREAD_COAP_DATA_RESPONSE,error);
    }
  }while(false);
}


/**
 * @brief This function acknowledge the data reception by sending an ACK
 *    back to the sender.
 * @param  pRequestHeader coap header
 * @param  pMessageInfo message info pointer
 * @retval None
 */

/*From RFC:  In a piggybacked response, the Message ID of the Confirmable
request and the Acknowledgement MUST match, and the tokens of the
response and original request MUST match.  In a separate
response, just the tokens of the response and original request
MUST match.*/

//static void APP_THREAD_SendDataResponse(void *message, uint16_t msgSize, otCoapHeader *pRequestHeader, const otMessageInfo *pMessageInfo) {
//	otError error = OT_ERROR_NONE;
//
//	//APP_DBG(" ********* APP_THREAD_SendDataResponse \r\n");
//	otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
//	otCoapHeaderSetMessageId(&OT_Header, otCoapHeaderGetMessageId(pRequestHeader));
//	otCoapHeaderSetToken(&OT_Header, otCoapHeaderGetToken(pRequestHeader), otCoapHeaderGetTokenLength(pRequestHeader));
//
//	if (msgSize > 0){
//		// need this so the coap server doesnt try to parse as 'utf-8' and error out
//		otCoapHeaderAppendContentFormatOption(&OT_Header, OT_COAP_OPTION_CONTENT_FORMAT_OCTET_STREAM);
////			  if (error != OT_ERROR_NONE) while(1);
//
//		// This function adds Payload Marker indicating beginning of the payload to the CoAP header
//		otCoapHeaderSetPayloadMarker(&OT_Header); //TODO: if no msg, dont set marker and remove empty message below
//	}
//
//	pOT_Message = otCoapNewMessage(NULL, &OT_Header);
//	if (pOT_Message == NULL) {
//		//APP_THREAD_Error(ERR_NEW_MSG_ALLOC,error);
//	}
//
//	// append message if there was one given
//	if (msgSize > 0) {
//		error = otMessageAppend(pOT_Message, message, msgSize);
//	}
//
//	error = otCoapSendResponse(NULL, pOT_Message, pMessageInfo);
//	if (error != OT_ERROR_NONE && pOT_Message != NULL) {
//		otMessageFree(pOT_Message);
//		//APP_THREAD_Error(ERR_THREAD_DATA_RESPONSE,error);
//	}
//}

void APP_THREAD_SendCoapMsg(void *message, uint16_t msgSize, otIp6Address *ipv6_addr, const char *resource,
		uint8_t request_ack, otCoapCode coapCode, uint8_t msgID) {
	/************ SET MESSAGE INFO (WHERE THE PACKET GOES) ************/
	// https://openthread.io/reference/struct/ot-message-info.html#structot_message_info
	do {
		// REMOVE BELOW CALLS (ONLY FOR DEBUGGING)
			  myRloc16 = otThreadGetRloc16(NULL);
			  unicastAddresses = otIp6GetUnicastAddresses(NULL);
			  isEnabledIpv6 = otIp6IsEnabled(NULL);
			  multicastAddresses = otIp6GetMulticastAddresses(NULL);
			  meshLocalEID =  otThreadGetMeshLocalEid(NULL);
			  linkLocalIPV6 = otThreadGetLinkLocalIp6Address(NULL);

		// clear info
		memset(&OT_MessageInfo, 0, sizeof(OT_MessageInfo));

		// add destination IPv6 address to header
		// TODO : swap the below statements once ST has their shit fixed

		if(msgSize > 100){ // TODO : semd to borderRouter if the message is a log message (this is a temporary fix)
#ifndef BORDER_ROUTER_NODE_TRANSMITTER
			memcpy(&OT_MessageInfo.mPeerAddr, &borderRouter.ipv6, sizeof(otIp6Address));
			memcpy(&OT_MessageInfo.mSockAddr, otThreadGetMeshLocalEid(NULL), sizeof(OT_MessageInfo.mSockAddr));
//			otIp6AddressFromString("fd11:22::c34c:7994:cccc:4b82", &OT_MessageInfo.mSockAddr);
#else
			memcpy(&OT_MessageInfo.mPeerAddr, &multicastAddr, sizeof(otIp6Address));
#endif
		}else{
			memcpy(&OT_MessageInfo.mPeerAddr, &multicastAddr, sizeof(otIp6Address));
			memcpy(&OT_MessageInfo.mSockAddr, otThreadGetMeshLocalEid(NULL), sizeof(OT_MessageInfo.mSockAddr));
		}

//			  memcpy(&OT_MessageInfo.mPeerAddr, ipv6_addr, sizeof(otIp6Address));

		// add source mesh-local IPv6 address to header
		// TODO: ST is supposed to address the issue in FW v1.5 but currently, the IP lookup function returns a null pointer
		// memcpy(&OT_MessageInfo.mSockAddr, otThreadGetMeshLocalEid(NULL), sizeof(OT_MessageInfo.mSockAddr));

		// REMOVE BELOW CALLS (ONLY FOR DEBUGGING)
		//			  otIp6AddressFromString("fd11:22::c55e:6732:c8cd:9443", &OT_MessageInfo.mPeerAddr);
//			  otIp6AddressFromString("fd11:1111:1122:0:7be1:2f12:9534:72d5", &OT_MessageInfo.mPeerAddr);
//			  			  otIp6AddressFromString("fd11:1111:1122:0:4faa:fd9:da06:9100", &OT_MessageInfo.mPeerAddr);
//			  			  otIp6AddressFromString("fd11:22:0:0:c34c:7994:19f2:4b82", &OT_MessageInfo.mPeerAddr);
//			   memcpy(&OT_MessageInfo.mSockAddr, otThreadGetMeshLocalEid(NULL), sizeof(OT_MessageInfo.mSockAddr));
//			  otIp6AddressFromString("fd11:22::c34c:7994:19f2:4b82", &OT_MessageInfo.mSockAddr);

//			  otIp6AddressFromString("fd11:22::c34c:7994:cccc:4b82", &OT_MessageInfo.mSockAddr);

		// populate message information
		OT_MessageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
		OT_MessageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
		OT_MessageInfo.mHopLimit = 64;

		/************** CREATE NEW MESSAGE ********************ifco*/

		// create header
		if (request_ack && (coapCode == OT_COAP_CODE_PUT))
			otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
		else if (request_ack && (coapCode == OT_COAP_CODE_GET))
			otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_GET);
		else if (request_ack && (coapCode == OT_COAP_CODE_POST))
			otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_POST);
		else if (!request_ack && (coapCode == OT_COAP_CODE_PUT))
			otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);
		else if (!request_ack && (coapCode == OT_COAP_CODE_GET))
			otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_GET);
		else if (!request_ack && (coapCode == OT_COAP_CODE_POST))
			otCoapHeaderInit(&OT_Header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
		else
			return // this return should never happen

//			  otCoapHeaderSetMessageId(&OT_Header, msgID); `//may not need since sendRequest should set to 0
			otCoapHeaderGenerateToken(&OT_Header, 2U); //This function sets the Token length and randomizes its value.

		// add the name of the resource
		error = otCoapHeaderAppendUriPathOptions(&OT_Header, resource);
//			  if (error != OT_ERROR_NONE) while(1);

		// need this so the coap server doesnt try to parse as 'utf-8' and error out
		otCoapHeaderAppendContentFormatOption(&OT_Header, OT_COAP_OPTION_CONTENT_FORMAT_OCTET_STREAM);
//			  if (error != OT_ERROR_NONE) while(1);

		// This function adds Payload Marker indicating beginning of the payload to the CoAP header
		if (msgSize > 0) {
			otCoapHeaderSetPayloadMarker(&OT_Header); //TODO: if no msg, dont set marker and remove empty message below
		}

		// creates new message with headers but with empty payload
		pOT_Message = otCoapNewMessage(NULL, &OT_Header);
//		if (pOT_Message == NULL)
//			while (1);

		// Append bytes to a message (this is where the payload gets added)

		// append message if there was one given
		if (msgSize > 0) {
			error = otMessageAppend(pOT_Message, message, msgSize);
		}
//		else{
//			error = otMessageAppend(pOT_Message, empty_message, 10);
//		}

//			  if (error != OT_ERROR_NONE) while(1);

		// TODO: the response function should only be used for the border update event (I think only if message is embedded in ACK)?


		error = otCoapSendRequest(NULL,
								pOT_Message,
								&OT_MessageInfo,
								NULL,
								(void*) NULL);


		// if error: free allocated message buffer if one was allocated
		if (error != OT_ERROR_NONE && pOT_Message != NULL) {
			otMessageFree(pOT_Message);
		}

	} while (false);
}
/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void APP_THREAD_RegisterCmdBuffer(TL_CmdPacket_t* p_buffer)
{
  p_thread_otcmdbuffer = p_buffer;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)p_thread_otcmdbuffer->cmdserial.cmd.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdRspPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)((TL_EvtPacket_t *)p_thread_otcmdbuffer)->evtserial.evt.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_NotificationPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)(p_thread_notif_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the Ot commands from the
 *         M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void Ot_Cmd_Transfer(void)
{
  /* OpenThread OT command cmdcode range 0x280 .. 0x3DF = 352 */
  p_thread_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  /* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
   * + ID (4 bytes) + Size (4 bytes) */
  uint32_t l_size = ((Thread_OT_Cmd_Request_t*)(p_thread_otcmdbuffer->cmdserial.cmd.payload))->Size * 4U + 8U;
  p_thread_otcmdbuffer->cmdserial.cmd.plen = l_size;

  TL_OT_SendCmd();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_M0();
}

/**
 * @brief  This function is called when acknowledge from OT command is received from the M0+.
 *
 * @param   Otbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_OT_CmdEvtReceived( TL_EvtPacket_t * Otbuffer )
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Otbuffer);

  Receive_Ack_From_M0();
}

/**
 * @brief  This function is called when notification from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_NotReceived( TL_EvtPacket_t * Notbuffer )
{
  p_thread_notif_M0_to_M4 = Notbuffer;

  Receive_Notification_From_M0();
}

/**
  * @brief  This function is called before sending any ot command to the M0
  *         core. The purpose of this function is to be able to check if
  *         there are no notifications coming from the M0 core which are
  *         pending before sending a new ot command.
  * @param  None
  * @retval None
  */
void Pre_OtCmdProcessing(void)
{

}

/**
  * @brief  This function waits for getting an acknowledgment from the M0.
  *
  * @param  None
  * @retval None
  */
static void Wait_Getting_Ack_From_M0(void)
{
  while (FlagReceiveAckFromM0 == 0)
  {
  }
  FlagReceiveAckFromM0 = 0;
}

/**
  * @brief  Receive an acknowledgment from the M0+ core.
  *         Each command send by the M4 to the M0 are acknowledged.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Ack_From_M0(void)
{
  FlagReceiveAckFromM0 = 1;
}

/**
  * @brief  Receive a notification from the M0+ through the IPCC.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Notification_From_M0(void)
{
  CptReceiveMsgFromM0++;
  osThreadFlagsSet(OsTaskMsgM0ToM4Id,1);
}

#if (CFG_USB_INTERFACE_ENABLE != 0)
#else
#if (CFG_FULL_LOW_POWER == 0)
static void RxCpltCallback(void)
{
  /* Filling buffer and wait for '\r' char */
  if (indexReceiveChar < C_SIZE_CMD_STRING)
  {
    CommandString[indexReceiveChar++] = aRxBuffer[0];
    if (aRxBuffer[0] == '\r')
    {
      CptReceiveCmdFromUser = 1U;

      /* UART task scheduling*/
      osThreadFlagsSet(OsTaskCliId,1);
    }
  }

  /* Once a character has been sent, put back the device in reception mode */
//  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1U, RxCpltCallback);
}
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_USB_INTERFACE_ENABLE != 0)
/**
 * @brief Process the command strings.
 *        As soon as a complete command string has been received, the task
 *        in charge of sending the command to the M0 is scheduled
 * @param  None
 * @retval None
 */
static uint32_t  ProcessCmdString( uint8_t* buf , uint32_t len )
{
  uint32_t i,j,tmp_start;
  tmp_start = 0;
  uint32_t res = 0;

  i= 0;
  while ((buf[i] != '\r') && (i < len))
  {
    i++;
  }

  if (i != len)
  {
    memcpy(CommandString, buf,(i+1));
    indexReceiveChar = i + 1U; /* Length of the buffer containing the command string */
    osThreadFlagsSet(OsTaskCliId,1)
    tmp_start = i;
    for (j = 0; j < (len - tmp_start - 1U) ; j++)
    {
      buf[j] = buf[tmp_start + j + 1U];
    }
    res = len - tmp_start - 1U;
  }
  else
  {
    res = len;
  }
  return res; /* Remaining characters in the temporary buffer */
}
#endif/* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_FULL_LOW_POWER == 0)
/**
 * @brief Process sends receive CLI command to M0.
 * @param  None
 * @retval None
 */
static void Send_CLI_To_M0(void)
{
  memset(ThreadCliCmdBuffer.cmdserial.cmd.payload, 0x0U, 255U);
  memcpy(ThreadCliCmdBuffer.cmdserial.cmd.payload, CommandString, indexReceiveChar);
  ThreadCliCmdBuffer.cmdserial.cmd.plen = indexReceiveChar;
  ThreadCliCmdBuffer.cmdserial.cmd.cmdcode = 0x0;

  /* Clear receive buffer, character counter and command complete */
  CptReceiveCmdFromUser = 0;
  indexReceiveChar = 0;
  memset(CommandString, 0, C_SIZE_CMD_STRING);

  TL_CLI_SendCmd();
}
#endif /* (CFG_FULL_LOW_POWER == 0) */

/**
 * @brief Send notification for CLI TL Channel.
 * @param  None
 * @retval None
 */
static void Send_CLI_Ack_For_OT(void)
{

  /* Notify M0 that characters have been sent to UART */
  TL_THREAD_CliSendAck();
}

/**
 * @brief Perform initialization of CLI UART interface.
 * @param  None
 * @retval None
 */
void APP_THREAD_Init_UART_CLI(void)
{
#if (CFG_FULL_LOW_POWER == 0)
  OsTaskCliId = osThreadNew(APP_THREAD_FreeRTOSSendCLIToM0Task, NULL,&ThreadCliProcess_attr);
#endif /* (CFG_FULL_LOW_POWER == 0) */

#if (CFG_USB_INTERFACE_ENABLE != 0)
#else
#if (CFG_FULL_LOW_POWER == 0)
//  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1, RxCpltCallback);
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */
}

/**
 * @brief Perform initialization of TL for THREAD.
 * @param  None
 * @retval None
 */
void APP_THREAD_TL_THREAD_INIT(void)
{
  ThreadConfigBuffer.p_ThreadOtCmdRspBuffer = (uint8_t*)&ThreadOtCmdBuffer;
  ThreadConfigBuffer.p_ThreadNotAckBuffer = (uint8_t*)ThreadNotifRspEvtBuffer;
  ThreadConfigBuffer.p_ThreadCliRspBuffer = (uint8_t*)&ThreadCliCmdBuffer;

  TL_THREAD_Init( &ThreadConfigBuffer );
}

/**
 * @brief  This function is called when notification on CLI TL Channel from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_CliNotReceived( TL_EvtPacket_t * Notbuffer )
{
  TL_CmdPacket_t* l_CliBuffer = (TL_CmdPacket_t*)Notbuffer;
  uint8_t l_size = l_CliBuffer->cmdserial.cmd.plen;

  /* WORKAROUND: if string to output is "> " then respond directly to M0 and do not output it */
  if (strcmp((const char *)l_CliBuffer->cmdserial.cmd.payload, "> ") != 0)
  {
    /* Write to CLI UART */
#if (CFG_USB_INTERFACE_ENABLE != 0)
    VCP_SendData( l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#else
//    HW_UART_Transmit_IT(CFG_CLI_UART, l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#endif /*USAGE_OF_VCP */
  }
  else
  {
    Send_CLI_Ack_For_OT();
  }
}

/**
 * @brief  End of transfer callback for CLI UART sending.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void HostTxCb(void)
{
  Send_CLI_Ack_For_OT();
}

/**
 * @brief Process the messages coming from the M0.
 * @param  None
 * @retval None
 */
void APP_THREAD_ProcessMsgM0ToM4(void)
{
  if (CptReceiveMsgFromM0 != 0)
  {
    /* If CptReceiveMsgFromM0 is > 1. it means that we did not serve all the events from the radio */
    if (CptReceiveMsgFromM0 > 1U)
    {
      APP_THREAD_Error(ERR_REC_MULTI_MSG_FROM_M0, 0);
    }
    else
    {
      OpenThread_CallBack_Processing();
    }
    /* Reset counter */
    CptReceiveMsgFromM0 = 0;
  }
}

#if (CFG_USB_INTERFACE_ENABLE != 0)
/**
 * @brief  This function is called when thereare some data coming
 *         from the Hyperterminal via the USB port
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 * @param  Buf: Buffer of data received
 * @param  Len: Number of data received (in bytes)
 * @retval Number of characters remaining in the buffer and not yet processed
 */
void VCP_DataReceived(uint8_t* Buf , uint32_t *Len)
{
  uint32_t i,flag_continue_checking = TRUE;
  uint32_t char_remaining = 0;
  static uint32_t len_total = 0;

  /* Copy the characteres in the temporary buffer */
  for (i = 0; i < *Len; i++)
  {
    TmpString[len_total++] = Buf[i];
  }

  /* Process the buffer commands one by one     */
  /* A command is limited by a \r caracaters    */
  while (flag_continue_checking == TRUE)
  {
    char_remaining = ProcessCmdString(TmpString,len_total);
    /* If char_remaining is equal to len_total, it means that the command string is not yet
     * completed.
     * If char_remaining is equal to 0, it means that the command string has
     * been entirely processed.
     */
    if ((char_remaining == 0) || (char_remaining == len_total))
    {
      flag_continue_checking = FALSE;
    }
    len_total = char_remaining;
  }
}
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
