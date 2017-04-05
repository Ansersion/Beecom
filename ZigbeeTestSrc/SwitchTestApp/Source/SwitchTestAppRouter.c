/******************************************************************************
  Filename:       SwitchTestApp.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "SwitchTestAppRouter.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

/*********************************************************************
 * MACROS
 */
#define   OPENLED1      P1_2=0
#define   OPENLED2      P1_3=0

#define   CLOSLED1      P1_2=1
#define   CLOSLED2      P1_3=1

#define   Led1Open      0x10
#define   Led1Clos      0x11

#define   Led2Open      0x20
#define   Led2Clos      0x21
/*********************************************************************
 * CONSTANTS
 */

//无线数据收发 
#define AF_TXRX_DATA_LEN  16  //&&开头 >>end 10 字符

static uint8 CodRecv_AFBuf[AF_TXRX_DATA_LEN];
static uint8 CodSend_AFBuf[AF_TXRX_DATA_LEN];


static uint8 SerialApp_RxBuf[SERIAL_APP_RSP_CNT];

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t SwitchTestApp_ClusterList[SwitchTestApp_MAX_CLUSTERS] =
{
  SwitchTestApp_CLUSTERID
};

const SimpleDescriptionFormat_t SwitchTestApp_SimpleDesc =
{
  SwitchTestApp_ENDPOINT,              //  int Endpoint;
  SwitchTestApp_PROFID,                //  uint16 AppProfId[2];
  SwitchTestApp_DEVICEID,              //  uint16 AppDeviceId[2];
  SwitchTestApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  SwitchTestApp_FLAGS,                 //  int   AppFlags:4;
  SwitchTestApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SwitchTestApp_ClusterList,  //  byte *pAppInClusterList;
  SwitchTestApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SwitchTestApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SwitchTestApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SwitchTestApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte SwitchTestApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SwitchTestApp_Init() is called.
devStates_t SwitchTestApp_NwkState;


byte SwitchTestApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SwitchTestApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Init_Port(void);
static void SwitchTestApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void SwitchTestApp_HandleKeys( byte shift, byte keys );
static void SwitchTestApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void SwitchTestApp_SendTheMessage( void );
static void SwitchTestApp_SendROUMsg( void );
static void SerialApp_CallBack(uint8 port, uint8 event);

#if defined( IAR_ARMCM3_LM )
static void SwitchTestApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void Init_Port(void)
{
  P1DIR|=(0x01<<2);
  P1DIR|=(0x01<<3);
  CLOSLED1;
  CLOSLED2;
}

/*********************************************************************
 * @fn      SwitchTestApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SwitchTestApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  
  SwitchTestApp_TaskID = task_id;
  SwitchTestApp_NwkState = DEV_INIT;
  SwitchTestApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  SwitchTestApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  SwitchTestApp_DstAddr.endPoint = 0;
  SwitchTestApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  SwitchTestApp_epDesc.endPoint = SwitchTestApp_ENDPOINT;
  SwitchTestApp_epDesc.task_id = &SwitchTestApp_TaskID;
  SwitchTestApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SwitchTestApp_SimpleDesc;
  SwitchTestApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SwitchTestApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SwitchTestApp_TaskID );
  
   //uart 0 init
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_9600;
  uartConfig.flowControl          = FALSE;
  //uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 256;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 256;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 100;   // 2x30 don't care - see uart driver.
  //uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
  
  Init_Port();
  
  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SwitchTestApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( SwitchTestApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( SwitchTestApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, SwitchTestApp_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      SwitchTestApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SwitchTestApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SwitchTestApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          SwitchTestApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          SwitchTestApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          SwitchTestApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          SwitchTestApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SwitchTestApp_NwkState == DEV_ZB_COORD)
              || (SwitchTestApp_NwkState == DEV_ROUTER)
              || (SwitchTestApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( SwitchTestApp_TaskID,
                                SwitchTestApp_SEND_MSG_EVT,
                                SwitchTestApp_SEND_MSG_TIMEOUT );
            if(SwitchTestApp_NwkState == DEV_ROUTER)
            {
                HalUARTWrite(0,">>Router LINK State <<",osal_strlen(">>Router LINK State "));
                SwitchTestApp_SendROUMsg();
            }
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SwitchTestApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SwitchTestApp_Init()).
  if ( events & SwitchTestApp_SEND_MSG_EVT )
  {
    // Send "the" message
    SwitchTestApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( SwitchTestApp_TaskID,
                        SwitchTestApp_SEND_MSG_EVT,
                        SwitchTestApp_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ SwitchTestApp_SEND_MSG_EVT);
  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & SwitchTestApp_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    SwitchTestApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ SwitchTestApp_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      SwitchTestApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void SwitchTestApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            SwitchTestApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            SwitchTestApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            SwitchTestApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      SwitchTestApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SwitchTestApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      // Since SW1 isn't used for anything else in this application...
#if defined( SWITCH1_BIND )
      // we can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            SwitchTestApp_epDesc.endPoint,
                            SwitchTestApp_PROFID,
                            SwitchTestApp_MAX_CLUSTERS, (cId_t *)SwitchTestApp_ClusterList,
                            SwitchTestApp_MAX_CLUSTERS, (cId_t *)SwitchTestApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        SwitchTestApp_PROFID,
                        SwitchTestApp_MAX_CLUSTERS, (cId_t *)SwitchTestApp_ClusterList,
                        SwitchTestApp_MAX_CLUSTERS, (cId_t *)SwitchTestApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SwitchTestApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void SwitchTestApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case SwitchTestApp_CLUSTERID:
      HalUARTWrite(HAL_UART_PORT_0,pkt->cmd.Data,osal_strlen(pkt->cmd.Data));
      osal_memcpy(CodRecv_AFBuf,pkt->cmd.Data,AF_TXRX_DATA_LEN);
      //if(CodRecv_AFBuf[0]=='&' && CodRecv_AFBuf[1]=='&')
      {
          //组网发送信息
          if(osal_memcmp(CodRecv_AFBuf,"&&OPENLED1<<",osal_strlen("&&OPENLED1<<")))
          {
              OPENLED1;
	      HalUARTWrite(HAL_UART_PORT_0,"open led 1",osal_strlen("open led 1"));
              HalLedSet ( HAL_LED_1, HAL_LED_MODE_TOGGLE );
          }
          else if(osal_memcmp(CodRecv_AFBuf,"&&OPENLED2<<",12))
          {
                //FSetShortAddr(SwitchRouterAddr,);
              OPENLED2;
              HalUARTWrite(HAL_UART_PORT_0,"open led 2",osal_strlen("open led 2"));
              HalLedSet ( HAL_LED_2, HAL_LED_MODE_TOGGLE );
          }
          else if(osal_memcmp(CodRecv_AFBuf,"&&CLOSELED1<<",12))
          {
              //FSetShortAddr(SwitchRouterAddr,);
              CLOSLED1;
              HalUARTWrite(HAL_UART_PORT_0,"Close led 1",osal_strlen("Close led 1"));
              HalLedSet ( HAL_LED_1, HAL_LED_MODE_TOGGLE );
          }
          else if(osal_memcmp(CodRecv_AFBuf,"&&CLOSELED2<<",12))
          {
              //FSetShortAddr(SwitchRouterAddr,);
              CLOSLED2;
              HalUARTWrite(HAL_UART_PORT_0,"Close led 2",osal_strlen("Close led 2"));
              HalLedSet ( HAL_LED_2, HAL_LED_MODE_TOGGLE );
          }
          
          
          
      }
      
      
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      SwitchTestApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void SwitchTestApp_SendTheMessage( void )
{
  char theMessageData[] = "Hello World";

  if ( AF_DataRequest( &SwitchTestApp_DstAddr, &SwitchTestApp_epDesc,
                       SwitchTestApp_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &SwitchTestApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}


/*********************************************************************
 * @fn      SwitchTestApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void SwitchTestApp_SendROUMsg( void )
{
  //char theMessageData[] = "Hello World";
  
  afAddrType_t dstAddrType_t;
  
  dstAddrType_t.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddrType_t.endPoint = SwitchTestApp_ENDPOINT;
  dstAddrType_t.addr.shortAddr = 0x0000;
  
  osal_memset(CodSend_AFBuf,0,AF_TXRX_DATA_LEN);
  osal_memcpy(CodSend_AFBuf,"&&ROUTER_LINK<<",AF_TXRX_DATA_LEN);

  if ( AF_DataRequest( &dstAddrType_t, &SwitchTestApp_epDesc,
                       SwitchTestApp_CLUSTERID,
                       (byte)osal_strlen( CodSend_AFBuf ) + 1,
                       (byte *)&CodSend_AFBuf,
                       &SwitchTestApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      SwitchTestApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void SwitchTestApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}
#endif


/*********************************************************************
 * @fn      SerialApp_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void SerialApp_CallBack(uint8 port, uint8 event)
{
    (void)port;

    uint8 readLen = 0;

    osal_memset(SerialApp_RxBuf,0,SERIAL_APP_RSP_CNT);
  
    readLen =  HalUARTRead(HAL_UART_PORT_0,SerialApp_RxBuf,SERIAL_APP_RSP_CNT);
    if(0 < readLen)
    {
          //osal_memcpy(SerialApp_RxBuf,"&&CLOSELED1<<",osal_strlen("&&CLOSELED1<<"));
          //Switch_SendMsgToRouter();  
        if(osal_memcmp(SerialApp_RxBuf,"&&OPENLED1<<",AF_TXRX_DATA_LEN))
        {
            OPENLED1;
	    HalUARTWrite(HAL_UART_PORT_0,"open led 1",osal_strlen("open led 1"));
        }
        else if(osal_memcmp(SerialApp_RxBuf,"&&OPENLED2<<",AF_TXRX_DATA_LEN))
        {
                //FSetShortAddr(SwitchRouterAddr,);
            OPENLED2;
            HalUARTWrite(HAL_UART_PORT_0,"open led 2",osal_strlen("open led 2"));
        }
        else if(osal_memcmp(SerialApp_RxBuf,"&&CLOSELED1<<",AF_TXRX_DATA_LEN))
        {
            //FSetShortAddr(SwitchRouterAddr,);
            CLOSLED1;
            HalUARTWrite(HAL_UART_PORT_0,"Close led 1",osal_strlen("Close led 1"));
        }
        else if(osal_memcmp(SerialApp_RxBuf,"&&CLOSELED2<<",AF_TXRX_DATA_LEN))
        {
            //FSetShortAddr(SwitchRouterAddr,);
            CLOSLED2;
            HalUARTWrite(HAL_UART_PORT_0,"Close led 2",osal_strlen("Close led 2"));
        }
    }
  

  #if 0
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
        !SerialApp_TxLen)

  {
     //SerialApp_Send();
  }
  #endif
}


/*********************************************************************
 */
