/******************************************************************************
Filename:     rf_test.c
Target:       cc2430
Revised:      16/12-2005
Revision:     1.0

Description:
    This file provides 4 small tests which demonstrates use of the radio.

******************************************************************************/
#include "ioCC2530.h"

#include <hal_led.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "basic_rf.h"
#include "hal_types.h"

// Application parameters
#define RF_CHANNEL               0x0b      // 2.4 GHz RF channel


// BasicRF address definitions
#define PAN_ID        0x07
#define ADDRESS_0     0x01
#define ADDRESS_1     0x02

#define SEND     0
#define RECEIVE  1
/*
#define SINGLE      0
#define CONTINUOUS  1
#define PING_PONG   2
#define PER_TEST    3
#define EXIT        4
*/
/*
#define RECEIVE_TIMEOUT               800

#define PING_PONG_TIMEOUT             1200
#define PING_PONG_REQUEST             0x80
#define PING_PONG_RESPONSE            0x40

#define PER_RECEIVE_TIMEOUT          10000
#define PER_TOTAL_PACKET_NUMBER       1000
#define PER_TEST_REQUEST              0x20
#define COMPLETE_APPLICATION

*/
void initRfTest(void);
void rf_test_main(void);
//void receivePacket(UINT8 *receiveByte);
//void sendPacket(UINT8 sendByte);

#ifdef SECURITY_CCM
// Security key
static uint8 key[]= {
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
};
#endif

void receiveMode(void);
void contionuousMode(void);

basicRfCfg_t basicRfConfig;

 uint8  RxTxState;
 uint8  myAddr;
 uint8  remoteAddr;
/******************************************************************************
* @fn  initRfTest
*
* @brief
*      Initializes components for the RF test application example.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void initRfTest(void)
{
   halLedClear(1);
   halLedClear(2);
   basicRfInit(&basicRfConfig);
}


/******************************************************************************
* @fn  rf_test_main
*
* @brief
*      Main function.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/

void rf_test_main(void){

   #ifdef RX
   {
                       
                        
   			basicRfConfig.myAddr = ADDRESS_0;
   			remoteAddr = ADDRESS_1;
                        basicRfConfig.channel = RF_CHANNEL;
                        basicRfConfig.panId = PAN_ID;
                        basicRfConfig.ackRequest = TRUE;//;FALSE
                        #ifdef SECURITY_CCM
                        basicRfConfig.securityKey = key;
                        #endif
   			initRfTest(); 
                        halLedSet(2);
                        halLedSet(1);
                        receiveMode();
                       
                        
   }
   #else
   {
                        
                         
   			basicRfConfig.myAddr = ADDRESS_1;
   			remoteAddr = ADDRESS_0;
                        basicRfConfig.channel = RF_CHANNEL;
                        basicRfConfig.panId = PAN_ID;  
                        basicRfConfig.ackRequest =TRUE ;//;FALSE
                        #ifdef SECURITY_CCM
                        basicRfConfig.securityKey = key;
                        #endif
   			initRfTest(); 
                        halLedSet(2);
                        halLedSet(1);
                        contionuousMode(); 
                       
   }
   #endif
}



/******************************************************************************
* @fn  contionuousMode
*
* @brief
*      Function for transferring packets until stopped by user.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void contionuousMode(void)
{
  
  uint8 res;// BOOL
 uint8 sendBuffer[5] ={1,2,3,4,5} ;//"Hello";BYTE  BYTE
 if(halRfInit()==FAILED) {
      HAL_ASSERT(FALSE);
    }
  while(1)
  
   {
     
     halLedClear(1);
      if(basicRfInit(&basicRfConfig)==FAILED)
      {
      HAL_ASSERT(FALSE);
      }
       // Keep Receiver off when not needed to save power
       basicRfReceiveOff();
       halLedSet(2);
       res = basicRfSendPacket(remoteAddr,sendBuffer, sizeof(sendBuffer));
       
       halIntOff();
       halMcuSetLowPowerMode(HAL_MCU_LPM_3); // Will turn on global
            // interrupt enable
       halIntOn();
       basicRfReceiveOn();
      halMcuWaitMs(200);
      // U0DBUF = res;
      // while (!UTX0IF);
      // UTX0IF = 0;
    //  YLED = LED_OFF;
      //halWait(200);
      if(res == TRUE)
      {
        res =0;
        int j,m;
        m=sizeof(sendBuffer);  
         halLedSet(1);
         for(j=0;j<m;j++)
         { 
          U0DBUF =sendBuffer[j];
          while (!UTX0IF);
          UTX0IF = 0;
         //0X01;        
         }
         j=0;
   halMcuWaitMs(200);
      }
      else
      {
      
       //halLedClear(2); 
      halMcuWaitMs(200);
      
      }
   }
  
 

}



/******************************************************************************
* @fn  receiveMode
*
* @brief
*      Function for receiving data.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void receiveMode(void)
{
  uint8 receiveBuffer[100]={0};
  uint8  length = 100;
  uint8  ress;
   //BYTE sender;
  if(halRfInit()==FAILED) {
      HAL_ASSERT(FALSE);
    }
   while(1)
   {
    
     halLedClear(2);
     if(basicRfInit(&basicRfConfig)==FAILED)
     {
     HAL_ASSERT(FALSE);
     }
     
     basicRfReceiveOn();
     
     while(!basicRfPacketIsReady());
     ress = basicRfReceive(receiveBuffer, length,NULL);
      
     if(ress > 0)
     {
       int j;
       halLedSet(2);
       for(j=0;j<ress;j++)
       {
         U0DBUF =receiveBuffer[j] ;//0X02;
         while (!UTX0IF);
         UTX0IF = 0;
       }         
       ress = 0;    
       halMcuWaitMs(50);
     }else{          
       halMcuWaitMs(200);
     }     
   }
}


