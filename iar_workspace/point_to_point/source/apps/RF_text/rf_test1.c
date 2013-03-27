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
#include "..\..\components\sht11\SHT11.h"
#include <stdio.h>
#include <string.h>

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
#undef RX
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
                        basicRfConfig.ackRequest =FALSE ;//;FALSE
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
  uint8 sendBuffer[5] ={'1','2','3','4','5'} ;//"Hello";BYTE  BYTE
  char ss[25];
  uint8 i;
  
 if(halRfInit()==FAILED) {
      HAL_ASSERT(FALSE);
    }
  while(1)
   {
     /* 传感器 模块,采集数据 */
     sht11MakeData();         
     if(gf_isSht11DataReady)
     {
       /* 将传感器采集到的数据 形成应用协议包 */
       // gf_temp float 温度
       // gf_humi float 湿度
       // gf_dewPoint float 湿度
       
        for(i=0; i<4; i++)
        {
           if(i==0) sprintf(ss,(char *)"Temp - %5.1fC\n",gf_temp);
           else if(i==1) sprintf(ss,(char *)"Humi - %5.1f%%\n",gf_humi);
           else if(i==2) sprintf(ss,(char *)"Dew_p - %5.1fC\n",gf_dewPoint);
           else if(i==3) sprintf(ss,"----------------\n");
           
           /* 将形成的协议包 通过无线发送给汇集器 */
           halLedClear(1);
           if(basicRfInit(&basicRfConfig)==FAILED)
           {
             HAL_ASSERT(FALSE);
           }
             // Keep Receiver off when not needed to save power
            basicRfReceiveOff();
            halLedSet(2);
            res = basicRfSendPacket(remoteAddr,(uint8*)(&ss[0]),strlen(ss));
             
            halIntOff();
            halMcuSetLowPowerMode(HAL_MCU_LPM_3); // Will turn on global
            
            // interrupt enable
            halIntOn();
            basicRfReceiveOn();
            halMcuWaitMs(200);
      
            if(res == SUCCESS)
            {
                res =0;
                int j,m;
                
                m=sizeof(ss);  
                halLedSet(1);
                for(j=0;ss[j] != '\0';j++)
                { 
                  U0DBUF =ss[j];
                  while (!UTX0IF);
                  UTX0IF = 0;
                }
                j=0;
                halMcuWaitMs(200);
            }//end if(res == TRUE)                       
        }        
        /* 将ready 写为0,进行下次采集 由sht11MakeData()完成*/
        gf_isSht11DataReady = 0;
        
     }// end if(gf_isSht11DataReady)
   }//end while(1){}
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
  uint8 receiveBuffer[10]={0};
  uint8  length;
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
       halLedSet(1);
      while(!basicRfPacketIsReady());
       ress = basicRfReceive(receiveBuffer, length,NULL);
      //if(res>0) 
     
     
     

      if(ress > 0)
      {
        int j;
        halLedSet(2);
         for(j=0;j<5;j++)
         {
          U0DBUF =receiveBuffer[j] ;//0X02;
          while (!UTX0IF);
          UTX0IF = 0;
          ress = 0;
         }
         j=0;
         U0DBUF = '\n';
         while(!UTX0IF);
         UTX0IF = 0;
        halMcuWaitMs(200);
        
      }
      else
      {  
        
        //halLedClear(1);
       halMcuWaitMs(200);
      }
     
   }
    //while (!UTX0IF);
   //UTX0IF = 0;
   //U0DBUF = &receiveBuffer;
}


