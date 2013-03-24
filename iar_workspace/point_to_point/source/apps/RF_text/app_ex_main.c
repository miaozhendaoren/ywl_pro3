/******************************************************************************
Filename:     app_ex_main.c
Target:       cc2530
Revised:      02/09-2010
Revision:     1.0

Description:
    Application example

******************************************************************************/
#include "..\..\components\sht11\SHT11.h"
#include <ioCC2530.h>
#include <stdio.h>
extern void rf_test_main(void);
extern void  halBoardInit(void);
extern void  halMcuWaitMs(unsigned short);     
/******************************************************************************
* @fn  initUART
*
* @brief
*      Initializes components for the UART application example.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void initUART(void)
{
 CLKCONCMD = 0X00;   //系统时钟32MHz
  while (CLKCONSTA == 0X00); 
  PERCFG = 0x00;//外部设备控制
  P0SEL = 0X3C;     //P0<5:2>设置成外围应用
 U0CSR|= 0X80;   //UART模式。无奇偶校验，
 U0GCR |= 0X0A;
 U0BAUD |= 216;    //波特率57600
 U0CSR |= 0x40; 
 UTX0IF = 1;
}

/******************************************************************************
* @fn  sndStrUart0
*
* @brief
*      send string through uart0, string must be ended with '\0'
*
* Parameters: pbuf发送缓冲区首地址,以'\0'为结束
*
* @param  char* pbuf
*
* @return void
*
******************************************************************************/
void sndStrUart0(char* pbuf)
{
  UTX0IF = 0;
  while(*pbuf)
  {
    U0DBUF = *pbuf++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

/******************************************************************************
* @fn  sndCharUart0
*
* @brief
*      send a character throut uart0
*
* Parameters: char ch
*
* @param  char* pbuf
*
* @return void
*
******************************************************************************/
void sndCharUart0(char ch)
{
  UTX0IF = 0;
  U0DBUF = ch;
  while(UTX0IF == 0);
  UTX0IF = 0;
}



/******************************************************************************
* @fn  main
*
* @brief
*      Main function of application example.
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/

void main(void)
{   
  char ss[20];
  
   /* 串口初始化 */
    initUART();
    /* SHT11 初始化 */
   SHT11_Init();   
   
   halBoardInit();
    
   rf_test_main();
        
   while(1){
     
     sht11MakeData();         
     if(gf_isSht11DataReady)
     {
         /* 显示温度 */
         sprintf(ss,(char *)"Temp - %5.1fC",gf_temp);
         sndStrUart0(ss);
         sndCharUart0('\n');
        
         /* 显示湿度 */
         sprintf(ss,(char *)"Humi - %5.1f%%",gf_humi);
         sndStrUart0(ss);
         sndCharUart0('\n');        
        
         /* 显示露点 */
          sprintf(ss,(char *)"Dew_p - %5.1fC",gf_dewPoint);
         sndStrUart0(ss);      
         sndCharUart0('\n');
        
         /*发空行*/
         sndStrUart0("----------------\n");
         halMcuWaitMs(2000);     
       gf_isSht11DataReady = 0;  
     }
   }
      
}
