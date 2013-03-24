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
 CLKCONCMD = 0X00;   //ϵͳʱ��32MHz
  while (CLKCONSTA == 0X00); 
  PERCFG = 0x00;//�ⲿ�豸����
  P0SEL = 0X3C;     //P0<5:2>���ó���ΧӦ��
 U0CSR|= 0X80;   //UARTģʽ������żУ�飬
 U0GCR |= 0X0A;
 U0BAUD |= 216;    //������57600
 U0CSR |= 0x40; 
 UTX0IF = 1;
}

/******************************************************************************
* @fn  sndStrUart0
*
* @brief
*      send string through uart0, string must be ended with '\0'
*
* Parameters: pbuf���ͻ������׵�ַ,��'\0'Ϊ����
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
  
   /* ���ڳ�ʼ�� */
    initUART();
    /* SHT11 ��ʼ�� */
   SHT11_Init();   
   
   halBoardInit();
    
   rf_test_main();
        
   while(1){
     
     sht11MakeData();         
     if(gf_isSht11DataReady)
     {
         /* ��ʾ�¶� */
         sprintf(ss,(char *)"Temp - %5.1fC",gf_temp);
         sndStrUart0(ss);
         sndCharUart0('\n');
        
         /* ��ʾʪ�� */
         sprintf(ss,(char *)"Humi - %5.1f%%",gf_humi);
         sndStrUart0(ss);
         sndCharUart0('\n');        
        
         /* ��ʾ¶�� */
          sprintf(ss,(char *)"Dew_p - %5.1fC",gf_dewPoint);
         sndStrUart0(ss);      
         sndCharUart0('\n');
        
         /*������*/
         sndStrUart0("----------------\n");
         halMcuWaitMs(2000);     
       gf_isSht11DataReady = 0;  
     }
   }
      
}
