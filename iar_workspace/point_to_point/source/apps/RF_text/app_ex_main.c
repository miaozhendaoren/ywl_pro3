/******************************************************************************
Filename:     app_ex_main.c
Target:       cc2530
Revised:      02/09-2010
Revision:     1.0

Description:
    Application example

******************************************************************************/
#include <ioCC2530.h>
extern void rf_test_main(void);
extern void  halBoardInit(void);
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
   
   initUART();
   
    halBoardInit();
    
   rf_test_main();
}
