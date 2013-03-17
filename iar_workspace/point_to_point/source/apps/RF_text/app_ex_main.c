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
