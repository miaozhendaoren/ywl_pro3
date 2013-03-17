/*******************************************************************************
 * �ļ����ƣ�Humiture_Ex.c
 * ��    �ܣ�����������ʵ�� --- ��ʪ�ȴ�����
 * ��    ����ʹ����ʪ�ȴ�����SHT11������ʪ�Ȳɼ�������¶�㣬��LCD����ʾ��
 * ʵ��Ӳ������SK-Sensor-Temperature And Humidity��������嵽SK-SmartRF05EB��
 *           P4(CC2530 IO)��(��һ��ע�ⷽ���ܲ巴).
 *           ��SK-SmartRF05EB�ϵ�P6:7-8;P6:19-20�ϵ�����ñ�ε���
 * ��    �ߣ�POWER
 * ��    ˾����̶˹�����ӿƼ����޹�˾
 *           www.sikai-tech.com
 * ��    �ڣ�2010-04-18
 ******************************************************************************/


/* ����ͷ�ļ� */
/********************************************************************/
#include "ioCC2530.h"    // CC2530��ͷ�ļ���������CC2530�ļĴ������ж������ȵĶ���
#include "LCD.h"         // lcd ����ͷ�ļ�
#include "SHT11.h"       // SHT11������ͷ�ļ�
#include "stdio.h"       // C���Ա�׼����/������ͷ�ļ�
/********************************************************************/


/* ����ö������ */
/********************************************************************/
enum SYSCLK_SRC{XOSC_32MHz,RC_16MHz};  // ����ϵͳʱ��Դ(��ʱ��Դ)ö������
/********************************************************************/


/* �û��Զ������� */
/********************************************************************/
typedef union
{ 
  unsigned int i;
  float f;
} value;
/********************************************************************/



/*********************************************************************
 * �������ƣ�delayUS
 * ��    �ܣ������ʱ����ʱָ����΢������
 * ��ڲ�����usec  ��ʱ������ֵԽ����ʱʱ��Խ������λ:us
 * ���ڲ�������
 * �� �� ֵ����
 * ע    �⣺�˺����߶�������MCU�ܹ��ͱ�����
 ********************************************************************/
#pragma optimize = none
void delayUS(unsigned short usec)
{
  usec >>= 1;
  while(usec--)
  {
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
  }
}


/*********************************************************************
 * �������ƣ�delayMS
 * ��    �ܣ������ʱ����ʱָ���ĺ�������
 * ��ڲ�����msec  ��ʱ������ֵԽ����ʱʱ��Խ������λ:ms
 * ���ڲ�������
 * �� �� ֵ����
 * ע    �⣺�˺����߶�������MCU�ܹ��ͱ�����
 ********************************************************************/
#pragma optimize = none
void delayMS(unsigned short msec)
{
  while(msec--)
    delayUS(1000);
}


/*********************************************************************
 * �������ƣ�SystemClockSourceSelect
 * ��    �ܣ�ѡ��ϵͳʱ��Դ(��ʱ��Դ)
 * ��ڲ�����source
 *             XOSC_32MHz  32MHz��������
 *             RC_16MHz    16MHz RC����
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void SystemClockSourceSelect(enum SYSCLK_SRC source)
{
  unsigned char osc32k_bm = CLKCONCMD & 0x80;
  unsigned char __clkconcmd,__clkconsta;
      
  /*
     ϵͳʱ��Դ(��ʱ��Դ)ѡ��16MHz RC��������ʱ��tick����Ϊ16MHz��ʱ���ٶ�����Ϊ16MHz
     CLKCONCMD.OSC32K[b7]���ı�      32KHzʱ��Դѡ�񱣳���ǰ����
     CLKCONCMD.OSC[b6] = 1           ϵͳʱ��Դ(��ʱ��Դ)ѡ��16MHz RC����
     CLKCONCMD.TICKSPD[b5..b3] = 001 ��ʱ��tick����Ϊ16MHz
     CLKCONCMD.CLKSPD[b2..b0] = 001  ʱ���ٶ�����Ϊ16MHz
   */
  if(source == RC_16MHz)
  {              /* CLKCONCMD.OSC32K[b7] */
    CLKCONCMD = ((osc32k_bm) | \
                 /* CLKCONCMD.OSC[b6] = 1 */
                 (0x01 << 6) | \
                 /* CLKCONCMD.TICKSPD[b5..b3] = 001 */  
                 (0x01 << 3) | \
                 /* CLKCONCMD.CLKSPD[b2..b0] = 001 */
                 (0x01 << 0));
  }

  
  /*
     ϵͳʱ��Դ(��ʱ��Դ)ѡ��32MHz������������ʱ��tick����Ϊ32MHz��ʱ���ٶ�����Ϊ32MHz
     CLKCONCMD.OSC32K[b7]���ı�      32KHzʱ��Դѡ�񱣳���ǰ����
     CLKCONCMD.OSC[b6] = 0           ϵͳʱ��Դ(��ʱ��Դ)ѡ��32MHz��������
     CLKCONCMD.TICKSPD[b5..b3] = 000 ��ʱ��tick����Ϊ32MHz
     CLKCONCMD.CLKSPD[b2..b0] = 000  ʱ���ٶ�����Ϊ32MHz
   */  
  else if(source == XOSC_32MHz)
  {
    CLKCONCMD = (osc32k_bm /*| (0x00<<6) | (0x00<<3) | (0x00 << 0)*/);
  }
  
  
  /* �ȴ���ѡ���ϵͳʱ��Դ(��ʱ��Դ)�ȶ� */
  __clkconcmd = CLKCONCMD;             // ��ȡʱ�ӿ��ƼĴ���CLKCONCMD
  do
  {
    __clkconsta = CLKCONSTA;           // ��ȡʱ��״̬�Ĵ���CLKCONSTA
  }while(__clkconsta != __clkconcmd);  // ֱ��CLKCONSTA�Ĵ�����ֵ��CLKCONCMD�Ĵ�
                                       // ����ֵһ�£�˵����ѡ���ϵͳʱ��Դ(��
                                       // ʱ��Դ)�Ѿ��ȶ�  
}


/*********************************************************************
 * �������ƣ�main
 * ��    �ܣ�main�������
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void main(void)
{
  value humi_val,temp_val;
  float dew_point;
  unsigned char error,checksum;
  char ss[20];
  
  SystemClockSourceSelect(XOSC_32MHz);  // ѡ��32MHz����������Ϊϵͳʱ��Դ(��ʱ��Դ)
  
  HalLcdInit();          // LCD��ʼ��
  HalLcd_HW_Clear();     // ���� 
  
  /* ��LCD����ʾ�����Ϣ */
  HalLcdWriteString("CC253x - Humi&Temp", HAL_LCD_LINE_1); // ��ʪ�ȴ�����ʵ��
  delayMS(20);
  
  SHT11_Init();        // SHT11��ʼ��
  
  while(1)
  { 
    /* ����SHT11������ʪ�Ȳ��� */
    error = 0;
    error += SHT11_Measure((unsigned char*) &humi_val.i,&checksum,SHT11_HUMI); // �������ʪ��
    error += SHT11_Measure((unsigned char*) &temp_val.i,&checksum,SHT11_TEMP); // �����¶�
    
    /* ��SHT11�������� */
    if(error != 0) 
    {
      SHT11_ConnectionReset();  // ͨ�Ÿ�λ                          
    }
    else
    /* ��SHT11�������� */
    {
      /* ���ݲ����������ʵ��������ֵ */
      humi_val.f = (float)humi_val.i;                         // ��������ת��
      temp_val.f = (float)temp_val.i;
      SHT11_CALC(&humi_val.f,&temp_val.f);                    // �������ʪ�Ⱥ��¶�
      dew_point = SHT11_CALCDewpoint(humi_val.f,temp_val.f);  // ����¶��

      /* 
         ��ʾ������� 
      */
      /* ��ʾ�¶� */
      sprintf(ss,(char *)"Temp - %5.1fC",temp_val.f);
      HalLcdWriteString((char *)ss, HAL_LCD_LINE_3);
      
      /* ��ʾʪ�� */
      sprintf(ss,(char *)"Humi - %5.1f%%",humi_val.f);
      HalLcdWriteString((char *)ss, HAL_LCD_LINE_4);
      
      /* ��ʾ¶�� */
      sprintf(ss,(char *)"Dew_p - %5.1fC",dew_point);
      HalLcdWriteString((char *)ss, HAL_LCD_LINE_5);
    }
  }
}