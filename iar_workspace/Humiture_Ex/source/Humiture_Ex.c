/*******************************************************************************
 * 文件名称：Humiture_Ex.c
 * 功    能：传感器基础实验 --- 温湿度传感器
 * 描    述：使用温湿度传感器SHT11进行温湿度采集并计算露点，在LCD上显示。
 * 实验硬件：将SK-Sensor-Temperature And Humidity传感器板插到SK-SmartRF05EB的
 *           P4(CC2530 IO)上(请一定注意方向不能插反).
 *           将SK-SmartRF05EB上的P6:7-8;P6:19-20上的跳线帽拔掉。
 * 作    者：POWER
 * 公    司：湘潭斯凯电子科技有限公司
 *           www.sikai-tech.com
 * 日    期：2010-04-18
 ******************************************************************************/


/* 包含头文件 */
/********************************************************************/
#include "ioCC2530.h"    // CC2530的头文件，包含对CC2530的寄存器、中断向量等的定义
#include "LCD.h"         // lcd 驱动头文件
#include "SHT11.h"       // SHT11驱动的头文件
#include "stdio.h"       // C语言标准输入/输出库的头文件
/********************************************************************/


/* 定义枚举类型 */
/********************************************************************/
enum SYSCLK_SRC{XOSC_32MHz,RC_16MHz};  // 定义系统时钟源(主时钟源)枚举类型
/********************************************************************/


/* 用户自定义类型 */
/********************************************************************/
typedef union
{ 
  unsigned int i;
  float f;
} value;
/********************************************************************/



/*********************************************************************
 * 函数名称：delayUS
 * 功    能：软件延时。延时指定的微秒数。
 * 入口参数：usec  延时参数，值越大延时时间越长，单位:us
 * 出口参数：无
 * 返 回 值：无
 * 注    意：此函数高度依赖于MCU架构和编译器
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
 * 函数名称：delayMS
 * 功    能：软件延时。延时指定的毫秒数。
 * 入口参数：msec  延时参数，值越大延时时间越长，单位:ms
 * 出口参数：无
 * 返 回 值：无
 * 注    意：此函数高度依赖于MCU架构和编译器
 ********************************************************************/
#pragma optimize = none
void delayMS(unsigned short msec)
{
  while(msec--)
    delayUS(1000);
}


/*********************************************************************
 * 函数名称：SystemClockSourceSelect
 * 功    能：选择系统时钟源(主时钟源)
 * 入口参数：source
 *             XOSC_32MHz  32MHz晶体振荡器
 *             RC_16MHz    16MHz RC振荡器
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void SystemClockSourceSelect(enum SYSCLK_SRC source)
{
  unsigned char osc32k_bm = CLKCONCMD & 0x80;
  unsigned char __clkconcmd,__clkconsta;
      
  /*
     系统时钟源(主时钟源)选择16MHz RC振荡器，定时器tick设置为16MHz，时钟速度设置为16MHz
     CLKCONCMD.OSC32K[b7]不改变      32KHz时钟源选择保持先前设置
     CLKCONCMD.OSC[b6] = 1           系统时钟源(主时钟源)选择16MHz RC振荡器
     CLKCONCMD.TICKSPD[b5..b3] = 001 定时器tick设置为16MHz
     CLKCONCMD.CLKSPD[b2..b0] = 001  时钟速度设置为16MHz
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
     系统时钟源(主时钟源)选择32MHz晶体振荡器，定时器tick设置为32MHz，时钟速度设置为32MHz
     CLKCONCMD.OSC32K[b7]不改变      32KHz时钟源选择保持先前设置
     CLKCONCMD.OSC[b6] = 0           系统时钟源(主时钟源)选择32MHz晶体振荡器
     CLKCONCMD.TICKSPD[b5..b3] = 000 定时器tick设置为32MHz
     CLKCONCMD.CLKSPD[b2..b0] = 000  时钟速度设置为32MHz
   */  
  else if(source == XOSC_32MHz)
  {
    CLKCONCMD = (osc32k_bm /*| (0x00<<6) | (0x00<<3) | (0x00 << 0)*/);
  }
  
  
  /* 等待所选择的系统时钟源(主时钟源)稳定 */
  __clkconcmd = CLKCONCMD;             // 读取时钟控制寄存器CLKCONCMD
  do
  {
    __clkconsta = CLKCONSTA;           // 读取时钟状态寄存器CLKCONSTA
  }while(__clkconsta != __clkconcmd);  // 直到CLKCONSTA寄存器的值与CLKCONCMD寄存
                                       // 器的值一致，说明所选择的系统时钟源(主
                                       // 时钟源)已经稳定  
}


/*********************************************************************
 * 函数名称：main
 * 功    能：main函数入口
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void main(void)
{
  value humi_val,temp_val;
  float dew_point;
  unsigned char error,checksum;
  char ss[20];
  
  SystemClockSourceSelect(XOSC_32MHz);  // 选择32MHz晶体振荡器作为系统时钟源(主时钟源)
  
  HalLcdInit();          // LCD初始化
  HalLcd_HW_Clear();     // 清屏 
  
  /* 在LCD上显示相关信息 */
  HalLcdWriteString("CC253x - Humi&Temp", HAL_LCD_LINE_1); // 温湿度传感器实验
  delayMS(20);
  
  SHT11_Init();        // SHT11初始化
  
  while(1)
  { 
    /* 控制SHT11进行温湿度测量 */
    error = 0;
    error += SHT11_Measure((unsigned char*) &humi_val.i,&checksum,SHT11_HUMI); // 测量相对湿度
    error += SHT11_Measure((unsigned char*) &temp_val.i,&checksum,SHT11_TEMP); // 测量温度
    
    /* 若SHT11测量错误 */
    if(error != 0) 
    {
      SHT11_ConnectionReset();  // 通信复位                          
    }
    else
    /* 若SHT11测量正常 */
    {
      /* 根据测量结果计算实际物理量值 */
      humi_val.f = (float)humi_val.i;                         // 数据类型转换
      temp_val.f = (float)temp_val.i;
      SHT11_CALC(&humi_val.f,&temp_val.f);                    // 计算相对湿度和温度
      dew_point = SHT11_CALCDewpoint(humi_val.f,temp_val.f);  // 计算露点

      /* 
         显示测量结果 
      */
      /* 显示温度 */
      sprintf(ss,(char *)"Temp - %5.1fC",temp_val.f);
      HalLcdWriteString((char *)ss, HAL_LCD_LINE_3);
      
      /* 显示湿度 */
      sprintf(ss,(char *)"Humi - %5.1f%%",humi_val.f);
      HalLcdWriteString((char *)ss, HAL_LCD_LINE_4);
      
      /* 显示露点 */
      sprintf(ss,(char *)"Dew_p - %5.1fC",dew_point);
      HalLcdWriteString((char *)ss, HAL_LCD_LINE_5);
    }
  }
}