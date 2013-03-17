/*******************************************************************************
 * 文件名称：hal_lcd.h
 * 功    能：LCD驱动
 *           使用硬件SPI总线驱动128*64点阵图形液晶（MzLH04-12864）
 * 硬件连接：液晶模块与CC2530的硬件连接关系如下：
 *                液晶模块                         CC2530  
 *               CS(PIN2)                           P1.2(GPIO)
 *               SDA(PIN3)                          P1.6(MOSI) 
 *               SCK(PIN5)                          P1.5(SCLK)
 *               RESET(PIN6)                        P0.0(GPIO)
 *               VDD(PIN1)                           x    
 *               NC(PIN4)                            x     
 *               VSS(PIN7)                           x  
 *
 * 作    者：POWER
 * 公    司：湘潭斯凯电子科技有限公司
 *           www.sikai-tech.com
 * 日    期：2009-09-09
 ******************************************************************************/


#ifndef HAL_LCD_H
#define HAL_LCD_H

#ifdef __cplusplus
extern "C"
{
#endif


/* 包含头文件 */
/********************************************************************/
#include "ioCC2530.h"
/********************************************************************/


/* LCD显示行定义 */
/********************************************************************/
#define HAL_LCD_LINE_1      0x01
#define HAL_LCD_LINE_2      0x02
#define HAL_LCD_LINE_3      0x03
#define HAL_LCD_LINE_4      0x04
#define HAL_LCD_LINE_5      0x05
#define HAL_LCD_LINE_6      0x06

/* 一行显示的最大字符数 */
#define HAL_LCD_MAX_CHARS   21
#define HAL_LCD_MAX_BUFF    25
/********************************************************************/

  
  

/*********************************************************************
 * 函数名称：HalLcdInit
 * 功    能：LCD初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcdInit(void);


/*********************************************************************
 * 函数名称：HalLcdWriteString
 * 功    能：写一串字符到LCD
 * 入口参数：str    被写入的字符串
 *           option 写入选项，这里指写入的行
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcdWriteString(char *str, unsigned char option);


/*********************************************************************
 * 函数名称：HalLcd_HW_Clear
 * 功    能：LCD硬件清屏
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Clear(void);


#ifdef __cplusplus
}
#endif

#endif
