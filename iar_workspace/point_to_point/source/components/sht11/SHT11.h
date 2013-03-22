/*******************************************************************************
 * 文件名称：SHT11.h
 * 功    能：SHT11温湿度传感器驱动的头文件
 ******************************************************************************/

#ifndef _SHT11_H_
#define _SHT11_H_


/* 包含头文件 */
/********************************************************************/
#include "ioCC2530.h"  // CC2530的头文件，包含对CC2530的寄存器、中断向量等的定义
#include "math.h"      // 包含数学库头文件
#include "hal_cc8051.h"//
/********************************************************************/

/* 宏定义 */
/*===================================================*/
/* 
   控制SHT11的相关引脚定义 
 */
#define SHT11_PROTSEL           P1SEL
#define SHT11_PORTDIR           P1DIR
#define SCK_BIT                (BIT6)
#define DATA_BIT               (BIT7)


#define SHT11_SCK              P1_6
#define SHT11_DATA             P1_7

/* 
   SHT11相关命令定义 
 */
#define SHT11_CMD_STATUS_REG_W 0x06  // "写状态寄存器"命令
#define SHT11_CMD_STATUS_REG_R 0x07  // "读状态寄存器"命令
#define SHT11_CMD_MEASURE_TEMP 0x03  // "测量温度"命令
#define SHT11_CMD_MEASURE_HUMI 0x05  // "测量相对湿度"命令
#define SHT11_CMD_RESET        0x1e  // "复位"命令

/* 
   SHT11应答标志定义 
 */
#define SHT11_noACK            0     // 不应答标志
#define SHT11_ACK              1     // 应答标志
/*===================================================*/


/* 定义枚举类型 */
/*===================================================*/
enum {SHT11_TEMP,SHT11_HUMI};  // 温度，相对湿度枚举类型
/*===================================================*/

#ifdef SHT11
void SHT11_ConnectionReset(void);
void SHT11_Init(void);
char SHT11_SoftReset(void);
char SHT11_Measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode);
void SHT11_CALC(float *p_humidity ,float *p_temperature);
float SHT11_CALCDewpoint(float h,float t);
#else
 
extern unsigned char gf_isSht11DataReady;
extern float gf_temp;
extern float gf_humi;
extern float gf_dewPoint;
extern void SHT11_Init(void);
extern void sht11MakeData(void);

#endif



#endif

