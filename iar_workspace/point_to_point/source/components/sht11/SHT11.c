/*******************************************************************************
 * 文件名称：SHT11.c
 * 功    能：SHT11温湿度传感器驱动
 * 硬件连接：SHT11与CC253x的硬件连接关系如下：
 *
 *           SHT11                  CC253x
 *           SCK                     P0.0
 *           DATA                    P0.7
 *
 ******************************************************************************/

/* 包含头文件 */
/********************************************************************/
#include "SHT11.h"  // CC2530的头文件，包含对CC2530的寄存器、中断向量等的定义
/********************************************************************/


/*********************************************************************
 * 函数名称：SHT11_IO_Init
 * 功    能：对控制SHT11的IO进行初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void SHT11_IO_Init(void)
{
  /* 
     由于CC2530上电默认配置所有IO为方向为输入的通用IO，
     因此此处不再配置控制SHT11的IO为通用IO。此处是否需要
     用户自行配置，请根据具体应用情况决定。
   */
   
  //IO口配置为普通IO
  SHT11_PROTSEL &= ~(SCK_BIT + DATA_BIT);  
  
  //上拉配置
  /*上电默认上下拉功能、上拉*/
  
  //IO方向配置为输出
  SHT11_PORTDIR |= (SCK_BIT + DATA_BIT); 

  SHT11_DATA = 1;   // 向数据线输出高电平
  SHT11_SCK = 0;    // 向时钟线输出低电平
}


/*********************************************************************
 * 函数名称：SHT11_WriteByte
 * 功    能：向SHT11写1个字节并检测SHT11是否应答
 * 入口参数：value  要写入的1字节数据
 * 出口参数：无
 * 返 回 值：返回1表明SHT11未应答
 ********************************************************************/
char SHT11_WriteByte(unsigned char value)
{
  unsigned char i,j,error = 0;
  
  SHT11_PORTDIR |= DATA_BIT;  // P0.7（连接SHT11的DATA）方向为输出
  
  /* 从高到低逐位发送 */
  for(i=0x80;i>0;i/=2)                                   
  {
    if(i & value)
    {
      SHT11_DATA = 1;
    }
    else
    {
      SHT11_DATA = 0;    
    }

    SHT11_SCK = 1;
    for(j=0;j<24;j++ )  asm("nop");  // 延时         	
    SHT11_SCK = 0;
  }

  SHT11_DATA = 1;      // 释放DATA线
  SHT11_SCK = 1;       // 第9个SCK

  SHT11_PORTDIR &= ~(DATA_BIT);     // P0.7（连接SHT11的DATA）方向为输入
  error = SHT11_DATA;  // 检查应答 (SHT11将拉底DATA作为应答)
  SHT11_SCK = 0;
  return error;        // error=1表明SHT11未应答
}


/*********************************************************************
 * 函数名称：SHT11_ReadByte
 * 功    能：从SHT11读1个字节并当输入参数ack=1时给出应答
 * 入口参数：ack  应答标志
 * 出口参数：无
 * 返 回 值：返回1表明SHT11未应答
 ********************************************************************/
char SHT11_ReadByte(unsigned char ack)
{
  unsigned char i,j,val = 0;

  SHT11_PORTDIR |= DATA_BIT;   // P0.7（连接SHT11的DATA）方向为输出

  SHT11_DATA=1;    // 释放DATA线

  SHT11_PORTDIR &= ~DATA_BIT;  // P0.7（连接SHT11的DATA）方向为输入
  
  /* 从高到低逐位读取 */
  for(i=0x80;i>0;i/=2)                                    
  {
    SHT11_SCK = 1;
    if(SHT11_DATA) val=(val | i);
    SHT11_SCK = 0;  					
  }

  SHT11_PORTDIR |= DATA_BIT;       // P0.7（连接SHT11的DATA）方向为输出
  SHT11_DATA = !ack;   // 当ack=1时拉底DATA线
  SHT11_SCK = 1;       // 第9个SCK
  for(j=0;j<24;j++ )  asm("nop");  // 延时         	
  SHT11_SCK = 0;						
  SHT11_DATA = 1;      // 释放DATA线
  return val;
}


/*********************************************************************
 * 函数名称：SHT11_TransStart
 * 功    能：向SHT11发送一个"启动传输"序列
 *                 _____         ________
 *           DATA:      |_______|
 *                     ___     ___
 *           SCK : ___|   |___|   |______
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void SHT11_TransStart(void)
{
   unsigned char j;

   SHT11_PORTDIR |= DATA_BIT;   // P0.7（连接SHT11的DATA）方向为输出

   SHT11_DATA = 1; SHT11_SCK = 0;  // 初始状态
   for(j=0;j<8;j++)  asm("nop");         	
   SHT11_SCK = 1;
   for(j=0;j<8;j++)  asm("nop");         	
   SHT11_DATA = 0;
   for(j=0;j<8;j++)  asm("nop");         	
   SHT11_SCK = 0;
   for(j=0;j<24;j++) asm("nop");         	
   SHT11_SCK = 1;
   for(j=0;j<8;j++)  asm("nop");         	
   SHT11_DATA = 1;		
   for(j=0;j<8;j++)  asm("nop");         	
   SHT11_SCK= 0;		
}


/*********************************************************************
 * 函数名称：SHT11_ConnectionReset
 * 功    能：控制SHT11通信复位
 *                 _____________________________________________________         ________
 *           DATA:                                                      |_______|
 *                    _    _    _    _    _    _    _    _    _        ___     ___
 *           SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void SHT11_ConnectionReset(void)
{
  unsigned char i;

  SHT11_PORTDIR |= DATA_BIT;   // P0.7（连接SHT11的DATA）方向为输出
  
  SHT11_DATA = 1; SHT11_SCK = 0;  // 初始状态

  /* 9个SCK 周期*/
  for(i=0;i<9;i++)
  {
    SHT11_SCK = 1;
    SHT11_SCK = 0;
  }

  SHT11_TransStart();  // 发送一个"启动传输"序列
}


/*********************************************************************
 * 函数名称：SHT11_Init
 * 功    能：SHT11初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void SHT11_Init(void)
{
  SHT11_IO_Init();          // 对控制SHT11的IO进行初始化
  SHT11_ConnectionReset();  // 控制SHT11通信复位
}


/*********************************************************************
 * 函数名称：SHT11_SoftReset
 * 功    能：控制SHT11软件复位
 *                 _____________________________________________________         ________
 *           DATA:                                                      |_______|
 *                    _    _    _    _    _    _    _    _    _        ___     ___
 *           SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：返回值为1表示SHT11未响应
 ********************************************************************/
char SHT11_SoftReset(void)
{
  unsigned char error = 0;

  SHT11_ConnectionReset();                    // 通信复位
  error += SHT11_WriteByte(SHT11_CMD_RESET);  // 发送"复位"命令给SHT11
  return error;                               // error=1表示SHT11未响应
}


/*********************************************************************
 * 函数名称：SHT11_ReadStatusREG
 * 功    能：读取状态寄存器和校验和
 * 入口参数：p_value      状态寄存器的值
 *           p_checksum   校验和
 * 出口参数：p_value      状态寄存器的值
 *           p_checksum   校验和
 * 返 回 值：返回值为1表示SHT11未响应
 ********************************************************************/
char SHT11_ReadStatusREG(unsigned char *p_value, unsigned char *p_checksum)
{
  unsigned char error = 0;

  SHT11_TransStart();                               // 发送一个"启动传输"序列
  error = SHT11_WriteByte(SHT11_CMD_STATUS_REG_R);  // 发送"读状态寄存器"命令
  *p_value = SHT11_ReadByte(SHT11_ACK);             // 读状态寄存器
  *p_checksum = SHT11_ReadByte(SHT11_noACK);        // 读校验和

  return error;                                     // error=1表示SHT11未响应
}


/*********************************************************************
 * 函数名称：SHT11_WriteStatusREG
 * 功    能：写状态寄存器
 * 入口参数：p_value      状态寄存器的值
 * 出口参数：无
 * 返 回 值：返回值为1表示SHT11未响应
 ********************************************************************/
char SHT11_WriteStatusREG(unsigned char *p_value)
{
  unsigned char error = 0;

  SHT11_TransStart();                                // 发送一个"启动传输"序列
  error += SHT11_WriteByte(SHT11_CMD_STATUS_REG_W);  // 发送"写状态寄存器"命令
  error += SHT11_WriteByte(*p_value);                // 写状态寄存器

  return error;                                            // error=1表示SHT11未响应
}


/*********************************************************************
 * 函数名称：SHT11_Measure
 * 功    能：进行一次测量(相对湿度或温度)
 * 入口参数：p_value      测量值
 *           checksum     校验和
 *           mode         TEMP表示进行温度测量
 *                        HUMI表示进行相对湿度测量
 * 出口参数：p_value      测量值
 * 返 回 值：返回值为1表示SHT11未响应
 ********************************************************************/
char SHT11_Measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
{
  unsigned error = 0;
  unsigned long i;

  SHT11_TransStart();  // 发送一个"启动传输"序列
  
  /* 根据输入参数mode进行一次相应的测量 */
  switch(mode)                                             
  {
    case SHT11_TEMP : error += SHT11_WriteByte(SHT11_CMD_MEASURE_TEMP); break;
    case SHT11_HUMI : error += SHT11_WriteByte(SHT11_CMD_MEASURE_HUMI); break;
    default     : break;	
  }

  for(i=0;i<165535;i++) if(SHT11_DATA == 0) break;  // 等待SHT11完成测量
  if(SHT11_DATA) error += 1;                        // 测量错误

  *(p_value + 1) = SHT11_ReadByte(SHT11_ACK);       // 读第1个字节 (MSB)
  *(p_value) = SHT11_ReadByte(SHT11_ACK);           // 读第2个字节 (LSB)
  *p_checksum = SHT11_ReadByte(SHT11_noACK);        // 读校验和

  return error;
}


/*********************************************************************
 * 函数名称：SHT11_CALC
 * 功    能：计算相对湿度和温度
 * 入口参数：p_humidity      SHT11采集到的相对湿度值
 *           p_temperature   SHT11采集到的温度值
 * 出口参数：p_humidity      实际物理量的值
 *           p_temperature   采实际物理量的值
 * 返 回 值：无
 ********************************************************************/
void SHT11_CALC(float *p_humidity ,float *p_temperature)
{
  const float C1 = -4.0;                     // 12位
  const float C2 = +0.0405;                  // 12 Bit
  const float C3 = -0.0000028;               // 12 Bit
  const float T1 = +0.01;                    // 14位 5V
  const float T2 = +0.00008;                 // 14位 5V	

  float rh = *p_humidity;                    // 相对湿度采集值 12位
  float t = *p_temperature;                  // 温度采集值 14位
  float rh_lin;                              // 相对湿度的非线性补偿
  float rh_true;                             // 相对湿度物理量值
  float t_C;                                 // 温度物理量值

  t_C = t*0.01 - 40;                         // 计算温度物理量值
  rh_lin = C3*rh*rh + C2*rh + C1;            // 计算相对湿度的非线性补偿
  rh_true = (t_C-25) * (T1+T2*rh) + rh_lin;  // 计算相对湿度物理量值

  /* 若计算出来的相对湿度物理量值超范围则截断 */
  if(rh_true > 100) rh_true = 100;
  if(rh_true < 0.1) rh_true = 0.1;

  *p_temperature = t_C;                      // 返回温度物理量值
  *p_humidity = rh_true;                     // 返回相对湿度物理量值
}


/*********************************************************************
 * 函数名称：SHT11_CALCDewpoint
 * 功    能：计算露点
 * 入口参数：h      相对湿度物理量值
 *           t      温度物理量值
 * 出口参数：无
 * 返 回 值：露点值
 ********************************************************************/
float SHT11_CALCDewpoint(float h,float t)
{
  float logEx,dew_point;

  logEx = 0.66077 + 7.5*t/(237.3 + t) + (log10(h) - 2);
  dew_point = (logEx - 0.66077) * 237.3/(0.66077 + 7.5 - logEx);

  return dew_point;
}






















