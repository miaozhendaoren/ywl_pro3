/*******************************************************************************
 * 文件名称：hal_lcd.c
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


/* 包含头文件 */
/********************************************************************/
#include "LCD.h"
#include "string.h"
/********************************************************************/

#ifndef BV
#define BV(n)      (1 << (n))
#endif

/* 防止编译器报错 */
#define st(x)      do { x } while (__LINE__ == -1)


/* LCD管脚宏定义 */
/********************************************************************/
#define HAL_LCD_RESET_PORT          0  
#define HAL_LCD_RESET_PIN           0

#define HAL_LCD_CS_PORT             1
#define HAL_LCD_CS_PIN              2

#define HAL_LCD_CLK_PORT            1
#define HAL_LCD_CLK_PIN             5

#define HAL_LCD_MOSI_PORT           1
#define HAL_LCD_MOSI_PIN            6

#define HAL_LCD_MISO_PORT           1
#define HAL_LCD_MISO_PIN            7

/* SPI 配置 */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_POL_HI       0x80
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_CLOCK_PHA_1        0x40
#define HAL_SPI_TRANSFER_MSB_LAST  0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20

/* LCD显示定义 */
#define LCD_MAX_LINE_COUNT          6     //最大行数
#define LCD_MAX_LINE_LENGTH         21    //每行最大显示字符数
#define LCD_MAX_BUF                 25

/* 字体设置 */
#define LCD_X_WITCH                 6
#define LCD_Y_WITCH                 10

#define FUNCTION_SET(options)       HalLcd_HW_Control(options)//LCD控制数据写入

/* 设置显示开始行 */
#define LINE1                       0x00
#define LINE2                       0x01
#define LINE3                       0x02
#define LINE4                       0x03

/* IO设置 */
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )

/* 设置IO输出 */
#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin## = val; \
                                                      P##port##DIR |= BV(pin); )
/* 设置IO为功能端口 */
#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )

/* SPI接口控制 */
#define LCD_SPI_BEGIN()     HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  0);
#define LCD_SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  1); /* chip select */         \
}

/* 清空接收和发送字节状态，往缓冲区里写入发送数据，等待传输完成 */
#define LCD_SPI_TX(x)         { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define LCD_SPI_WAIT_RXRDY()  { while(!(U1CSR & BV(1))); }

/* LCD复位 */
#define LCD_ACTIVATE_RESET()  HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 0);
#define LCD_RELEASE_RESET()   HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
/********************************************************************/


/* 本地函数声明 */
/********************************************************************/
#if (HAL_LCD == TRUE)
void HalLcd_HW_Init(void);
void HalLcd_HW_WaitUs(unsigned int i);
void HalLcd_HW_Clear(void);
void HalLcd_HW_SetBackLight(unsigned char Deg) ;
void HalLcd_HW_FontCharSet(unsigned char Font_NUM,unsigned char Color);
void HalLcd_HW_FontMode(unsigned char Cover,unsigned char Color);
void HalLcd_HW_Control(unsigned char cmd);
void HalLcd_HW_Write(unsigned char x,unsigned char y,unsigned char a);
void HalLcd_HW_WriteChar(unsigned char line, unsigned char col, char text);
void HalLcd_HW_WriteLine(unsigned char line, const char *pText);
#endif 
/********************************************************************/



/*********************************************************************
 * 函数名称：HalLcdInit
 * 功    能：LCD初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcdInit(void)
{
#if(HAL_LCD == TRUE)
  HalLcd_HW_Init();
#endif
}


/*********************************************************************
 * 函数名称：HalLcdWriteString
 * 功    能：写一串字符到LCD
 * 入口参数：str    被写入的字符串
 *           option 写入选项，这里指写入的行
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcdWriteString(char *str, unsigned char option)
{
  HalLcd_HW_WriteLine (option, str);
}


#if (HAL_LCD == TRUE)
/*********************************************************************
 * 函数名称：halLcd_ConfigIO
 * 功    能：LCD显示IO配置
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void halLcd_ConfigIO(void)
{
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT, HAL_LCD_CS_PIN, 1);
}


/*********************************************************************
 * 函数名称：halLcd_ConfigSPI
 * 功    能：LCD显示SPI总线配置
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
static void halLcd_ConfigSPI(void)
{
  unsigned char baud_exponent;
  unsigned char baud_mantissa;

  PERCFG |= 0x02;   // 配置SPI为UART1,alt2

  /* 配置CLK MOSI 等IO口 */
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);


  /* 设置 SPI 速度为1MHz(系统时钟为32MHz) */ 
  baud_exponent = 14;
  baud_mantissa =  0;

  /* 配置SPI总线各项具体参数 */
  U1UCR  = 0x80;  //Flush 和IDLE状态
  U1CSR &= ~0xA0; // SPI 主模式
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_1 | HAL_SPI_CLOCK_POL_HI | baud_exponent;
  U1BAUD = baud_mantissa;
}
 

/*********************************************************************
 * 函数名称：HalLcd_HW_Init
 * 功    能：LCD硬件初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Init(void)
{
  halLcd_ConfigIO(); // 初始化LCD IO口
  halLcd_ConfigSPI();// 初始化 SPI 口

  /* 复位LCD */
  LCD_ACTIVATE_RESET();
  HalLcd_HW_WaitUs(8000);
  LCD_RELEASE_RESET();
  HalLcd_HW_WaitUs(30000); 
  HalLcd_HW_WaitUs(30000); 

  HalLcd_HW_SetBackLight(0); // 设置背光强度
  HalLcd_HW_Clear();         // 清屏
  HalLcd_HW_SetBackLight(100);
	
  HalLcd_HW_FontCharSet(0,1);// 字体设置
  HalLcd_HW_FontMode(1,0);
}


/*********************************************************************
 * 函数名称：HalLcd_HW_Control
 * 功    能：LCD硬件写入控制
 * 入口参数：cmd   写入的命令或者数据
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Control(unsigned char cmd)
{
  LCD_SPI_BEGIN();
  LCD_SPI_TX(cmd);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_END();
}


/*********************************************************************
 * 函数名称：HalLcd_HW_Write
 * 功    能：在x y 地址上写入数据
 * 入口参数：x   x地址
 *           y   y地址
 *           a   写入的数据
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Write(unsigned char x,unsigned char y,unsigned char a)
{
  LCD_SPI_BEGIN();	//SS置低电平	
  FUNCTION_SET(0x07);	//传送指令0x07
  FUNCTION_SET(x);	//要显示字符的左上角的X轴位置
  FUNCTION_SET(y);	//要显示字符的左上角的Y轴位置
  FUNCTION_SET(a);	//要显示字符ASCII字符的ASCII码值
  LCD_SPI_END();	//完成操作置SS高电平
}


/*********************************************************************
 * 函数名称：HalLcd_HW_Clear
 * 功    能：LCD硬件清屏
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_Clear(void)
{
  //清屏操作
  LCD_SPI_BEGIN();	//SS置低电平
  FUNCTION_SET(0x80);	//送指令0x80
  LCD_SPI_END();	//完成操作置SS高电平
}


/********************************************************************
 * 函数名称：HalLcd_HW_SetBackLight
 * 功    能：设置背光亮度等级
 * 入口参数：Deg  亮度等级0~127    
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_SetBackLight(unsigned char Deg) 
{
  LCD_SPI_BEGIN();	
  FUNCTION_SET(0x8A);	//传送指令0x8A
  FUNCTION_SET(Deg);	//背光设置亮度值
  LCD_SPI_END();	
}


/*********************************************************************
 * 函数名称：HalLcd_HW_FontCharSet
 * 功    能：ASCII字符字体设置
 * 入口参数：Font_NUM  字体选择,以驱动所带的字库为准
 *           Color     文本颜色,仅作用于ASCII字库
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_FontCharSet(unsigned char Font_NUM,unsigned char Color)
{
  unsigned char ucTemp = 0;
	
  ucTemp = (Font_NUM<<4)|Color;

  LCD_SPI_BEGIN();	//SS置低电平			
  FUNCTION_SET(0x81);	//传送指令0x81
  FUNCTION_SET(ucTemp); //选择6X10的ASCII字体,字符色为黑色
  LCD_SPI_END();	//完成操作置SS高电平	
}


/*********************************************************************
 * 函数名称：HalLcd_HW_FontMode
 * 功    能：设置字符显示覆盖模式
 * 入口参数：Cover  字符覆盖模式设置，0或1
 *           Color  覆盖模式为1时字符显示时的背景覆盖色 
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_FontMode(unsigned char Cover,unsigned char Color)
{
  unsigned char ucTemp=0;
  
  ucTemp = (Cover<<4)|Color;
	
  LCD_SPI_BEGIN();	//SS置低电平			
  FUNCTION_SET(0x89);	//传送指令0x89
  FUNCTION_SET(ucTemp); //发送设置值
  LCD_SPI_END();	//完成操作置SS高电平	
}


/*********************************************************************
 * 函数名称：HalLcd_HW_WriteChar
 * 功    能：显示ASCII码字符
 * 入口参数：line  要显示的字符的行位置
 *           col   要显示的字符列位置
 *           text  要显示的ASCII码字符值
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_WriteChar(unsigned char line, unsigned char col, char text)
{
  if (col < LCD_MAX_LINE_LENGTH)
  {
    HalLcd_HW_Write(col * LCD_X_WITCH, (line-1) * LCD_Y_WITCH, text);
  }
  else
  {
    return;
  }
}


/*********************************************************************
 * 函数名称：HalLcd_HW_WriteLine
 * 功    能：LCD显示一行字符串
 * 入口参数：line  要显示的字符的行位置
 *           pText 要显示的字符串
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_WriteLine(unsigned char line, const char *pText)
{
  unsigned char count;
  unsigned char totalLength = (unsigned char)strlen((char *)pText);

  /* 首先写入字符 */
  for (count=0; count<totalLength; count++)
  {
    HalLcd_HW_WriteChar(line, count, (*(pText++)));
  }

  /* 在尾部写入空白 */
  for(count=totalLength; count<LCD_MAX_LINE_LENGTH;count++)
  {
    HalLcd_HW_WriteChar(line, count, ' ');
  }
}


/*********************************************************************
 * 函数名称：HalLcd_HW_WaitUs
 * 功    能：LCD延时函数
 * 入口参数：microSecs  要延时的微妙数
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void HalLcd_HW_WaitUs(unsigned int microSecs)
{
  while(microSecs--)
  {
    /* 32 NOP为1微妙 */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

#endif




