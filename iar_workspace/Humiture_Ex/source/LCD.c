/*******************************************************************************
 * �ļ����ƣ�hal_lcd.c
 * ��    �ܣ�LCD����
 *           ʹ��Ӳ��SPI��������128*64����ͼ��Һ����MzLH04-12864��
 * Ӳ�����ӣ�Һ��ģ����CC2530��Ӳ�����ӹ�ϵ���£�
 *                Һ��ģ��                         CC2530  
 *               CS(PIN2)                           P1.2(GPIO)
 *               SDA(PIN3)                          P1.6(MOSI) 
 *               SCK(PIN5)                          P1.5(SCLK)
 *               RESET(PIN6)                        P0.0(GPIO)
 *               VDD(PIN1)                           x    
 *               NC(PIN4)                            x     
 *               VSS(PIN7)                           x  
 *
 * ��    �ߣ�POWER
 * ��    ˾����̶˹�����ӿƼ����޹�˾
 *           www.sikai-tech.com
 * ��    �ڣ�2009-09-09
 ******************************************************************************/


/* ����ͷ�ļ� */
/********************************************************************/
#include "LCD.h"
#include "string.h"
/********************************************************************/

#ifndef BV
#define BV(n)      (1 << (n))
#endif

/* ��ֹ���������� */
#define st(x)      do { x } while (__LINE__ == -1)


/* LCD�ܽź궨�� */
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

/* SPI ���� */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_POL_HI       0x80
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_CLOCK_PHA_1        0x40
#define HAL_SPI_TRANSFER_MSB_LAST  0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20

/* LCD��ʾ���� */
#define LCD_MAX_LINE_COUNT          6     //�������
#define LCD_MAX_LINE_LENGTH         21    //ÿ�������ʾ�ַ���
#define LCD_MAX_BUF                 25

/* �������� */
#define LCD_X_WITCH                 6
#define LCD_Y_WITCH                 10

#define FUNCTION_SET(options)       HalLcd_HW_Control(options)//LCD��������д��

/* ������ʾ��ʼ�� */
#define LINE1                       0x00
#define LINE2                       0x01
#define LINE3                       0x02
#define LINE4                       0x03

/* IO���� */
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )

/* ����IO��� */
#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin## = val; \
                                                      P##port##DIR |= BV(pin); )
/* ����IOΪ���ܶ˿� */
#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )

/* SPI�ӿڿ��� */
#define LCD_SPI_BEGIN()     HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  0);
#define LCD_SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  1); /* chip select */         \
}

/* ��ս��պͷ����ֽ�״̬������������д�뷢�����ݣ��ȴ�������� */
#define LCD_SPI_TX(x)         { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define LCD_SPI_WAIT_RXRDY()  { while(!(U1CSR & BV(1))); }

/* LCD��λ */
#define LCD_ACTIVATE_RESET()  HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 0);
#define LCD_RELEASE_RESET()   HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
/********************************************************************/


/* ���غ������� */
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
 * �������ƣ�HalLcdInit
 * ��    �ܣ�LCD��ʼ��
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcdInit(void)
{
#if(HAL_LCD == TRUE)
  HalLcd_HW_Init();
#endif
}


/*********************************************************************
 * �������ƣ�HalLcdWriteString
 * ��    �ܣ�дһ���ַ���LCD
 * ��ڲ�����str    ��д����ַ���
 *           option д��ѡ�����ָд�����
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcdWriteString(char *str, unsigned char option)
{
  HalLcd_HW_WriteLine (option, str);
}


#if (HAL_LCD == TRUE)
/*********************************************************************
 * �������ƣ�halLcd_ConfigIO
 * ��    �ܣ�LCD��ʾIO����
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
static void halLcd_ConfigIO(void)
{
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT, HAL_LCD_CS_PIN, 1);
}


/*********************************************************************
 * �������ƣ�halLcd_ConfigSPI
 * ��    �ܣ�LCD��ʾSPI��������
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
static void halLcd_ConfigSPI(void)
{
  unsigned char baud_exponent;
  unsigned char baud_mantissa;

  PERCFG |= 0x02;   // ����SPIΪUART1,alt2

  /* ����CLK MOSI ��IO�� */
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);


  /* ���� SPI �ٶ�Ϊ1MHz(ϵͳʱ��Ϊ32MHz) */ 
  baud_exponent = 14;
  baud_mantissa =  0;

  /* ����SPI���߸��������� */
  U1UCR  = 0x80;  //Flush ��IDLE״̬
  U1CSR &= ~0xA0; // SPI ��ģʽ
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_1 | HAL_SPI_CLOCK_POL_HI | baud_exponent;
  U1BAUD = baud_mantissa;
}
 

/*********************************************************************
 * �������ƣ�HalLcd_HW_Init
 * ��    �ܣ�LCDӲ����ʼ��
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_Init(void)
{
  halLcd_ConfigIO(); // ��ʼ��LCD IO��
  halLcd_ConfigSPI();// ��ʼ�� SPI ��

  /* ��λLCD */
  LCD_ACTIVATE_RESET();
  HalLcd_HW_WaitUs(8000);
  LCD_RELEASE_RESET();
  HalLcd_HW_WaitUs(30000); 
  HalLcd_HW_WaitUs(30000); 

  HalLcd_HW_SetBackLight(0); // ���ñ���ǿ��
  HalLcd_HW_Clear();         // ����
  HalLcd_HW_SetBackLight(100);
	
  HalLcd_HW_FontCharSet(0,1);// ��������
  HalLcd_HW_FontMode(1,0);
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_Control
 * ��    �ܣ�LCDӲ��д�����
 * ��ڲ�����cmd   д��������������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_Control(unsigned char cmd)
{
  LCD_SPI_BEGIN();
  LCD_SPI_TX(cmd);
  LCD_SPI_WAIT_RXRDY();
  LCD_SPI_END();
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_Write
 * ��    �ܣ���x y ��ַ��д������
 * ��ڲ�����x   x��ַ
 *           y   y��ַ
 *           a   д�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_Write(unsigned char x,unsigned char y,unsigned char a)
{
  LCD_SPI_BEGIN();	//SS�õ͵�ƽ	
  FUNCTION_SET(0x07);	//����ָ��0x07
  FUNCTION_SET(x);	//Ҫ��ʾ�ַ������Ͻǵ�X��λ��
  FUNCTION_SET(y);	//Ҫ��ʾ�ַ������Ͻǵ�Y��λ��
  FUNCTION_SET(a);	//Ҫ��ʾ�ַ�ASCII�ַ���ASCII��ֵ
  LCD_SPI_END();	//��ɲ�����SS�ߵ�ƽ
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_Clear
 * ��    �ܣ�LCDӲ������
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_Clear(void)
{
  //��������
  LCD_SPI_BEGIN();	//SS�õ͵�ƽ
  FUNCTION_SET(0x80);	//��ָ��0x80
  LCD_SPI_END();	//��ɲ�����SS�ߵ�ƽ
}


/********************************************************************
 * �������ƣ�HalLcd_HW_SetBackLight
 * ��    �ܣ����ñ������ȵȼ�
 * ��ڲ�����Deg  ���ȵȼ�0~127    
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_SetBackLight(unsigned char Deg) 
{
  LCD_SPI_BEGIN();	
  FUNCTION_SET(0x8A);	//����ָ��0x8A
  FUNCTION_SET(Deg);	//������������ֵ
  LCD_SPI_END();	
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_FontCharSet
 * ��    �ܣ�ASCII�ַ���������
 * ��ڲ�����Font_NUM  ����ѡ��,�������������ֿ�Ϊ׼
 *           Color     �ı���ɫ,��������ASCII�ֿ�
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_FontCharSet(unsigned char Font_NUM,unsigned char Color)
{
  unsigned char ucTemp = 0;
	
  ucTemp = (Font_NUM<<4)|Color;

  LCD_SPI_BEGIN();	//SS�õ͵�ƽ			
  FUNCTION_SET(0x81);	//����ָ��0x81
  FUNCTION_SET(ucTemp); //ѡ��6X10��ASCII����,�ַ�ɫΪ��ɫ
  LCD_SPI_END();	//��ɲ�����SS�ߵ�ƽ	
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_FontMode
 * ��    �ܣ������ַ���ʾ����ģʽ
 * ��ڲ�����Cover  �ַ�����ģʽ���ã�0��1
 *           Color  ����ģʽΪ1ʱ�ַ���ʾʱ�ı�������ɫ 
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_FontMode(unsigned char Cover,unsigned char Color)
{
  unsigned char ucTemp=0;
  
  ucTemp = (Cover<<4)|Color;
	
  LCD_SPI_BEGIN();	//SS�õ͵�ƽ			
  FUNCTION_SET(0x89);	//����ָ��0x89
  FUNCTION_SET(ucTemp); //��������ֵ
  LCD_SPI_END();	//��ɲ�����SS�ߵ�ƽ	
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_WriteChar
 * ��    �ܣ���ʾASCII���ַ�
 * ��ڲ�����line  Ҫ��ʾ���ַ�����λ��
 *           col   Ҫ��ʾ���ַ���λ��
 *           text  Ҫ��ʾ��ASCII���ַ�ֵ
 * ���ڲ�������
 * �� �� ֵ����
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
 * �������ƣ�HalLcd_HW_WriteLine
 * ��    �ܣ�LCD��ʾһ���ַ���
 * ��ڲ�����line  Ҫ��ʾ���ַ�����λ��
 *           pText Ҫ��ʾ���ַ���
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_WriteLine(unsigned char line, const char *pText)
{
  unsigned char count;
  unsigned char totalLength = (unsigned char)strlen((char *)pText);

  /* ����д���ַ� */
  for (count=0; count<totalLength; count++)
  {
    HalLcd_HW_WriteChar(line, count, (*(pText++)));
  }

  /* ��β��д��հ� */
  for(count=totalLength; count<LCD_MAX_LINE_LENGTH;count++)
  {
    HalLcd_HW_WriteChar(line, count, ' ');
  }
}


/*********************************************************************
 * �������ƣ�HalLcd_HW_WaitUs
 * ��    �ܣ�LCD��ʱ����
 * ��ڲ�����microSecs  Ҫ��ʱ��΢����
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void HalLcd_HW_WaitUs(unsigned int microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPΪ1΢�� */
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




