/*******************************************************************************
 * �ļ����ƣ�SHT11.c
 * ��    �ܣ�SHT11��ʪ�ȴ���������
 * Ӳ�����ӣ�SHT11��CC253x��Ӳ�����ӹ�ϵ���£�
 *
 *           SHT11                  CC253x
 *           SCK                     P0.0
 *           DATA                    P0.7
 *
 ******************************************************************************/

/* ����ͷ�ļ� */
/********************************************************************/
#include "SHT11.h"  // CC2530��ͷ�ļ���������CC2530�ļĴ������ж������ȵĶ���
/********************************************************************/


/*********************************************************************
 * �������ƣ�SHT11_IO_Init
 * ��    �ܣ��Կ���SHT11��IO���г�ʼ��
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void SHT11_IO_Init(void)
{
  /* 
     ����CC2530�ϵ�Ĭ����������IOΪ����Ϊ�����ͨ��IO��
     ��˴˴��������ÿ���SHT11��IOΪͨ��IO���˴��Ƿ���Ҫ
     �û��������ã�����ݾ���Ӧ�����������
   */
   
  //IO������Ϊ��ͨIO
  SHT11_PROTSEL &= ~(SCK_BIT + DATA_BIT);  
  
  //��������
  /*�ϵ�Ĭ�����������ܡ�����*/
  
  //IO��������Ϊ���
  SHT11_PORTDIR |= (SCK_BIT + DATA_BIT); 

  SHT11_DATA = 1;   // ������������ߵ�ƽ
  SHT11_SCK = 0;    // ��ʱ��������͵�ƽ
}


/*********************************************************************
 * �������ƣ�SHT11_WriteByte
 * ��    �ܣ���SHT11д1���ֽڲ����SHT11�Ƿ�Ӧ��
 * ��ڲ�����value  Ҫд���1�ֽ�����
 * ���ڲ�������
 * �� �� ֵ������1����SHT11δӦ��
 ********************************************************************/
char SHT11_WriteByte(unsigned char value)
{
  unsigned char i,j,error = 0;
  
  SHT11_PORTDIR |= DATA_BIT;  // P0.7������SHT11��DATA������Ϊ���
  
  /* �Ӹߵ�����λ���� */
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
    for(j=0;j<24;j++ )  asm("nop");  // ��ʱ         	
    SHT11_SCK = 0;
  }

  SHT11_DATA = 1;      // �ͷ�DATA��
  SHT11_SCK = 1;       // ��9��SCK

  SHT11_PORTDIR &= ~(DATA_BIT);     // P0.7������SHT11��DATA������Ϊ����
  error = SHT11_DATA;  // ���Ӧ�� (SHT11������DATA��ΪӦ��)
  SHT11_SCK = 0;
  return error;        // error=1����SHT11δӦ��
}


/*********************************************************************
 * �������ƣ�SHT11_ReadByte
 * ��    �ܣ���SHT11��1���ֽڲ����������ack=1ʱ����Ӧ��
 * ��ڲ�����ack  Ӧ���־
 * ���ڲ�������
 * �� �� ֵ������1����SHT11δӦ��
 ********************************************************************/
char SHT11_ReadByte(unsigned char ack)
{
  unsigned char i,j,val = 0;

  SHT11_PORTDIR |= DATA_BIT;   // P0.7������SHT11��DATA������Ϊ���

  SHT11_DATA=1;    // �ͷ�DATA��

  SHT11_PORTDIR &= ~DATA_BIT;  // P0.7������SHT11��DATA������Ϊ����
  
  /* �Ӹߵ�����λ��ȡ */
  for(i=0x80;i>0;i/=2)                                    
  {
    SHT11_SCK = 1;
    if(SHT11_DATA) val=(val | i);
    SHT11_SCK = 0;  					
  }

  SHT11_PORTDIR |= DATA_BIT;       // P0.7������SHT11��DATA������Ϊ���
  SHT11_DATA = !ack;   // ��ack=1ʱ����DATA��
  SHT11_SCK = 1;       // ��9��SCK
  for(j=0;j<24;j++ )  asm("nop");  // ��ʱ         	
  SHT11_SCK = 0;						
  SHT11_DATA = 1;      // �ͷ�DATA��
  return val;
}


/*********************************************************************
 * �������ƣ�SHT11_TransStart
 * ��    �ܣ���SHT11����һ��"��������"����
 *                 _____         ________
 *           DATA:      |_______|
 *                     ___     ___
 *           SCK : ___|   |___|   |______
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void SHT11_TransStart(void)
{
   unsigned char j;

   SHT11_PORTDIR |= DATA_BIT;   // P0.7������SHT11��DATA������Ϊ���

   SHT11_DATA = 1; SHT11_SCK = 0;  // ��ʼ״̬
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
 * �������ƣ�SHT11_ConnectionReset
 * ��    �ܣ�����SHT11ͨ�Ÿ�λ
 *                 _____________________________________________________         ________
 *           DATA:                                                      |_______|
 *                    _    _    _    _    _    _    _    _    _        ___     ___
 *           SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void SHT11_ConnectionReset(void)
{
  unsigned char i;

  SHT11_PORTDIR |= DATA_BIT;   // P0.7������SHT11��DATA������Ϊ���
  
  SHT11_DATA = 1; SHT11_SCK = 0;  // ��ʼ״̬

  /* 9��SCK ����*/
  for(i=0;i<9;i++)
  {
    SHT11_SCK = 1;
    SHT11_SCK = 0;
  }

  SHT11_TransStart();  // ����һ��"��������"����
}


/*********************************************************************
 * �������ƣ�SHT11_Init
 * ��    �ܣ�SHT11��ʼ��
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ����
 ********************************************************************/
void SHT11_Init(void)
{
  SHT11_IO_Init();          // �Կ���SHT11��IO���г�ʼ��
  SHT11_ConnectionReset();  // ����SHT11ͨ�Ÿ�λ
}


/*********************************************************************
 * �������ƣ�SHT11_SoftReset
 * ��    �ܣ�����SHT11�����λ
 *                 _____________________________________________________         ________
 *           DATA:                                                      |_______|
 *                    _    _    _    _    _    _    _    _    _        ___     ___
 *           SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
 * ��ڲ�������
 * ���ڲ�������
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ********************************************************************/
char SHT11_SoftReset(void)
{
  unsigned char error = 0;

  SHT11_ConnectionReset();                    // ͨ�Ÿ�λ
  error += SHT11_WriteByte(SHT11_CMD_RESET);  // ����"��λ"�����SHT11
  return error;                               // error=1��ʾSHT11δ��Ӧ
}


/*********************************************************************
 * �������ƣ�SHT11_ReadStatusREG
 * ��    �ܣ���ȡ״̬�Ĵ�����У���
 * ��ڲ�����p_value      ״̬�Ĵ�����ֵ
 *           p_checksum   У���
 * ���ڲ�����p_value      ״̬�Ĵ�����ֵ
 *           p_checksum   У���
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ********************************************************************/
char SHT11_ReadStatusREG(unsigned char *p_value, unsigned char *p_checksum)
{
  unsigned char error = 0;

  SHT11_TransStart();                               // ����һ��"��������"����
  error = SHT11_WriteByte(SHT11_CMD_STATUS_REG_R);  // ����"��״̬�Ĵ���"����
  *p_value = SHT11_ReadByte(SHT11_ACK);             // ��״̬�Ĵ���
  *p_checksum = SHT11_ReadByte(SHT11_noACK);        // ��У���

  return error;                                     // error=1��ʾSHT11δ��Ӧ
}


/*********************************************************************
 * �������ƣ�SHT11_WriteStatusREG
 * ��    �ܣ�д״̬�Ĵ���
 * ��ڲ�����p_value      ״̬�Ĵ�����ֵ
 * ���ڲ�������
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ********************************************************************/
char SHT11_WriteStatusREG(unsigned char *p_value)
{
  unsigned char error = 0;

  SHT11_TransStart();                                // ����һ��"��������"����
  error += SHT11_WriteByte(SHT11_CMD_STATUS_REG_W);  // ����"д״̬�Ĵ���"����
  error += SHT11_WriteByte(*p_value);                // д״̬�Ĵ���

  return error;                                            // error=1��ʾSHT11δ��Ӧ
}


/*********************************************************************
 * �������ƣ�SHT11_Measure
 * ��    �ܣ�����һ�β���(���ʪ�Ȼ��¶�)
 * ��ڲ�����p_value      ����ֵ
 *           checksum     У���
 *           mode         TEMP��ʾ�����¶Ȳ���
 *                        HUMI��ʾ�������ʪ�Ȳ���
 * ���ڲ�����p_value      ����ֵ
 * �� �� ֵ������ֵΪ1��ʾSHT11δ��Ӧ
 ********************************************************************/
char SHT11_Measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
{
  unsigned error = 0;
  unsigned long i;

  SHT11_TransStart();  // ����һ��"��������"����
  
  /* �����������mode����һ����Ӧ�Ĳ��� */
  switch(mode)                                             
  {
    case SHT11_TEMP : error += SHT11_WriteByte(SHT11_CMD_MEASURE_TEMP); break;
    case SHT11_HUMI : error += SHT11_WriteByte(SHT11_CMD_MEASURE_HUMI); break;
    default     : break;	
  }

  for(i=0;i<165535;i++) if(SHT11_DATA == 0) break;  // �ȴ�SHT11��ɲ���
  if(SHT11_DATA) error += 1;                        // ��������

  *(p_value + 1) = SHT11_ReadByte(SHT11_ACK);       // ����1���ֽ� (MSB)
  *(p_value) = SHT11_ReadByte(SHT11_ACK);           // ����2���ֽ� (LSB)
  *p_checksum = SHT11_ReadByte(SHT11_noACK);        // ��У���

  return error;
}


/*********************************************************************
 * �������ƣ�SHT11_CALC
 * ��    �ܣ��������ʪ�Ⱥ��¶�
 * ��ڲ�����p_humidity      SHT11�ɼ��������ʪ��ֵ
 *           p_temperature   SHT11�ɼ������¶�ֵ
 * ���ڲ�����p_humidity      ʵ����������ֵ
 *           p_temperature   ��ʵ����������ֵ
 * �� �� ֵ����
 ********************************************************************/
void SHT11_CALC(float *p_humidity ,float *p_temperature)
{
  const float C1 = -4.0;                     // 12λ
  const float C2 = +0.0405;                  // 12 Bit
  const float C3 = -0.0000028;               // 12 Bit
  const float T1 = +0.01;                    // 14λ 5V
  const float T2 = +0.00008;                 // 14λ 5V	

  float rh = *p_humidity;                    // ���ʪ�Ȳɼ�ֵ 12λ
  float t = *p_temperature;                  // �¶Ȳɼ�ֵ 14λ
  float rh_lin;                              // ���ʪ�ȵķ����Բ���
  float rh_true;                             // ���ʪ��������ֵ
  float t_C;                                 // �¶�������ֵ

  t_C = t*0.01 - 40;                         // �����¶�������ֵ
  rh_lin = C3*rh*rh + C2*rh + C1;            // �������ʪ�ȵķ����Բ���
  rh_true = (t_C-25) * (T1+T2*rh) + rh_lin;  // �������ʪ��������ֵ

  /* ��������������ʪ��������ֵ����Χ��ض� */
  if(rh_true > 100) rh_true = 100;
  if(rh_true < 0.1) rh_true = 0.1;

  *p_temperature = t_C;                      // �����¶�������ֵ
  *p_humidity = rh_true;                     // �������ʪ��������ֵ
}


/*********************************************************************
 * �������ƣ�SHT11_CALCDewpoint
 * ��    �ܣ�����¶��
 * ��ڲ�����h      ���ʪ��������ֵ
 *           t      �¶�������ֵ
 * ���ڲ�������
 * �� �� ֵ��¶��ֵ
 ********************************************************************/
float SHT11_CALCDewpoint(float h,float t)
{
  float logEx,dew_point;

  logEx = 0.66077 + 7.5*t/(237.3 + t) + (log10(h) - 2);
  dew_point = (logEx - 0.66077) * 237.3/(0.66077 + 7.5 - logEx);

  return dew_point;
}






















