/*******************************************************************************
 * �ļ����ƣ�SHT11.h
 * ��    �ܣ�SHT11��ʪ�ȴ�����������ͷ�ļ�
 ******************************************************************************/

#ifndef _SHT11_H_
#define _SHT11_H_


/* ����ͷ�ļ� */
/********************************************************************/
#include "ioCC2530.h"  // CC2530��ͷ�ļ���������CC2530�ļĴ������ж������ȵĶ���
#include "math.h"      // ������ѧ��ͷ�ļ�
#include "hal_cc8051.h"//
/********************************************************************/

/* �궨�� */
/*===================================================*/
/* 
   ����SHT11��������Ŷ��� 
 */
#define SHT11_PROTSEL           P1SEL
#define SHT11_PORTDIR           P1DIR
#define SCK_BIT                (BIT6)
#define DATA_BIT               (BIT7)


#define SHT11_SCK              P1_6
#define SHT11_DATA             P1_7

/* 
   SHT11�������� 
 */
#define SHT11_CMD_STATUS_REG_W 0x06  // "д״̬�Ĵ���"����
#define SHT11_CMD_STATUS_REG_R 0x07  // "��״̬�Ĵ���"����
#define SHT11_CMD_MEASURE_TEMP 0x03  // "�����¶�"����
#define SHT11_CMD_MEASURE_HUMI 0x05  // "�������ʪ��"����
#define SHT11_CMD_RESET        0x1e  // "��λ"����

/* 
   SHT11Ӧ���־���� 
 */
#define SHT11_noACK            0     // ��Ӧ���־
#define SHT11_ACK              1     // Ӧ���־
/*===================================================*/


/* ����ö������ */
/*===================================================*/
enum {SHT11_TEMP,SHT11_HUMI};  // �¶ȣ����ʪ��ö������
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

