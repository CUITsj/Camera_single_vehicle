/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_it.c
 * @brief      ɽ��K60 ƽ̨�жϷ�����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */

#include    "MK60_it.h"
#include "common.h"
#include "include.h"

/*********************************�жϷ�����********************************/
/*!
 *  @brief      UART4�жϷ�����
 *  @since      v5.0
 */

void uart4_handler()
{
   /*char str[2];

    if(uart_querychar (UART4, str) == 1)   //�������ݼĴ�����
    {
      if (str[0] == 's')
      {
          uart_putchar (UART4, 'Y');
      }
    }*/
  
}
void uart3_handler()
{
 
  
}
void PORTA_IRQHandler()
{
  uint8 n;
  uint32 flag;
  
  flag = PORTA_ISFR;
  PORTA_ISFR = ~0;
  
  n=29;
  if(flag & (1<<n))
  {
    camera_vsync();
  }
#if (CAMERA_USE_HREF == 1)
  n=28;
  if(flag & (1<<n))
  {
    camera_href();
  }
#endif
}

void PORTE_IRQHandler()
{
  uint8 n;
  uint32 flag;
  
  flag = PORTE_ISFR;
  PORTE_ISFR = ~0;
  
  n = 27;
  if (flag & (1<<n))
  {
    nrf_handler();
  }
}

void DMA0_IRQHandler()
{
  camera_dma();
}

void PIT0_IRQHandler()
{
  speed_measure(); 
  PIT_Flag_Clear(PIT0);
}

void PIT1_IRQHandler()
{
  static uint8 time_5s = 0;
  
  if (check_flag == 0)
  {
     time_5s++;
  }
  if (time_5s == 100)//��ʱ5�� 
  {
    time_5s = 0;
    start_flag = 0;
    check_flag = 1;
  }
  PIT_Flag_Clear(PIT1);
}

/*void PIT2_IRQHandler()
{
  //vcan_sendware(var, sizeof(var));
  PIT_Flag_Clear(PIT2);
}*/