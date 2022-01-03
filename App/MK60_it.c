/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_it.c
 * @brief      山外K60 平台中断服务函数
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */

#include    "MK60_it.h"
#include "common.h"
#include "include.h"

/*********************************中断服务函数********************************/
/*!
 *  @brief      UART4中断服务函数
 *  @since      v5.0
 */

void uart4_handler()
{
   /*char str[2];

    if(uart_querychar (UART4, str) == 1)   //接收数据寄存器满
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
  if (time_5s == 100)//定时5秒 
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