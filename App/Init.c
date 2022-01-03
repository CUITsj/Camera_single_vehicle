#include "Init.h"

uint8 nrf_rx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];
uint8 nrf_tx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];

void All_init()
{
  /************************ 中断优先级的配置  ***********************/
  NVIC_SetPriorityGrouping(4);//参数范围（0~4）4表示4bit全为抢占优先级，没有压优先级
  NVIC_SetPriority(PORTA_IRQn, 0);  //参数越小优先级越高，摄像头
  NVIC_SetPriority(PORTE_IRQn, 1);  //NRF
  NVIC_SetPriority(DMA0_IRQn, 2);  //摄像头
  NVIC_SetPriority(PIT0_IRQn, 3);  //定时器
  NVIC_SetPriority(PIT1_IRQn, 4);  //定时器
  
  
  /************************** LED 初始化  ***************************/
  //led_init(LED_MAX);
  
  /************************ LCD 液晶屏 初始化  ***********************/
  
  
  /************************ 舵机初始化  ***********************/
  ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ, S3010_MID);//占空比为 S3010_MID/FTM3_PRECISON,FTM3_PRECISON为10000u,在MK60_FTM.h中定义
  
  /************************ 电机初始化静止  ***********************/
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, MOTOR1_DUTY); //占空比为0
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 0);
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, MOTOR2_DUTY);
  
  /************************ 摄像头 初始化  ***********************/
  camera_init(imgbuff);  //把图像采集到imgbuff里面
  //配置中断服务函数
  set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);
  set_vector_handler(DMA0_VECTORn, DMA0_IRQHandler);
  
  /************************ 编码器初始化  ***********************/   
  ftm_quad_init(FTM1);
  ftm_quad_init(FTM2);
  /************************** 蓝牙串口 初始化  ***********************/
 /* uart_init(UART3, 9600);
  uart_putchar(UART3, 0xff);    //发送一次结束符，清除上电产生的串口杂波数据
  uart_rx_irq_en(UART3);
  set_vector_handler(UART3_RX_TX_VECTORn , uart3_handler);
  enable_irq(UART3_RX_TX_IRQn );*/
  /************************ 无线模块 NRF 初始化  ***********************/
/*#if NRF_OFF
  uint32 i = 20;
  while(!nrf_init());
  //配置中断服务函数
  set_vector_handler(PORTE_VECTORn, PORTE_IRQHandler);
  enable_irq(PORTE_IRQn);
  nrf_msg_init();
  
  while(i--)
  {
    nrf_msg_tx(COM_RETRAN, nrf_tx_buff);// COM_RETRAN复位传输，丢弃之前接收到的数据
  }
#endif*/
  
  /******************** 定时器 初始化  *********************/ 
  pit_init_ms(PIT0, 10);    //每10ms测量一次车速
  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
  enable_irq(PIT0_IRQn);
  
  pit_init_ms(PIT1, 50);    //用于停车
  set_vector_handler(PIT1_VECTORn, PIT1_IRQHandler);
  enable_irq(PIT1_IRQn);
  
 /* pit_init_ms(PIT2, 50);    //用于串口
  set_vector_handler(PIT2_VECTORn, PIT2_IRQHandler);
  enable_irq(PIT2_IRQn);*/
  
  /********************** PID参数初始化 ***********************/
  S3010PID_Init(&S3010_PID);
  MOTORPID_Init(&MOTOR_PID);
  
  /************************ 图像采集 **************************/
  camera_get_img();  //先采集一次图像
}