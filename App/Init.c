#include "Init.h"

uint8 nrf_rx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];
uint8 nrf_tx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];

void All_init()
{
  /************************ �ж����ȼ�������  ***********************/
  NVIC_SetPriorityGrouping(4);//������Χ��0~4��4��ʾ4bitȫΪ��ռ���ȼ���û��ѹ���ȼ�
  NVIC_SetPriority(PORTA_IRQn, 0);  //����ԽС���ȼ�Խ�ߣ�����ͷ
  NVIC_SetPriority(PORTE_IRQn, 1);  //NRF
  NVIC_SetPriority(DMA0_IRQn, 2);  //����ͷ
  NVIC_SetPriority(PIT0_IRQn, 3);  //��ʱ��
  NVIC_SetPriority(PIT1_IRQn, 4);  //��ʱ��
  
  
  /************************** LED ��ʼ��  ***************************/
  //led_init(LED_MAX);
  
  /************************ LCD Һ���� ��ʼ��  ***********************/
  
  
  /************************ �����ʼ��  ***********************/
  ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ, S3010_MID);//ռ�ձ�Ϊ S3010_MID/FTM3_PRECISON,FTM3_PRECISONΪ10000u,��MK60_FTM.h�ж���
  
  /************************ �����ʼ����ֹ  ***********************/
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, MOTOR1_DUTY); //ռ�ձ�Ϊ0
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 0);
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, MOTOR2_DUTY);
  
  /************************ ����ͷ ��ʼ��  ***********************/
  camera_init(imgbuff);  //��ͼ��ɼ���imgbuff����
  //�����жϷ�����
  set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);
  set_vector_handler(DMA0_VECTORn, DMA0_IRQHandler);
  
  /************************ ��������ʼ��  ***********************/   
  ftm_quad_init(FTM1);
  ftm_quad_init(FTM2);
  /************************** �������� ��ʼ��  ***********************/
 /* uart_init(UART3, 9600);
  uart_putchar(UART3, 0xff);    //����һ�ν�����������ϵ�����Ĵ����Ӳ�����
  uart_rx_irq_en(UART3);
  set_vector_handler(UART3_RX_TX_VECTORn , uart3_handler);
  enable_irq(UART3_RX_TX_IRQn );*/
  /************************ ����ģ�� NRF ��ʼ��  ***********************/
/*#if NRF_OFF
  uint32 i = 20;
  while(!nrf_init());
  //�����жϷ�����
  set_vector_handler(PORTE_VECTORn, PORTE_IRQHandler);
  enable_irq(PORTE_IRQn);
  nrf_msg_init();
  
  while(i--)
  {
    nrf_msg_tx(COM_RETRAN, nrf_tx_buff);// COM_RETRAN��λ���䣬����֮ǰ���յ�������
  }
#endif*/
  
  /******************** ��ʱ�� ��ʼ��  *********************/ 
  pit_init_ms(PIT0, 10);    //ÿ10ms����һ�γ���
  set_vector_handler(PIT0_VECTORn, PIT0_IRQHandler);
  enable_irq(PIT0_IRQn);
  
  pit_init_ms(PIT1, 50);    //����ͣ��
  set_vector_handler(PIT1_VECTORn, PIT1_IRQHandler);
  enable_irq(PIT1_IRQn);
  
 /* pit_init_ms(PIT2, 50);    //���ڴ���
  set_vector_handler(PIT2_VECTORn, PIT2_IRQHandler);
  enable_irq(PIT2_IRQn);*/
  
  /********************** PID������ʼ�� ***********************/
  S3010PID_Init(&S3010_PID);
  MOTORPID_Init(&MOTOR_PID);
  
  /************************ ͼ��ɼ� **************************/
  camera_get_img();  //�Ȳɼ�һ��ͼ��
}