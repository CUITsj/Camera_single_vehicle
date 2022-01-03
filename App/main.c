#include "common.h"
#include "include.h"

uint8 * imgbuff = (((uint8 *)&nrf_tx_buff) + COM_LEN);
uint8 img[CAMERA_W*CAMERA_H];
uint8 screen_mid = 40;          //��Ļ����
uint8 check_flag = 0;

uint8 mid_point = 40;

int16 SetSpeed, SetSpeed1 = 0, SetSpeed2 = 0;//ֱ�� ���


void main(void)
{
  Site_t site     = {0, 0};               //��ʾͼ�����Ͻ�λ��
  
  LCD_init();
  key_init(KEY_MAX);
  while(key_check(KEY_B) != KEY_DOWN)
    {
        if(key_check(KEY_U) == KEY_DOWN) //�ٶ�ģʽ1�����ȶ�
        { 
          SetSpeed1 = 115;
          SetSpeed2 = 115;    
          site.x = 0;
          site.y = 0;
          LCD_clear(RED);     
          LCD_num(site,115, BLUE,RED); 
          site.x = 0;
          site.y = 15;
          LCD_num(site,115, BLUE,RED);         
        }
        if(key_check(KEY_D) == KEY_DOWN) //�ٶ�ģʽ2�����ȶ�
        {   
          SetSpeed1 = 155;
          SetSpeed2 = 115;      
          site.x = 0;
          site.y = 0;
          LCD_clear(RED);     
          LCD_num(site,155, BLUE,RED); 
          site.x = 0;
          site.y = 15;
          LCD_num(site,115, BLUE,RED);      
                                       
        }
        if(key_check(KEY_L) == KEY_DOWN) //�ٶ�ģʽ3���ȶ��ı���
        {         
          SetSpeed1 = 160;
          SetSpeed2 = 115;      
          site.x = 0;
          site.y = 0;
          LCD_clear(RED);     
          LCD_num(site,160, BLUE,RED); 
          site.x = 0;
          site.y = 15;
          LCD_num(site,115, BLUE,RED);                                          
        }
        if(key_check(KEY_R) == KEY_DOWN) //�ٶ�ģʽ4,���ȶ��ı���
        {                 
          SetSpeed1 = 165;
          SetSpeed2 = 110;
          site.x = 0;
          site.y = 0;
          LCD_clear(RED);     
          LCD_num(site,165, BLUE,RED); 
          site.x = 0;
          site.y = 15;
          LCD_num(site,110, BLUE,RED);                 
        }
        if(key_check(KEY_A) == KEY_DOWN) //�ٶ�ģʽ5,���ģʽ���ã�����
        {                 
          SetSpeed1 = 170;
          SetSpeed2 = 110;
          site.x = 0;
          site.y = 0;
          LCD_clear(RED);     
          LCD_num(site,170, BLUE,RED); 
          site.x = 0;
          site.y = 15;
          LCD_num(site,110, BLUE,RED);                 
        }
    }
  //SCCB_WriteByte (OV7725_CNST, 64);	//�ı�ͼ����ֵ
  check_flag = 1;
  DELAY_MS(3000);
  check_flag = 0;
  All_init();
  
  
  while(1)
  {
    /************************ ͼ��ɼ�  ***********************/
    camera_get_img();
    img_extract(img, imgbuff, CAMERA_SIZE);

    /************************ ͼ����� ***********************/  
    mid_point = Image_analyze(img);
    
    /************************ �������  ***********************/ 
    S3010_DUTY = S3010PID_Control(&S3010_PID, mid_point, screen_mid);
    S3010_DUTY = S3010_protect(S3010_DUTY, S3010_MIN, S3010_MAX);
    ftm_pwm_duty(S3010_FTM, S3010_CH,  S3010_DUTY);
    
    /************************ �������  ***********************/  
   if (S3010_DUTY - S3010_MID >= 0)
    {
      if (S3010_DUTY - S3010_MID < 10) //ֱ��
      {
        SetSpeed = SetSpeed1;
        MOTOR1_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed,  MOTOR1_speed);
        MOTOR2_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed, -MOTOR2_speed);
        if (MOTOR1_DUTY < 0)
        {
          MOTOR1_DUTY = -MOTOR1_DUTY;
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, MOTOR1_DUTY);  //��  
          
        }
        else
        {
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_MAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, MOTOR1_DUTY);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);  //��  
        }
        if (MOTOR2_DUTY < 0)
        {
          MOTOR2_DUTY = -MOTOR2_DUTY;
          MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, MOTOR2_DUTY);  //��  
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0); 
        }
        else
        {
           MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_MAX);        
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);  //��  
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, MOTOR2_DUTY); 
        }
      }
      
      else if (S3010_DUTY - S3010_MID >=10)                           //��ת
      {
        SetSpeed = SetSpeed2;
        MOTOR1_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed,  MOTOR1_speed);
        if (MOTOR1_DUTY < 0)
        {
          MOTOR1_DUTY = -MOTOR1_DUTY;
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, MOTOR1_DUTY);  //��  
          
        }
        else
        {
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_MAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, MOTOR1_DUTY);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);  //��  
        }
        SetSpeed = SetSpeed-((S3010_DUTY - S3010_MID)*65/150);
        MOTOR2_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed, -MOTOR2_speed);
        if (MOTOR2_DUTY < 0)
        {
          MOTOR2_DUTY = -MOTOR2_DUTY;
          MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, MOTOR2_DUTY);  //��  
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0); 
        }
        else
        {
           MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_MAX);        
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);  //��  
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, MOTOR2_DUTY); 
        }
        
      }
    }
    else if (S3010_MID - S3010_DUTY >= 0)
    {
      if (S3010_MID - S3010_DUTY < 10)  //ֱ��
      {
        SetSpeed = SetSpeed1;
        MOTOR1_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed,  MOTOR1_speed);
        MOTOR2_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed, -MOTOR2_speed);
        if (MOTOR1_DUTY < 0)
        {
          MOTOR1_DUTY = -MOTOR1_DUTY;
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, MOTOR1_DUTY);  //��  
          
        }
        else
        {
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_MAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, MOTOR1_DUTY);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);  //��  
        }
        if (MOTOR2_DUTY < 0)
        {
          MOTOR2_DUTY = -MOTOR2_DUTY;
          MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, MOTOR2_DUTY);  //��  
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0); 
        }
        else
        {
           MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_MAX);        
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);  //��  
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, MOTOR2_DUTY); 
        }
      }
     
      else if (S3010_MID - S3010_DUTY >=10)                          //��ת
      {
        
        SetSpeed = SetSpeed2;
        MOTOR2_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed, -MOTOR2_speed);
        if (MOTOR2_DUTY < 0)
        {
          MOTOR2_DUTY = -MOTOR2_DUTY;
          MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, MOTOR2_DUTY);  //��  
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0); 
        }
        else
        {
           MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_MAX);        
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);  //��  
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, MOTOR2_DUTY); 
        }
        SetSpeed = SetSpeed-((S3010_MID-S3010_DUTY)*65/150);
        MOTOR1_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed,  MOTOR1_speed);
        if (MOTOR1_DUTY < 0)
        {
          MOTOR1_DUTY = -MOTOR1_DUTY;
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, MOTOR1_DUTY);  //��  
          
        }
        else
        {
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_MAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, MOTOR1_DUTY);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);  //��  
        }   
      }
    }
    /*********************************************** ��ֹ��ת  **********************************************/
    if ((MOTOR1_speed <= 0 && -MOTOR2_speed <= 0) && (MOTOR1_DUTY == MOTOR_MAX && MOTOR2_DUTY == MOTOR_MAX) && check_flag == 1)
    {
      while(1)
      {
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,  0); //ռ�ձ�Ϊ0
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,  0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,  0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,  0);
      }
    }
    /*********************************************** ��ֹ��ת  **********************************************/
    if (check_flag == 1 && start_flag > 0) //ͣ��
    { 
      SetSpeed = 100;
      while(SetSpeed--)
      {
        MOTOR1_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed,  MOTOR1_speed);
        MOTOR2_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed, -MOTOR2_speed);
        if (MOTOR1_DUTY < 0)
        {
          MOTOR1_DUTY = -MOTOR1_DUTY;
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, MOTOR1_DUTY);  //��  
          
        }
        else
        {
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_MAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, MOTOR1_DUTY);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);  //��  
        }
        if (MOTOR2_DUTY < 0)
        {
          MOTOR2_DUTY = -MOTOR2_DUTY;
          MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, MOTOR2_DUTY);  //��  
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0); 
        }
        else
        {
           MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_MAX);        
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);  //��  
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, MOTOR2_DUTY); 
        }
        camera_get_img();
        img_extract(img, imgbuff, CAMERA_SIZE);
        mid_point = Image_analyze(img);
        S3010_DUTY = S3010PID_Control(&S3010_PID, mid_point, screen_mid);
        S3010_DUTY = S3010_protect(S3010_DUTY, S3010_MIN, S3010_MAX);
        ftm_pwm_duty(S3010_FTM, S3010_CH,  S3010_DUTY);
      }
      while(1)
      {
        SetSpeed = 0;
        MOTOR1_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed,  MOTOR1_speed);
        MOTOR2_DUTY = MOTORPID_Control(&MOTOR_PID, SetSpeed, -MOTOR2_speed);
        if (MOTOR1_DUTY < 0)
        {
          MOTOR1_DUTY = -MOTOR1_DUTY;
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, 0);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, MOTOR1_DUTY);  //��  
          
        }
        else
        {
          MOTOR1_DUTY = MOTOR_protect(MOTOR1_DUTY, MOTOR_MIN, MOTOR_MAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM, MOTOR1_DUTY);
          ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM, 0);  //��  
        }
        if (MOTOR2_DUTY < 0)
        {
          MOTOR2_DUTY = -MOTOR2_DUTY;
          MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_STOPMAX);
          ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, MOTOR2_DUTY);  //��  
          ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, 0); 
        }
        else
        {
           MOTOR2_DUTY = MOTOR_protect(MOTOR2_DUTY, MOTOR_MIN, MOTOR_MAX);        
           ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM, 0);  //��  
           ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM, MOTOR2_DUTY); 
        }
        camera_get_img();
        img_extract(img, imgbuff, CAMERA_SIZE);
        mid_point = Image_analyze(img);
        S3010_DUTY = S3010PID_Control(&S3010_PID, mid_point, screen_mid);
        S3010_DUTY = S3010_protect(S3010_DUTY, S3010_MIN, S3010_MAX);
        ftm_pwm_duty(S3010_FTM, S3010_CH,  S3010_DUTY);
      }     
    }
    }
}