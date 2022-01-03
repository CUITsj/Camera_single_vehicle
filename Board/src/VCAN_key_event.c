/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_key_event.h
 * @brief      KEY �¼�����ʵ��
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2014-01-04
 */

#include "VCAN_key_event.h"
#include "VCAN_UI_VAR.h"
//#include "NRF24L0.h"
//#include "NRF24L0_MSG.h"


void key_event_init()
{
    //��ʼ�� ȫ�� ����
    key_init(KEY_MAX);

    //��ʼ����ʱ����
    var_init();

    //ͬ��ȫ�����ݲ���ʾ
    //var_syn(VAR_MAX);       //ͬ��ȫ�� ,������ͬ������ʾȫ������Ϊ�п���ͬ��ʧ�ܡ�
    //var_display(VAR_MAX);   //��ʾȫ��
}

void deal_key_event()
{
    KEY_MSG_t keymsg;

    while(get_key_msg(& keymsg))     //��ð����ͽ��д���
    {
        if(keymsg.status == KEY_DOWN)
        {
            switch(keymsg.key)
            {
            case KEY_U:
                
                break;

            case KEY_D:
                
                break;

            case KEY_L:
                
                break;

            case KEY_R:
                
                break;

            case KEY_B:
                
                break;

            case KEY_A:
                
                break;
            default:
                break;
            }
        }
        else if(keymsg.status == KEY_HOLD)
        {
            switch(keymsg.key)
            {
            case KEY_U:
                
                break;

            case KEY_D:
                
                break;

            case KEY_L:
                
                break;

            case KEY_R:
                
                break;

            case KEY_B:                //���� OK �� ͬ��ȫ�����ݲ���ʾ
                
                break;
     
            default:            //����Ҫ���� cancel
                break;
            }
        }
        else
        {
            //KEY_UP ,�����д���
        }
    }
}





