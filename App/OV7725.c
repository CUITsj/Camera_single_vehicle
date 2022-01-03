#include "OV7725.h"

uint8 Weight[60] = {	0,  0,  0,  0,  0,    0,  0,  0,  0,  0,
			0,  0,  0,  0,  0,    0,  0,  0,  0,  0,
                        5,  4,  3,  3,  3,     5,  4,  3,  3,  3,  
                        5,  4,  3,  3,  3,     5,  4,  3,  3,  3,
                       5,  4,  3,  3,  3,    1,  1,  1,  1,  1,
			1,  1,  1,  1,  1,    1,  1,  1,  1,  1   };	//加权平均参数
uint8 start_flag = 0;

uint8 Image_analyze(uint8 *img)//0表示黑255表示白
{
  uint8 circle_under = 0, circle_under1 = 0, circle_under2 = 0;
  uint8 left_cross_flag, left_jump_flag, right_cross_flag, right_jump_flag;
  uint8 get_right_flag[61], get_left_flag[61], get_mid_flag[61];//丢线标志
  uint8 left_white_flag = 0, right_white_flag = 0;//全白或全黑丢线
  uint8 scan_point, scan_upbound = 10, scan_uppoint = 0, find_black = 0;
  int8 i, j, k, k1, circle_tier;//圆环行
  int8 left_bound[61], right_bound[61], mid_point[61];
  uint16 weight_sum = 1, midpoint_sum = 0, mid_pointcount = 1;
  
  uint8 road_midpoint, start_flag1 = 0, start_flag2 = 0, start_flag3 = 0, uncircle_flag1 = 0, uncircle_flag2 = 0;
  
  for(i=59,scan_upbound = 10; i>=scan_upbound; i--)//从59到scan_upbound进行扫描//59改57
  {
    left_cross_flag = 0;
    left_jump_flag = 0;
    right_cross_flag = 0;
    right_jump_flag = 0;
    get_right_flag[i] = 0;
    get_left_flag[i] = 0;
    get_mid_flag[i] = 0;
    left_white_flag = 0;
    right_white_flag = 0; 
    uncircle_flag1 = 0;
    uncircle_flag2 = 0;//初始化标志变量     
   
/********************************** 开始找左边界 **************************************/
    if (i == 59) //设置第一行找左边界的起始点
    {
      j = 30;
      scan_point=30;
    }
    else if (i < 59)
    {
      for (k=i+1; k<=59; k++)
      {
        if (get_left_flag[k] == 1)  //以前面的左边界为参考
        {
          j = left_bound[k] + 5;
          scan_point = left_bound[k] + 5;   //腾出一定空隙以免丢线,该值一定要大于等于5
          break;
        }
      }
      if (k == 60)  //前面没有找到的左边界
      {
        j = 30;
        scan_point=30;
      }    
    }
    for(j=j,scan_point=scan_point; j>=2; j--)//开始找左边界
    {
      if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//找到了边界
      {
        if (i < 59)
        {
          for (k=i+1; k<=59; k++)
          {
            if (get_right_flag[k] == 1)
            {
              if (j - 1 >= right_bound[k])//交叉丢线
              {
                left_cross_flag = 1;        
                break;
              }
              else
              {
                left_cross_flag = 0;     
                break;
              }
            }
          }
          for (k=i+1; k<=59; k++)
          {
            if (k-i-1<10 && get_left_flag[k] == 1)
            {
              if (j - 1 > left_bound[k] + 8 || j - 1 < left_bound[k])//跳跃丢线，j - 1 < left_bound[k] - 4防止左S弯道丢线、、优化环形去掉-4
              {
                left_jump_flag = 1;
                break;
              }
              else
              {
                left_jump_flag = 0;
                break;
              }
            }
          }     
          if (left_cross_flag == 1 || left_jump_flag == 1)//丢线
          {
            get_left_flag[i] = 0;
            left_bound[i] = 0;
            break;
          }
          else if (left_cross_flag == 0 && left_jump_flag == 0)//没有丢线
          {
            left_bound[i] = j - 1;
            get_left_flag[i] = 1;
            if (left_bound[i] >= 70)           //更新上界
            {
              scan_upbound = i;
            }
            break;
          }
        }
        else
        {             
            left_bound[i] = j - 1;//找到了左边界
            get_left_flag[i] = 1;          
            break;
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point - 1] == 0)//全黑
      {
        if (scan_point == 79)  //该行没有左边界
        {
          get_left_flag[i] = 0;
          left_bound[i] = 0;    //全黑丢线，默认0
          break;
        }
        scan_point++;
        j = scan_point + 1;
      }
      else if(img[i*80 + scan_point] == 255 && img[i*80 + scan_point - 1] == 255)//全白
      {
        if (scan_point == 2)//该行没有左边界
        {
          get_left_flag[i] = 0;
          left_white_flag = 1;  //全白丢线
          left_bound[i] = 0;   //全白丢线，默认0
          break;
        }
        scan_point--;
      }
    }
/********************************** 找左边界结束 **************************************/
    
/********************************** 开始找右边界 **************************************/
    if (i == 59) //设置第一行找右边界的起始点
    {
      j = 50;
      scan_point=50;
    }
    else if (i < 59)
    {
      for (k=i+1; k<=59; k++) //以前面找到的右边界为参考
      {
        if (get_right_flag[k] == 1)
        {
          j = right_bound[k]-5;           //腾出一定空隙来扫描边界,该值一定要大于等于5
          scan_point = right_bound[k]-5;
          break;
        }
      }
      if (k == 60) //前面没有找到的右边界
      {
        j = 50;
        scan_point=50;
      }    
    }
    for(j=j,scan_point=scan_point; j<=78; j++)//开始找右边界
    {
      if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
      {        
        if (i < 59)
        {
          for (k=i+1; k<=59; k++)
          {
            if (get_left_flag[k] == 1)
            {
              if (j + 1 <= left_bound[k])//交叉丢线
              {
                right_cross_flag = 1;                              
                break;
              }
              else
              {
                right_cross_flag = 0;                              
                break;
              }
            }
          }
          for (k=i+1; k<=59; k++)
          {
            if (k-i-1<10 && get_right_flag[k] == 1)
            {
              if (j + 1 > right_bound[k] || j + 1 < right_bound[k] - 8)//跳跃丢线，j + 1 > right_bound[k] + 4 防止右S弯道丢线、、优化环形去掉+4
              {
                right_jump_flag = 1;                
                break;
              }
              else
              {
                right_jump_flag = 0;
                break;
              }
            }
          }
          if (right_cross_flag == 1 || right_jump_flag == 1)//丢线
          {
            get_right_flag[i] = 0;
            right_bound[i] = 79;
            break;
          }
          else if (right_cross_flag == 0 && right_jump_flag == 0)//没有丢线
          {
            get_right_flag[i] = 1;
            right_bound[i] = j + 1;
            if (right_bound[i] <= 10)//更新上界
            {
              scan_upbound = i;
            }
            break;
          }
          
        }
        else
        {          
            right_bound[i] = j + 1;
            get_right_flag[i] = 1;    //找到了右边界
            break;  
        }
      }
      else if (img[i*80 + scan_point] == 0 && img[i*80 + scan_point + 1] == 0)//全黑
      {
        if (scan_point == 0) // 该行无右边界
        {
          right_bound[i] = 79;//全黑丢线，默认79
          get_right_flag[i] = 0;
          break;
        }
        scan_point--;
        j = scan_point - 1;
      }
      else if (img[i*80 + scan_point] == 255 && img[i*80 + scan_point + 1] == 255)//全白
      {
        if (scan_point == 78)
        {
          right_bound[i] = 79;//全白丢线，默认79
          get_right_flag[i] = 0;
          right_white_flag = 1;//全白丢线
          break;
        }
        scan_point++;
      }
    }            
/********************************** 找右边界结束 **************************************/   
          
/********************************** 处理该行找线结果 **************************************/
    
    if (get_left_flag[i] == 0 && get_right_flag[i] == 1)//左边丢线
    {
      mid_point[i] = right_bound[i]-38*i/100-14;
      get_mid_flag[i] = 1;
        
      if (right_bound[i] < 65)
      {
        for (scan_uppoint = i; scan_uppoint>=2; scan_uppoint--)   //由于左边丢线 更新扫描上界
        {
          if (img[scan_uppoint*80+1] == 255 && img[(scan_uppoint-1)*80+1] == 0)
          {
            if (i - scan_uppoint > 30) //左边丢线宽度大于17则认为找到新的扫描上界
            {
              scan_upbound = scan_uppoint;
            }
            break;
          }
        }
      }
    }
    if (get_left_flag[i] == 1 && get_right_flag[i] == 0)//右边丢线
    {
      mid_point[i] = left_bound[i]+38*i/100+14;
      get_mid_flag[i] = 1;
        
      if (left_bound[i] > 15)
      {
        for (scan_uppoint = i; scan_uppoint>=2; scan_uppoint--)
        {
          if (img[scan_uppoint*80 + 79] == 255 && img[(scan_uppoint-1)*80 + 79] == 0)
          {
            if (i - scan_uppoint > 30)  //左边丢线宽度大于17则认为找到新的扫描上界
            {
              scan_upbound = scan_uppoint;
            }
            break;
          }
        }
      }
    }
    if (get_left_flag[i] == 1 && get_right_flag[i] == 1)              //没有丢线
    {
      mid_point[i] = (left_bound[i] + right_bound[i])/2;
      get_mid_flag[i] = 1;
    }
    if (get_left_flag[59] == 1 && get_right_flag[59] == 1)////////////////////////////////////////////
    {
      if (right_bound[59] - left_bound[59] < 50)
      {
        return (right_bound[59] + left_bound[59])/2;
      }
    }
       if (left_white_flag == 1 && right_white_flag ==1 || left_white_flag == 1 && right_jump_flag == 1 || right_white_flag == 1 && left_jump_flag == 1 || right_jump_flag == 1 && left_jump_flag == 1)
       {
    /***************************************此处加入十字和环形赛道处理************************************************/      
         find_black = 40;
     for (circle_tier = i; circle_tier>10; circle_tier--)
     {
           if (img[circle_tier*80 + find_black] == 255 && img[(circle_tier-1)*80 + find_black] == 0)//有可能是环形
           {
             circle_under = circle_tier - 1;
             //防止与十字冲突
             for (k=circle_under+10; k<59 && k>10; k--)
             {
               if (img[k*80 + find_black - 8] == 0)
               {
                 circle_under1 = k;
                 break;
               }
             }
             for (k=circle_under+10; k<59 && k>10; k--)
             {
               if (img[k*80 + find_black + 8] == 0)
               {
                 circle_under2 = k;
                 break;
               }
             }
             if(circle_under1 == 0 || circle_under2 == 0)
             {
               break;
             }
             
             if (circle_under1 <= circle_under)
             {
               if (circle_under - circle_under1 > 5)//非环形
               {
                 break;
               }
             }
             else
             {
               if (circle_under1 - circle_under > 5)//非环形
               {
                 break;
               }
             }
             if (circle_under2 <= circle_under)
             {
               if (circle_under - circle_under2 > 5)//非环形
               {
                 break;
               }
             }
             else
             {
               if (circle_under2 - circle_under > 5)//非环形
               {
                 break;
               }
             }   
             //防止与弯道冲突
             for (k=find_black; k<find_black + 20 && k<79; k++)
             {
               if (img[(circle_tier+3)*80 + k] == 0)//非环形
               {
                 goto skip1;
               }
             }
             for (k=find_black; k>find_black - 20 && k>0; k--)
             {
               if (img[(circle_tier+3)*80 + k] == 0)//非环形
               {
                 goto skip1;
               }
             }
               //防止与起跑线冲突
             for (k=find_black ; k>find_black - 6; k--)
             {
               if (img[(circle_under-5)*80 + k] == 255)
               {
                 uncircle_flag1 = 1;
                 break;
               }          
             }
             for (k=find_black ; k<find_black + 6; k++)
             {
               if (img[(circle_under-5)*80 + k] == 255)
               {
                 uncircle_flag2 = 1;
                 break;
               }          
             }
             if (uncircle_flag1 == 1 && uncircle_flag2 == 1) //起跑线
             {           
                 break;//return 40;
             }
               for (i=circle_under; i>10; i--)
               {        
                 for(j=79; j>=find_black; j--)//开始找左边界
                {
                  if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)//找到了边界
                  {                  
                    left_bound[i] = j - 1;                   
                    mid_point[i] = left_bound[i]+38*i/100-4;
                    get_mid_flag[i] = 1;                  
                     break;
                  }               
                }   
               }
      for (i=circle_under; i>10; i--)
      {
         if (get_mid_flag[i] == 1 && mid_point[i] > 0 && mid_point[i] < 79)
          {
            midpoint_sum += mid_point[i];
            mid_pointcount++;
          }
      }
      if (midpoint_sum/mid_pointcount == 0)
      {
        return 60;
      }
      else
      {
        mid_pointcount = mid_pointcount - 1;
        return midpoint_sum/mid_pointcount;
      }
           }
           if (0)
           {
              skip1:break;
           }
      } 
        }
    /*****************************************十字和环形赛道    处理完毕***********************************************/      
     /********************************** 该行找线结果处理完毕 **************************************/
  }//进入下一行 
    weight_sum = 1;
 for (i=scan_upbound; i<=59; i++)
 {
     if (get_mid_flag[i] == 1 && mid_point[i] > 0 && mid_point[i] < 79)
      {
        midpoint_sum += mid_point[i]*Weight[i];
        weight_sum += Weight[i];
      }
 }
 if (midpoint_sum/weight_sum == 0)
 {
   road_midpoint = 40;
 }
 else
 {
   weight_sum = weight_sum - 1;
   road_midpoint = midpoint_sum/weight_sum;
 }
 if (road_midpoint > 37 && road_midpoint < 43)
 {
       for (i=59; i>21; i--)
       {
         for (j=road_midpoint; j>road_midpoint-8 && j>0; j--)  //找左边界 8可改
         {
           if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0) //找到了左边界
           {                        
             for (k=j-1; k>j-6 && k>0; k--)
             {
               if (img[(i-1)*80 + k] == 0 && img[(i-1)*80 + k - 1] == 255)//起跑线
               {
                 start_flag1 =  1; 
                 break;
               }              
             }
             for (k=j; k<j+6 && k<79; k++)
             {
               if (img[(i-1)*80 + k] == 255 && img[(i-1)*80 + k + 1] == 0)//起跑线
               {
                 start_flag2 =  1;     
                 break;
               }
             }
             if (start_flag2 == 1)
             {
               for (k1=k; k1<k+6 && k<79; k++)
               {         
                   if (img[(i-1)*80 + k] == 0 && img[(i-1)*80 + k + 1] == 255)//起跑线
                   {
                     start_flag3 =  1;     
                     break;
                   }
               }
             }           
             if (start_flag1 == 1 && start_flag2 == 1 && start_flag3 == 1)
             {
               start_flag = start_flag + 1;
               return road_midpoint;
             }
             else
             {
               start_flag1 = 0;
               start_flag2 = 0;
               start_flag3 = 0;
             }
             break;
           }
         }    
       }
       for (i=30; i<57; i++)//以右边界为基准找障碍 右边界 函数为：y=79+0.3x-29 左边界函数关系 y=38-x/2 半边赛道宽度 y=0.4x+6.5
       {
         if (get_right_flag[i] == 1 && right_bound[i] > 40+0.3*i)
         {
            for (j = right_bound[i] - i*4/10 - 6 ; j > right_bound[i] - i*4/10 - 6 - 15; j--)
            {
              if (img[i*80 + j] == 255 && img[i*80 + j - 1] == 0)
              {
                return (j - 1 + right_bound[i])/2 + 5;
              }
            }
         }
         else
         {
           break;
         }
       }
       for (i=30; i<57; i++)//以左边界为基准找障碍 右边界 函数为：y=79+0.3x-29 左边界函数关系 y=38-x/2 半边赛道宽度 y=0.4x+6.5
       {
         if (get_left_flag[i] == 1 && left_bound[i] < 48-i/2)
         {
            for (j = left_bound[i] + i*4/10 + 6 ; j < left_bound[i] + i*4/10 + 6 + 15; j++)
            {
              if (img[i*80 + j] == 255 && img[i*80 + j + 1] == 0)
              {
                return (j + 1 + left_bound[i])/2 - 5;
              }
            }
         }
         else
         {
           break;
         }
       }
       return road_midpoint;
 }
 else
 {
   return road_midpoint;
 }
}

