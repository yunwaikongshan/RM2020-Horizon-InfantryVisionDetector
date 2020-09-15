/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  serial.h
  *Author:  解佳朋
  *Version: 1.3.2.200724_RC
  *Date:  2020.07.24
  *Description: 收发数相关实现
  *Function List:  //主要函数列表，每条记录应包含函数名及功能简要说明
     1.OpenPort 打开串口
     2.configurePort    配置串口
     3.SendData向stm32发送数据
     4.GetData从stm32得到数据
  * Others:主要与RemoteController相互间信息传递
**********************************************************************************/
#ifndef SERIAL_H
#define SERIAL_H

#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>
#include <iostream>
#include<include/Variables.h>
//#include<AngleSolver.h>



/*
*   @brief:创建能够发送小数的共用体
*/
typedef union{
    float f;
    unsigned char c[4];
}float2uchar;

typedef union{
    int16_t d;
    unsigned char c[2];
}int16uchar;

/*
*   @brief:发送云台数据的结构体
*   @param:pitch_bit_   pitch轴数据
*          yaw_bit_     yaw轴数据
*/
typedef struct{
    float2uchar pitch_data_;
    float2uchar yaw_data_;
    unsigned char IsHaveArmor;
    unsigned char distance;
    unsigned char shoot;

}VisionData;

/*
*   @brief:发送云台数据的结构体
*   @param:pitch_bit_   pitch轴数据
*          yaw_bit_     yaw轴数据
*/
typedef struct{
    float2uchar pitch_data_;
    float2uchar yaw_data_;
    float ShootSpeed;
    unsigned char IsHave = false;
    bool IsShootBuff = false;
}Stm32Data;


/*
*   @brief:相关函数
*/
int OpenPort(const char *Portname);
int configurePort(int fd);
void SendData(int fd,VisionData data);
Stm32Data GetData(int fd);
#endif // SERIAL_H

