/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  serial.cpp
  *Author:  解佳朋
  *Version: 1.3.2.200724_RC
  *Date:  2020.07.24
  *Description: 收发数相关实现
  * Others:主要与RemoteController相互间信息传递
**********************************************************************************/
#include "include/serial.h"
#include "include/CRC_Check.h"
#include<include/RemoteController.h>
#define DATA_LENGTH 13               //接受的数据位数
using namespace std;
//unsigned char rec_bytes[255];

/**
*   @brief:打开串口
*   @param  Portname    类型 const char     串口的位置
*/
int OpenPort(const char *Portname)
{
    int fd;
//    fd = open(Portname,O_RDWR);
    fd = open(Portname,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(-1 == fd)
    {
        printf("The port open error!");
        return -1;
    }
    else
    {
        fcntl(fd,F_SETFL,0);   //读取串口的信息
    }
    return fd;
}

/**
*   @brief:配置串口,无奇偶校验
*/
int configurePort(int fd)
{
    struct termios port_set;

    //波特率
    cfsetispeed(&port_set,B115200);
    cfsetospeed(&port_set,B115200);
    //No parity
    port_set.c_cflag &= ~PARENB;         //无奇偶校验
    port_set.c_cflag &= ~CSTOPB;         //停止位:1bit
    port_set.c_cflag &= ~CSIZE;          //清除数据位掩码
    port_set.c_cflag |=  CS8;

    tcsetattr(fd,TCSANOW,&port_set);
    return (fd);

}


/**
*   @brief:发送数据
*   @param:send_bytes[1]--send_bytes[4]为pitch数据
*          send_bytes[5] pitch标志位
*          send_bytes[6]--send_bytes[9]为yaw数据
*          send_bytes[10]  yaw标志位
*          send_bytes[11]--send_bytes[12]为CRC校验码
>>>>>>> 47dad157eee92bce1fca6030481f3b2978a34bec
*/
void SendData(int fd,VisionData data)
{
    unsigned char pitch_bit_,yaw_bit_;
    if(data.pitch_data_.f<0)
     {   pitch_bit_ = 0x00;
         data.pitch_data_.f=-data.pitch_data_.f;
    }
    else pitch_bit_ = 0x01;

    if(data.yaw_data_.f<0)
     {   yaw_bit_ = 0x00;
         data.yaw_data_.f=-data.yaw_data_.f;
    }
    else yaw_bit_ = 0x01;

//    data.yaw_data_.f = 5;
//    data.pitch_data_.f = 10;
    unsigned char send_bytes[13];
    send_bytes[0] = 0xFF;

    send_bytes[1] = data.pitch_data_.c[0];
    send_bytes[2] = data.pitch_data_.c[1];
    send_bytes[3] = data.pitch_data_.c[2];
    send_bytes[4] = data.pitch_data_.c[3];
    send_bytes[5] = pitch_bit_;
//    send_bytes[5] = 0x01;


    send_bytes[6] = data.yaw_data_.c[0];
    send_bytes[7] = data.yaw_data_.c[1];
    send_bytes[8] = data.yaw_data_.c[2];
    send_bytes[9] = data.yaw_data_.c[3];
    send_bytes[10] = yaw_bit_;
//        send_bytes[10] = 0x00;


    send_bytes[11]=data.IsHaveArmor;
    send_bytes[12]=data.distance;
    send_bytes[13]=data.shoot;

    Append_CRC16_Check_Sum(send_bytes, 16);

    write(fd, send_bytes, 16);
    for(int i=0;i<16;i++)
    {
        printf("%X ",send_bytes[i]);
//        cout<<"发"<<endl;
    }
    cout<<"\n";
}

/**
*   @brief:串口PC端接收
* @param:头帧和尾帧均为0xFF,第6位为pith标志位，第11位为yaw轴标志位，共12位，标志位0x01为正，0x00为负
*/
//理想情况每次接受一位，将所接收的数存入get_car，根据首尾帧0xFF来进行定位，
//非理想下没接收到的数据会继续存入流中，需要继续向下取数，存入get_char
//未考虑一次接收大于单次和的情况

//**********************************************//
Stm32Data GetData(int fd)
{
    int bytes;
    unsigned char rec_bytes[1024] = {0};
    ioctl(fd, FIONREAD, &bytes);
    Stm32Data get_data;
    if(bytes<DATA_LENGTH) {
//        get_data.pitch_data_.f = 0;
//        get_data.yaw_data_.f = 0;
//        get_data.ShootSpeed = 16;
        return get_data;
    }
    bytes = read(fd,rec_bytes,bytes);

    int FirstIndex = -1;
    int LastIndex = -1;
    for(int i=0;i<bytes;i++){
        cout<<hex<<(int)rec_bytes[i]<<" ";
        if(rec_bytes[i] == 0xaa&&FirstIndex == -1){
            FirstIndex = i;
        }else if(rec_bytes[i] == 0xbb&&FirstIndex != -1&&i - FirstIndex == 13){
            LastIndex = i;
            break;
        }
    }
    cout<<endl;

    if(FirstIndex != -1&&LastIndex != -1){
        get_data.IsHave = true;
        get_data.pitch_data_.c[0] = rec_bytes[FirstIndex+6];
        get_data.pitch_data_.c[1] = rec_bytes[FirstIndex+7];
        get_data.pitch_data_.c[2] = rec_bytes[FirstIndex+8];
        get_data.pitch_data_.c[3] = rec_bytes[FirstIndex+9];
        get_data.yaw_data_.c[0] = rec_bytes[FirstIndex+1];
        get_data.yaw_data_.c[1] = rec_bytes[FirstIndex+2];
        get_data.yaw_data_.c[2] = rec_bytes[FirstIndex+3];
        get_data.yaw_data_.c[3] = rec_bytes[FirstIndex+4];

        if(rec_bytes[FirstIndex+5] == 0x00){
            get_data.yaw_data_.f = -fabs(get_data.yaw_data_.f);
        }else{
            get_data.yaw_data_.f = fabs(get_data.yaw_data_.f);
        }
        if(rec_bytes[FirstIndex+10] == 0x00){
            get_data.pitch_data_.f = -fabs(get_data.pitch_data_.f);
        }else{
            get_data.pitch_data_.f = fabs(get_data.pitch_data_.f);
        }

        //设置射速
        if(rec_bytes[FirstIndex+11] == 0x00){
            get_data.ShootSpeed = 13;
        }else if(rec_bytes[FirstIndex+11] == 0x01){
            get_data.ShootSpeed = 22;
        }else if(rec_bytes[FirstIndex+11] == 0x02){
            get_data.ShootSpeed = 20;
        }else if(rec_bytes[FirstIndex+11] == 0x03){
            get_data.ShootSpeed = 28;
        }
        if(rec_bytes[FirstIndex+12] == 0x00){
            get_data.IsShootBuff = false;
        }else{
            get_data.IsShootBuff = true;
        }

    }
    ioctl(fd, FIONREAD, &bytes);

    if(bytes>0)
    {
        read(fd,rec_bytes,bytes);
    }
    return get_data;
}


