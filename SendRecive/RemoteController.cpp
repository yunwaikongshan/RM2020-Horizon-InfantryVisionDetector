/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  RemoteController.cpp
  *Author:  解佳朋
  *Version: 1.3.2.200724_RC
  *Date:  2020.07.24
  *Description: 收发数控制
  *Function List:
     1.RemoteController 构造函数,用于记录程序运行时间的cpu周期数初始化
     2.ClearData    数据清空
     3.paraReceiver 发数控制
     4.GetData  从stm32得到数据
     5.paraGetCar   收数控制
     6.paraGetCar   返回当前程序运行时间
     7.ArmorToData  发数接口
  * Others:
        全局收发数变量
        RemoteController Send;          控制发数
        RemoteController Recive;       控制收数
**********************************************************************************/
#include <fstream>
#include <string.h>
#include "include/RemoteController.h"
#define BUFFER_SIZE_ 1
#define MIN_ANGLE 1.5                 //击打最小角度
#define MIN_ANGLE_DIFFERENCE 1.5                 //击打最小角度差
//角度保留
float PreviousPitch = 0;
float PreviousYaw = 0;
volatile unsigned int SerialBuffer ;
VisionData data;
int fd = 0;

RemoteController::RemoteController(){
    BeginTime =(double)cvGetTickCount();
}

void RemoteController::ClearData(){
//    data = new VisionData();
    data.distance = 0;
    data.pitch_data_ .f= 0;
    data.yaw_data_.f = 0;
    data.IsHaveArmor = 0x00;
    data.shoot = 0x00;
    PreviousPitch = 0;
    PreviousYaw = 0;
}

int RemoteController::paraReceiver()
{

    fd=OpenPort("/dev/ttyUSB0");
    if(fd==-1)
    {
        return -1;
    }
    configurePort(fd);

    while(1)
    {
        while(SerialBuffer>=BUFFER_SIZE_);
//        if(data.IsHaveArmor)
//        {
//            SendData(fd, data);
//        }+-
        SerialBuffer=BUFFER_SIZE_;
                    SendData(fd, data);

    }
    close(fd);
}

void RemoteController::paraGetCar(){
    //程序开始时间
//    BeginTime=static_cast<double>(getTickCount());
//    while(1){
//        if(fd!=0&&fd!=-1){
//            break;
//        }
//    }

    while(1){
        get_data = GetData(fd);
        CarData GetCar;
        GetCar.pitch = get_data.pitch_data_.f;
        GetCar.yaw = get_data.yaw_data_.f;
        GetCar.ShootSpeed = get_data.ShootSpeed;
        double timeing=((double)getTickCount()-BeginTime)/getTickFrequency();
        GetCar.BeginToNowTime = timeing*1000.0 ;

        //挂起资源锁
        reciveRes.lock();
        getStm32.pitch = GetCar.pitch;      //传入数据
        getStm32.BeginToNowTime = GetCar.BeginToNowTime;      //传入数据
        getStm32.yaw = GetCar.yaw;
        getStm32.ShootSpeed = GetCar.ShootSpeed;
        //解锁
        reciveRes.unlock();

//        cout<<"getCar:  "<<GetCar.pitch<<GetCar.yaw<<endl;
    }
}

double RemoteController::getClock(){
    double timeing=((double)getTickCount()-BeginTime)/getTickFrequency();
    return timeing*1000.0;
}

/**
 * @brief RemoteController::ArmorToData         发数接口函数
 * @param armor
 * @param _camera
 */
void RemoteController::ArmorToData(ArmorDate armor,Camera _camera){
    if(armor.status == stop)
        ClearData();
    else{
        PreviousPitch = armor.Armor.pitch;
        PreviousYaw =armor.Armor.yaw;
        data.distance =(int)( sqrt(pow(armor.Armor.tz,2)+pow(armor.Armor.ty,2)+pow(armor.Armor.tx,2))/100)+1;
        data.pitch_data_ .f= armor.Armor.pitch;
        data.yaw_data_ .f= armor.Armor.yaw;
        data.IsHaveArmor =0x01;
        if(_camera == UsingGuang){
            data.shoot = 0x00;
        }else{
            if(armor.Armor.pitch<MIN_ANGLE&&armor.Armor.yaw<MIN_ANGLE
                    &&fabs(armor.Armor.pitch - PreviousPitch)<MIN_ANGLE_DIFFERENCE
                    &&fabs(armor.Armor.yaw - PreviousYaw)<MIN_ANGLE_DIFFERENCE&&armor.Armor.IsShooting){
                data.shoot = 0x01;
            }else{
                data.shoot = 0x00;
            }
        }
        PreviousPitch = armor.Armor.pitch;
        PreviousYaw =armor.Armor.yaw;
    }
    if(armor.status == Shoot){
//        data.distance =(int)( sqrt(pow(armor.Armor.tz,2)+pow(armor.Armor.ty,2)+pow(armor.Armor.tx,2))/100)+1;
//        data.pitch_data_ .f= armor.Armor.pitch;
//        data.yaw_data_ .f= armor.Armor.yaw;
//        data.IsHaveArmor =0x01;
//        if(_camera == UsingGuang){
//            data.shoot = 0x00;
//        }else{
//            if(armor.Armor.pitch<MIN_ANGLE&&armor.Armor.yaw<MIN_ANGLE){
//                data.shoot = 0x01;
//            }else{
//                data.shoot = 0x00;
//            }
//        }
    }
    SerialBuffer --;
}


//全局收发数变量
RemoteController Send;
RemoteController Recive;
