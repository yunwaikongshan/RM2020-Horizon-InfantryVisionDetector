/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  RemoteController.h
  *Author:  解佳朋
  *Version: 1.3.2.200724_RC
  *Date:  2020.07.24
  *Description: 收发数控制
  *Function List:
     1.RemoteController 构造函数,用于记录程序运行时间的cpu周期数初始化
     2.ClearData    数据情空
     3.paraReceiver 发数控制
     4.GetData  从stm32得到数据
     5.paraGetCar   收数控制
     6.paraGetCar   返回当前程序运行时间
     7.ArmorToData  发数接口
**********************************************************************************/
#ifndef REMOTECONTROLLER_H
#define REMOTECONTROLLER_H
#include<include/Variables.h>
//#include<AngleSolver.h>
#include<include/serial.h>

class RemoteController
{
private:

    Stm32Data get_data;                              //收数共用体
    double BeginTime;
public:
    RemoteController();
    void ClearData();
    int paraReceiver();
    void ArmorToData(ArmorDate armor,Camera _camera);
    void GetSerial(char * serial);
    void paraGetCar();
    double getClock();
};

//全局收发数变量
extern RemoteController Send;
extern RemoteController Recive;
#endif // REMOTECONTROLLER_H
