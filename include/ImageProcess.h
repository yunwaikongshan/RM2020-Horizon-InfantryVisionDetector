/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  ImageProcess.h
  *Author:  解佳朋
  *Version: 1.3.2.2007021_RC
  *Date:  2020.07.21
  *Description: 图片获取与处理控制
  *Function List:
     1.ImageProducter 获取图片
     2.ImageConsumer  消费图片
     3.DrawImage  绘制反馈结果
**********************************************************************************/
#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H
#include<include/Variables.h>
#include<include/VideoCapture.h>
#include<include/LongFindArmor.h>
#include<include/DaHengCamera.h>
#include<include/AngleSolver.h>
#include<include/FindBuff.h>

class ImageProcess{
public:
    ImageProcess();
    void ImageProducter();
    void ImageConsumer();
    void DrawImage(Mat Src,ArmorDate BestArmorDate);
    void SaveData();
};

#endif // IMAGEPROCESS_H
