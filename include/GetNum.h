/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  GetNum.h
  *Author:  解佳朋
  *Version: 1.2.3.191017_RC
  *Date:  2020.07.24
  *Description: 利用svm分类器进行装甲数字识别
  *Function List:
     1.GetNumber:   数字识别传入接口
  * Others:
        所传入Src为装甲识别后所得ROI
**********************************************************************************/
#ifndef GETNUM_H
#define GETNUM_H
#include<include/Variables.h>
#include<include/svm.h>

int GetNumber(Mat Src,Point2f center,float height,float angle);
#endif // GETNUM_H
