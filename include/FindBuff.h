/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  FindBuff.h
  *Author:  解佳朋
  *Version: 1.3.1.200517_RC
  *Date:  2020.05.17
  *Description: 大符识别
  *Function List:
     1.BuffModeSwitch   大符识别接口
     2.PreDelBuff   大符识别预处理
     3.FindBestBuff    寻找并存入所有的大符目标
     4.GetShootBuff    寻找待击打的大符目标
**********************************************************************************/
#ifndef FINDBUFF_H
#define FINDBUFF_H
#include<include/Variables.h>
#include<include/VideoCapture.h>
#include<include/BuffAngleSolver.h>
class FindBuff{
private:
    void PreDelBuff(Mat Src, Mat &dst);
    vector<RotatedRect> FindBestBuff(Mat Src,Mat & dst);
    RotatedRect GetShootBuff(vector<RotatedRect> box_buffs,Mat Src);
public:
    RM_BuffData* BuffModeSwitch(Mat Src);
};

#endif // FINDBUFF_H
