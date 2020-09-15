/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  ShootTuoluo.h
  *Author:  解佳朋
  *Version: 1.3.4.200709_RC
  *Date:  2020.07.09
  *Description: 陀螺检测
  * Function List:
     1.getTuoluoData    陀螺检测接口,传入当前处理图片和识别信息,内部自动连续处理,反馈识别结果
     2.firstSetTuoluo     第一次发现目标,将各项积累值清零
     3.continueSetTuoluo    第一种连续判断目标运动状态函数
     3.SecContinueSetTuoluo     第二种连续检测方案,添加计算最近几帧的目标角度方差作为判断依据
     4. drawImage   显示判断为陀螺状态时目标图像
  * Others:利用单目测距和三角形相似原理得到目标运动规律,估计目标当前是否处于陀螺状态,并给出具体击打所需信息
**********************************************************************************/
#ifndef SHOOTTUOLUO_H
#define SHOOTTUOLUO_H
#include<include/Variables.h>
#include<include/AngleSolver.h>
#include<include/Filter.h>

class ShootTuoluo{
private:
    double tz_armor_width_ ;
    double tz_normal ;
public:
    TuoluoData getTuoluoData(Mat Src,ArmorDate BestArmor);
    ShootTuoluo();
private:
    double getAngle(RM_ArmorDate BestArmor);
    void firstSetTuoluo(double angle,RM_ArmorDate armor);
    //第二种陀螺检测方案
    bool ContinueSetTuoluo(Mat Src,double & angle,RM_ArmorDate BestArmor,double & R,Point2f & center,TuoluoStatus & status,TuoluoRunDirection & direction,float & spinSpeed);
    void drawImage(Mat Src,double angle,Point2f object,double R,float armorWidth);
};


struct Angle_tz{
    double angle = 90;
    float tz = 0;
    float r = 0;
};

#endif // SHOOTTUOLUO_H
