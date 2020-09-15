/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  BuffAngleSolver.h
  *Author:  解佳朋
  *Version: 1.3.1.200517_RC
  *Date:  2020.05.17
  *Description: 大符识别
  *Function List:
     1.BuffModeSwitch   大符识别接口,返回用于圆心计算的三个位置和当前待打击位置的数组
     2.PreDelBuff   大符识别预处理
     3.FindBestBuff    寻找并存入所有的大符目标
     4.GetShootBuff    寻找待击打的大符目标
**********************************************************************************/
#ifndef BUFFANGLESOLVER_H
#define BUFFANGLESOLVER_H
#include<include/Variables.h>
#include<include/AngleSolver.h>
#include<include/Filter.h>

class BuffAngleSolver{
private:
    float BuffWidth;
    float BuffHeight;
    float ChassisToPtz_x;
    float ChassisToPtz_y;
    float ChassisToPtz_z;
    float ChassisToPtz_Pitch;
    float ChassisToPtz_Yaw;
    float ChassisToPtz_Roll;

    void GetPoint2D( RM_BuffData & BestArmor,std::vector<cv::Point2f>&point2D);
    void GetPoint3D( RM_BuffData & BestArmor,std::vector<cv::Point3f>&point3D);
    void CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, RM_BuffData & BestArmor);
    void GetBuffrAngle(RM_BuffData & BestArmor);
    float GetBuffCentralAngle(Point3f ObjectPoistion,Point3f CircleCenter,Point3f LowerBoundaryPoint,Point3f LefterBoundaryPoint);
    Point3f GetBuffCenter(RM_BuffData * BestArmor);
    Point3f GetAveBuffCenter();
    Point3f GetNormalVector(Point3f new_vector,Point3f old_vector);
    float ShootAdjust(float & x,float & y,float & z,float pitch,float yaw);
    void BuffAngleSpeedFilter(float & AngleSpeed,KF_two Filter,CarData carDatas);
    Point3f GetShootPoistion(float angle,Point3f center,float r);
    void ChassisToPtz(RM_BuffData & BestArmor);
    Angle_t ComputeBuffShootTime(float tx, float ty, float distance,struct CarData CarDatas);

    //底盘usb
    cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
           640.50006, 0, 305.09497,
          0, 641.66663, 255.88171,
            0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.22324,0.19562,-0.00020  , 0.00282 , 0.00000);


public:
    BuffAngleSolver();                  //构造函数

    void GetBuffShootAngle(RM_BuffData * BestArmor,BuffStatus BuffShootStatus,CarData carDatas);

};

#endif // BUFFANGLESOLVER_H
