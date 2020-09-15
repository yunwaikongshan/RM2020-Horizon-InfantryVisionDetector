/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  AngleSolver.h
  *Author:  解佳朋
  *Version: 1.3.4.2007026_RC
  *Date:  2020.07.26
  *Description: 角度解算,移动和陀螺预测
  *Function List:
     1.GetArmorAngle 击打角度获取接口,出入处理后的装甲信息
     2.ComputeRecoup  移动预测
     3.ComTuoluoShootPosition  陀螺检测
     4.ShootAdjust  相机坐标转换到枪口坐标,将y坐标转换成绝对坐标
     5.ComputeShootTime 计算击打时间和弹道缓冲(不考虑空气阻力等影响)
     第1种移动预测方案,使用绝对坐标(已弃用):
     6.SetKF 滤波控制接口
     7.BufferSetFilter缓冲状态接口
     8.FirstFind    首次发现目标,各项初始化
     9.FirstSetFilter   第一次连续滤波,卡尔曼对象初始化
     10.SetSecOrderKF   二阶加速度滤波(弃用)
     第2种移动预测方案,采用角速度(更稳定,对测距精度要求更低,缺点为不符合实际运动规律):
     11.angle_FirstFind 角度滤波初始化
     12.angle_FirstSetFilter    首次角度滤波
     13.setAngleForceParam  设置角度滤波卡尔曼参数
  * Others:
        包含陀螺判断机制和卡尔曼预测
**********************************************************************************/
#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H

#include<include/Variables.h>
#include<include/Filter.h>
#include<include/ShootTuoluo.h>
#include<include/RemoteController.h>
#include<include/DrawCurve.h>

#define G 9.8


class AngleSolver
{
public:
    AngleSolver();
    void GetArmorAngle(Mat Src,  struct ArmorDate & BestArmor, Camera _camera,struct CarData CarDatas);

    void BufferSetFilter(struct RM_ArmorDate & BestArmor,struct CarData CarDatas);                       //击打缓冲

    Angle_t ComputeShootTime(float tx, float ty, float distance,struct CarData CarDatas);                                                                                      //计算击打时间及仰角

private:

    float fBigArmorWidth;
    float fBigArmorHeight;
    float fSmallArmorWidth;
    float fSmallArmorHeight;        //原5.5cm

    //坐标系转换
    float ptz_camera_y ;                  //相机与云台垂直方向距离差
    float ptz_camera_z;                  //相机与云台轴向方向距离差
    float ptz_camera_x;                    //相机与云台水平方向距离差

    float ptz_camera_pitch ;            //对应绕x旋转角度,弧度，角度乘0.017453
    float ptz_camera_yaw;               //对应绕y旋转角度，弧度
    float ptz_camera_roll ;                 //对应绕z旋转角度，弧度

    void GetPoint2D( RM_ArmorDate & BestArmor,std::vector<cv::Point2f>&point2D);
    void GetPoint3D( RM_ArmorDate & BestArmor,std::vector<cv::Point3f>&point3D);
    void CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, ArmorDate & BestArmor,Camera camera);
    void ComputeRecoup(Mat Src,ArmorDate & BestArmor,CarData CarDatas,Point3f RelativePoisition);                                                                      //弹道补偿计算
    Point3f SetKF(Mat Src,ArmorDate & BestArmor,CarData CarDatas,double t);
    void FirstFind(RM_ArmorDate BestArmor,CarData carDatas);                                                                                                                             //首次识别
    void FirstSetFilter(RM_ArmorDate & BestArmor,CarData carDatas);                            //首次击打相同目标
    void ContinueSetFilter(RM_ArmorDate & BestArmor,CarData carDatas);                  //连续击打同一目标
    void ShootAdjust(float & tx,float & ty,float & tz,float Carpitch,float Caryaw);                                                                                                   //相机坐标与枪口坐标转换
    void AbsToRelative(const float tx,const float ty,const float tz, float Carpitch, float Caryaw,float & pitch,float & yaw);                         //由绝对坐标得到相对角度
    Point3f GetAbsToRelative(Point3f p, float Carpitch, float Caryaw);
    //利用角速度设置卡尔曼预测
    void angle_FirstFind(RM_ArmorDate BestArmor,CarData carDatas);                                         //首次识别
    void angle_FirstSetFilter(RM_ArmorDate & BestArmor,CarData carDatas);                            //首次击打相同目标
    void angle_ContinueSetFilter(RM_ArmorDate & BestArmor,CarData carDatas);                  //连续击打同一目标
    void  setAngleForceParam(KF_two & KF);                                                                                              //设置滤波参数

    //陀螺击打
    void ComTuoluoShootPosition(RM_ArmorDate & BestArmor,TuoluoData tuoluoData,double nowTime,double oldTime,CarData carDatas);

    //二阶滤波
    void SetSecOrderKF(Mat Src,ArmorDate & BestArmor, Point3f Poisition,float v_x,float v_y,float v_z,float t);
    void FirstSetSecOrderKF(Point3f Poisition,float v_x,float v_y,float v_z);
    void ContinueSetSecOrderKF(Mat Src,RM_ArmorDate & BestArmor,Point3f Poisition,float v_x,float v_y,float v_z,float t);




    //长焦参数
    //哨兵
    //相机内参

            //640*480,6mm
//        cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
//               1824.604338923094019, 0, 335.371822196209507,
//              0, 1823.461537190771651, 275.500352800671635,
//                0, 0, 1);
//        cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.357423557883196 ,-0.533751270034913 , -0.003780017804386 , -0.006254250487621 , 0.000000000000000);

            //640*480,4mm
//            cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
//                   773.47793, 0, 348.49462,
//                  0, 766.26321, 275.62439,
//                    0, 0, 1);
//            cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  0.12192 ,-0.26506 , 0.00202 , 0.01579 , 0.000000000000000);
            //大恒2*2
            cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
                   623.94494, 0, 308.05780,
                  0, 626.19679, 255.94809,
                    0, 0, 1);
            cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.22196  , 0.18886 ,  -0.00333 ,  -0.00029,  0.00000);
//    cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
//           1180.18826, 0, 610.62537,
//          0, 1181.75307, 507.94374,
//            0, 0, 1);
//    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.15835 ,  0.09077 ,  0.00057 ,  0.00749,  0.00000);



            //1280*720
//        cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
//                   2834.73041, 0, 733.47325,
//                  0, 2830.77643, 412.41369,
//                    0, 0, 1);
//            cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<  -0.45572,1.27325  , 0.00526   ,-0.00040 , 0.00000);

    //英雄
//    cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
//                            573.8442689411178, 0, 313.5415193229257,
//                             0, 571.8607195828303, 268.4912125622909,
//                             0, 0, 1);
//    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<0.02798642163767799, -0.047521539044798, -2.789782380159369e-05, -0.004344517742473584, -0.1611927545248823);

    //步兵
//    cv::Mat caremaMatrix = (cv::Mat_<float>(3, 3) <<
//                            557.2267192211751, 0, 310.7867944540003,
//                             0, 555.988272597227, 238.7766536772089,
//                             0, 0, 1);
//    cv::Mat distCoeffs = (cv::Mat_<float>(1, 5) <<-0.01609636741207191, 0.1402802012124325, -0.001807390481900501, 0.002016737706806919, -0.2842695420037122);

    //相机内参
    cv::Mat GuangcaremaMatrix = (cv::Mat_<float>(3, 3) <<
           562.606295264005894, 0, 333.517503364747881,
          0, 562.440209521245038, 286.540368213054762,
            0, 0, 1);
    cv::Mat GuangdistCoeffs = (cv::Mat_<float>(1, 5) << -0.371768456212426 , 0.178098224531363 , -0.001402844076506 , -0.003672829990462 , 0.000000000000000);


};
#endif // ANGLESOLVER_H
