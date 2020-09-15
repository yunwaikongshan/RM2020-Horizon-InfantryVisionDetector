/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  AngleSolver.cpp
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
#include<include/AngleSolver.h>
#define SHOOT_DELAY_TIME 120                //发弹延迟
#define MIN_FORCE_DISTANCE_CHA  10000 //符合使用的最小距离差
#define USING_ANGLE_FILTER                  //使用角度滤波
#define RECORD_RUN_TIMES 1                //记录移动数组容量
#define USING_POISITION_FILTER                  //使用位置滤波
//#define USING_POISITION_A

//tz滤波(弃用)
float _time = 0;
float v = 0;
float vz = 0;


//定义卡尔曼类型
KF_two KF_forecast;             //预测一阶滤波
KF_two KF_tz;
KF_two KF_tuoluo;
KF_two KF_SecOrder;        //预测二阶滤波

//角度滤波
KF_two KF_AngleForeCast;            //角度滤波
/********************击打置信级数***************************/
#define MIN_SHOOT_TRUST_LEVEL 3         //最小满足击打的置信等级
int ForceShootTrustlevel = 0;
float RunSpeed[MIN_SHOOT_TRUST_LEVEL];
/********************击打置信级数***************************/

/********************第一版(角度)***************************/
//设置tz卡尔曼滤波
bool isSetKF_tz = false;
bool isSetAngleKF = false;          //是否设置过角度滤波
float tz_old;
//定义运动绝对角度
float realCarPitch = 0;
float realCarYaw = 0;
//用于缓冲计算
float SendPitch = 0;                //记录上一帧pitch发送角度量
float SendYaw = 0;                //记录上一帧yaw发送角度量
//预测计算保留量
CarData old_carDatas;                //上次收数
float old_Pitch= 0;            //上级Pitch相对角度
float old_Yaw = 0;              //上级Yaw相对角度

CarData buffer_old_carDatas ;     //缓冲保留计算量
bool IsHaveBuffer = false;

//保留最新几帧的目标移动速度
float PitchRunSpeed[RECORD_RUN_TIMES];
float YawRunSpeed[RECORD_RUN_TIMES];
int RecornRunTimes = 0;

/*****************************************************/

/********************第二版(位置)***************************/
float v_tx_old;                     //线速度保留量
float v_ty_old;
float v_tz_old;
float p_tx_old;                  //位置保留量
float p_ty_old;
float p_tz_old;

CarData carDatasOld;                //上次收数
Point3f old_Poisition;              //上级的二阶预测的位置
Point3f old_objectP;
/*****************************************************/
//陀螺连续保留量
ShootTuoluo shootTuoluo;
bool isFirstShootTuoluo = true;     //是否当前情况为首次发现陀螺目标
float old_angle = 0;                            //上一次陀螺角度
float old_SpinSpeed = 0;                 //上一次陀螺角速度
bool isSetKF_tuoluo = false;         //是否设置过陀螺卡尔曼矩阵
bool isSetKF_SecOrder = false;         //是否设置过陀螺卡尔曼矩阵
//记录前后时间
double  old_CarTime = 0;

AngleSolver::AngleSolver(){
    //类成员初始化
    fBigArmorWidth=22.5;
    fBigArmorHeight=8.85;
    fSmallArmorWidth=13.5;
    fSmallArmorHeight=8.55;        //原5.5cm

    //坐标系转换
    ptz_camera_y = 2;                  //相机与云台垂直方向距离差
    ptz_camera_z =10.5;                  //相机与云台轴向方向距离差
    ptz_camera_x = 2;                    //相机与云台水平方向距离差

    ptz_camera_pitch = 0*PI/180;            //对应绕x旋转角度,弧度，角度乘0.017453
    ptz_camera_yaw = 0*PI/180;               //对应绕y旋转角度，弧度
    ptz_camera_roll = 1.2*PI/180;                 //对应绕z旋转角度，弧度

    //测距滤波初始化,仅需执行一次
    if(!isSetKF_tz){
        isSetKF_tz = true;
        //状态协方差矩阵附初值

        Eigen::MatrixXd P_in = Eigen::MatrixXd(2,2);
        P_in << 1.0, 0.0,
                        0.0,1.0;
        KF_tz.P = P_in;

         //过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(2,2);
        Q_in<<1.0, 0.0,
                        0.0,1e-1;
        KF_tz.Q = Q_in;

          //测量矩阵附初值
        Eigen::MatrixXd H_in(1,2);
        H_in<<1.0, 0.0;
        KF_tz.H = H_in;

        //测量噪声矩阵附初值
        Eigen::MatrixXd R_in(1,1);
        R_in<<10;
        KF_tz.R = R_in;

        Eigen::MatrixXd F_in(2,2);
        F_in<<1.0,1.0,
                0.0,1.0;
        KF_tz.F = F_in;
    }
    //陀螺滤波初始化,仅需执行一次
    if(!isSetKF_tuoluo){
        isSetKF_tuoluo = true;
        //状态协方差矩阵附初值

        Eigen::MatrixXd P_in = Eigen::MatrixXd(2,2);
        P_in << 1.0, 0.0,
                        0.0,1.0;
        KF_tuoluo.P = P_in;

         //过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(2,2);
        Q_in<<1.0, 0.0,
                        0.0,1e-1;
        KF_tuoluo.Q = Q_in;

          //测量矩阵附初值
        Eigen::MatrixXd H_in(1,2);
        H_in<<1.0, 0.0;
        KF_tuoluo.H = H_in;

        //测量噪声矩阵附初值
        Eigen::MatrixXd R_in(1,1);
        R_in<<8500;
        KF_tuoluo.R = R_in;

        Eigen::MatrixXd F_in(2,2);
        F_in<<1.0,1.0,
                0.0,1.0;
        KF_tuoluo.F = F_in;
    }
    //预测二阶滤波
//    if(!isSetKF_SecOrder){

//    }

    //角度预测滤波
    if(!isSetAngleKF){
        isSetAngleKF = true;
        setAngleForceParam(KF_AngleForeCast);
    }
}

/**
 * @brief AngleSolver::GetArmorAngle        角度解算
 * @param Src
 * @param _camera  摄像头信息
 * @param CarDatas  内角度现应为弧度
 * @remarks 计算绝对的打击角度时,ty坐标需使用绝对坐标,而tx和tz坐标需使用相对坐标
 */
void AngleSolver::GetArmorAngle(Mat Src, ArmorDate & BestArmor,Camera _camera,CarData CarDatas ){
    std::vector<cv::Point2f>point2D;
    std::vector<cv::Point3f>point3D;

    GetPoint2D(BestArmor.Armor,point2D);                                                                                                     //矩形转换为2d坐标
    GetPoint3D(BestArmor.Armor,point3D);                                                                                                     //矩形转换为3d坐标
    CountAngleXY(point2D,point3D,BestArmor,_camera);                                                                             //解决pnp问题
    //相机坐标与云台初始枪口坐标转换,坐标系做云台当前角度反向旋转得到绝对坐标
//    return;
    Point3f RelativePoisition = Point3f(BestArmor.Armor.tx,BestArmor.Armor.ty,BestArmor.Armor.tz);              //保留相对坐标
    ShootAdjust(BestArmor.Armor.tx,BestArmor.Armor.ty,BestArmor.Armor.tz,CarDatas.pitch,-CarDatas.yaw);     //转换为绝对坐标
//    return;
    RelativePoisition.y = BestArmor.Armor.ty;                       //ty坐标使用绝对坐标
//    return;
    //判断陀螺
    TuoluoData tuoluoData = shootTuoluo.getTuoluoData(Src,BestArmor);
    if(tuoluoData.isTuoluo){
        //陀螺击打,
        if(tuoluoData.runDirection == LEFT){                        //标志左右方向
            circle(Src,Point(20,20),5,Scalar(255,0,0),-1,8);
        }else{
            circle(Src,Point(20,20),5,Scalar(0,0,255),-1,8);
        }
        ComTuoluoShootPosition(BestArmor.Armor,tuoluoData,CarDatas.BeginToNowTime,old_CarTime,CarDatas);
        //减去当前角度量
//        BestArmor.Armor.pitch -= CarDatas.pitch;
//        BestArmor.Armor.yaw -= CarDatas.yaw;
    }else{
        //移动预测

        ComputeRecoup(Src,BestArmor,CarDatas,RelativePoisition);
//        BestArmor.Armor.pitch += ;
        if(BestArmor.Armor.pitch>0){
//            BestArmor.Armor.pitch += 0.9;
        }else if(BestArmor.Armor.pitch<0){
//            BestArmor.Armor.pitch -= 0.9;
        }
//        BestArmor.Armor.pitch = -atan2( BestArmor.Armor.ty, BestArmor.Armor.tz);
//         BestArmor.Armor.yaw = -atan2( BestArmor.Armor.tx, BestArmor.Armor.tz);
    }

    //根据绝对坐标得到当前相对的应旋转的角度
//    AbsToRelative(BestArmor.Armor.tx,BestArmor.Armor.ty,BestArmor.Armor.tz,CarDatas.pitch,CarDatas.yaw,BestArmor.Armor.pitch,BestArmor.Armor.yaw);

//    BestArmor.Armor.pitch += 0.8;
    if(BestArmor.status!=buffering){
        //当前不处于缓冲状态就更新记录时间
        //缓冲状态应不参与滤波计算,根据最后的滤波所得速度计算缓冲时间内绝对坐标,由绝对坐标计算相对角度,会产生丢失目标僵直状态
        old_CarTime = CarDatas.BeginToNowTime;
    }
}

void AngleSolver::GetPoint2D( RM_ArmorDate & BestArmor,std::vector<cv::Point2f>&point2D){

    cv::Point2f lu,ld,ru,rd;        //right_down right_up left_up left_down

    lu = BestArmor.point[0];
    ld = BestArmor.point[3];
    ru = BestArmor.point[1];
    rd = BestArmor.point[2];

//     cout<<"lu:"<<lu<<endl;
    point2D.clear();///先清空再存入
    point2D.push_back(lu);
    point2D.push_back(ru);
    point2D.push_back(rd);
    point2D.push_back(ld);
}

//矩形转换为3d坐标                                                                                                                                                                                             3
void AngleSolver::GetPoint3D( RM_ArmorDate & BestArmor,std::vector<cv::Point3f>&point3D)
{
    float fHalfX=0;
    float fHalfY=0;
    if(BestArmor.IsSmall)                                  ///大装甲
    {
        cout<<"小"<<endl;
        fHalfX=fSmallArmorWidth/2.0;
        fHalfY=fSmallArmorHeight/2.0;
    }
    else{                                                                                                ///小装甲
//         cout<<"大"<<endl;
        fHalfX=fBigArmorWidth/2.0;
        fHalfY=fBigArmorHeight/2.0;
    }
    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY,0.0));
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY,0.0));
    point3D.push_back(cv::Point3f(fHalfX,fHalfY,0.0));
    point3D.push_back(cv::Point3f(-fHalfX,fHalfY,0.0));

}

//pnp转换,得到目标坐标
void AngleSolver::CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, ArmorDate & BestArmor,Camera camera){
    cv::Mat rvecs=cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs=cv::Mat::zeros(3,1,CV_64FC1);

//     cv::solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs,false,SOLVEPNP_UPNP);

    if(camera == UsingGuang){
        //广角解算
        cv::solvePnP(point3D,point2D,GuangcaremaMatrix,GuangdistCoeffs,rvecs,tvecs);
    }else{
        //长焦解算
        cv::solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs);
    }

//     cv::solvePnPRansac(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs,false,100,8.0,0.99,noArray(),SOLVEPNP_UPNP);
//     cout<<"旋转向量："<<rvecs<<endl;
//     cout<<"平移向量："<<tvecs<<endl;
    double tx = tvecs.ptr<double>(0)[0];
    double ty = -tvecs.ptr<double>(0)[1];
    double tz = tvecs.ptr<double>(0)[2];

    if(BestArmor.status == pattern::FirstFind){
        //各个值初始化
        tz_old = tz;
        KF_tz.is_set_x = false;
    }else{
        //对tz坐标进行滤波,使距离信息更稳定(当前基本已弃用)
        if(!KF_tz.is_set_x){
            Eigen::VectorXd _x(2,1);
            _x<<tz , tz - tz_old;
            KF_tz.set_x(_x);
        }else{
            KF_tz.Prediction(KF_tz.F);
            Eigen::VectorXd _z(1,1);
            _z<<tz;
            KF_tz.update(_z,KF_tz.F);
        }
        tz = KF_tz.get_x()(0);
//         cout<<"滤波tz："<<tz<<"   滤波tz速度："<<KF_tz.get_x()(1)<<endl;

    }


    BestArmor.Armor.tx = tx;
    BestArmor.Armor.ty = ty;
    BestArmor.Armor.tz = tz;

    BestArmor.Armor.pitch = atan2( BestArmor.Armor.ty, BestArmor.Armor.tz)*180/PI;
     BestArmor.Armor.yaw = atan2( BestArmor.Armor.tx, BestArmor.Armor.tz)*180/PI;
}

/**
 * @brief AngleSolver::ComputeRecoup        移动预测弹道补偿计算
 * @param BestArmor
 * @param CarDatas
 * @param RelativePoisition             其中tx和tz为相对坐标,ty为绝对坐标
 * @remark CarDatas中角度传入弧度
 */
void AngleSolver::ComputeRecoup(Mat Src,ArmorDate &BestArmor, CarData CarDatas,Point3f RelativePoisition){


    //抬升角度计算,得到绝对角度
    Angle_t ShootBuff = ComputeShootTime(RelativePoisition.x,RelativePoisition.y,RelativePoisition.z,CarDatas);
    float old_CurveValue = RelativePoisition.y;                         //波形绘制
     float old_CurveValue2 = RelativePoisition.z;                         //波形绘制
//    BestArmor.Armor.pitch -= CarDatas.pitch;
            //    ShootBuff.t = 0;
//    BestArmor.Armor.yaw = atan2(tx,tz)*180/CV_PI;
//    BestArmor.Armor.pitch = atan2(ty,tz)*180/CV_PI;
     if(ShootBuff.t == 0){
         //出现可能性极小的无解情况,出现往往时算法错误
//         BestArmor.Armor.yaw = atan2(BestArmor.Armor.tx,BestArmor.Armor.tz)*180/CV_PI;
//         BestArmor.Armor.pitch = atan2(BestArmor.Armor.ty,BestArmor.Armor.tz)*180/CV_PI;
         cout<<"角度计算无解"<<endl;
         ShootBuff.t = 15;

     }else{

         //传参
         BestArmor.Armor.pitch = ShootBuff.pitch;
         BestArmor.Armor.yaw = ShootBuff.yaw;
     }

#ifdef USING_POISITION_FILTER                   //使用位置滤波适合大恒相机
     //卡尔曼滤波
     Point3f poisition = SetKF(Src,BestArmor,CarDatas,ShootBuff.t);
     float y = poisition.y;                 //保留绝对y坐标
     poisition = GetAbsToRelative(poisition,-CarDatas.pitch,CarDatas.yaw);              //将绝对坐标转化为相对坐标
     float t = poisition.y;
     poisition.y = y;                           //y使用绝对坐标
     Angle_t AT = ComputeShootTime(poisition.x,poisition.y,poisition.z,CarDatas);       //计算打击时间和偏向角
     //更新打击偏向角
     BestArmor.Armor.pitch = AT.pitch  ;
     BestArmor.Armor.yaw = AT.yaw;
//     poisition.y = t;
     SendPitch = AT.pitch;
     SendYaw = AT.yaw;
     BestArmor.Armor.tx = poisition.x;
     BestArmor.Armor.ty = poisition.y;
     BestArmor.Armor.tz = poisition.z;
     old_objectP.y = y;                                                 //保留位置坐标,为计算缓冲做准备
     old_objectP.x = BestArmor.Armor.tx;
     old_objectP.z = BestArmor.Armor.tz;
     //绘制波形
     DrawCurve drawCurve;
     if(KF_forecast.is_set_x){
         Angle_t AT2 = ComputeShootTime(poisition.x,old_CurveValue + v*_time,old_CurveValue2+vz*_time,CarDatas);
         drawCurve.InsertData(AT2.pitch,BestArmor.Armor.pitch,"Actual value","Predictive value");
//         drawCurve.InsertData(KF_forecast.x_(1));
//         drawCurve.InsertData(old_CurveValue);

     }
//     old_carDatas = CarDatas;
     return;
#endif

#ifdef USING_ANGLE_FILTER                               //使用角度滤波,适合普通usb相机
     if(BestArmor.status == pattern::buffering){
         //缓冲
         //得到预测量
         if(KF_AngleForeCast.is_set_x){
             float time =     Recive.getClock() - CarDatas.BeginToNowTime;
             float runPitch = (CarDatas.pitch - buffer_old_carDatas.pitch);
             float runYaw = (CarDatas.yaw - buffer_old_carDatas.yaw);
             BestArmor.Armor.pitch = KF_AngleForeCast.x_(2)/1000*(SHOOT_DELAY_TIME+ShootBuff.t+time + (CarDatas.BeginToNowTime - buffer_old_carDatas.BeginToNowTime)) + old_Pitch - runPitch;
             BestArmor.Armor.yaw = KF_AngleForeCast.x_(3)/1000*(SHOOT_DELAY_TIME+ShootBuff.t+time+ (CarDatas.BeginToNowTime - buffer_old_carDatas.BeginToNowTime)) + old_Yaw- runYaw;
//             old_carDatas = CarDatas;
             IsHaveBuffer = true;
         }
     }else{
         if(BestArmor.status == pattern::FirstFind){
             //首次发现目标,初始化
             angle_FirstFind(BestArmor.Armor,CarDatas);
         }else if(BestArmor.status == pattern::Shoot){
             //连续射击
             if(KF_AngleForeCast.is_set_x){
                 //当前已初始化
                 angle_ContinueSetFilter(BestArmor.Armor,CarDatas);
             }else{
                 //未初始化,第一次连续滤波初始化
                 angle_FirstSetFilter(BestArmor.Armor,CarDatas);
             }

             //计算得到预测量
             float time =     Recive.getClock() - CarDatas.BeginToNowTime;
             BestArmor.Armor.pitch += KF_AngleForeCast.x_(2)/1000.0*(SHOOT_DELAY_TIME+ShootBuff.t+time);
             BestArmor.Armor.yaw += KF_AngleForeCast.x_(3)/1000.0*(SHOOT_DELAY_TIME+ShootBuff.t+time);
             IsHaveBuffer = false;

         }
     }
#endif
}

/**
 * @brief AngleSolver::ShootRevice      相机与枪口坐标调整
 * @param tx
 * @param ty
 * @param tz
 * 旋转时以逆时针为正                pitch转换为绝对角度计算抬升
 */
void AngleSolver::ShootAdjust(float &tx, float &ty, float &tz,float Carpitch,float Caryaw){
    //角度转弧度
    Carpitch *=PI/180;
    Caryaw *=PI/180;

    //绕roll轴旋转，即为绕z轴旋转
    Eigen::MatrixXd r_Roll(3,3);
    r_Roll<<1 , 0 , 0,
                    0 , cos(ptz_camera_roll) , sin(ptz_camera_roll),
                    0 , -sin(ptz_camera_roll) , cos(ptz_camera_roll);
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(ptz_camera_pitch) , 0 , -sin(ptz_camera_pitch),
                        0 , 1 , 0,
                        sin(ptz_camera_pitch) , 0 , cos(ptz_camera_pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(ptz_camera_yaw) , sin(ptz_camera_yaw) , 0,
                     -sin(ptz_camera_yaw) , cos(ptz_camera_yaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);          //按z，x，y传入，即变化对应到左手坐标系
    original<<tz,tx,ty;

    //平移变换,先旋转再平移
    Eigen::VectorXd translation(3,1);
    translation<<ptz_camera_z,ptz_camera_x,ptz_camera_y;
    original = original + translation;


    Eigen::VectorXd change(3,1);
    //旋转变换
    change =  r_Roll * original;
    change = r_Pitch*change;
    change = r_Yaw*change;
    //以上部分的偏移参数调节


    //去掉车云台旋转相对影响,坐标系转换到相对初始位的绝对坐标
    //pitch转换
    Eigen::MatrixXd r_pitch_car(3,3);
    r_pitch_car<<cos(Carpitch) , 0 , -sin(Carpitch),
                        0 , 1 , 0,
                        sin(Carpitch) , 0 , cos(Carpitch);
//    change = r_pitch_car*change;
    //yaw转换
    Eigen::MatrixXd r_yaw_car(3,3);
    r_yaw_car<<cos(Caryaw) , sin(Caryaw) , 0,
                            -sin(Caryaw) , cos(Caryaw) , 0 ,
                              0 , 0 , 1;
    change = r_pitch_car*change;

#ifdef USING_POISITION_FILTER
    //位置滤波,角度滤波无需将ty和ty转换为绝对角度
    change = r_yaw_car*change;
#endif//USING_POISITION_FILTER

    tx = change(1);
    ty = change(2);
    tz = change(0);

//    tx += ptz_camera_x;
//    ty += ptz_camera_y;
//    tz += ptz_camera_z;

}

Point3f AngleSolver::GetAbsToRelative(Point3f p, float Carpitch, float Caryaw){
    Carpitch *=PI/180;
    Caryaw *=PI/180;
    cout<<"转换前:x:"<<p.x<<"  y:"<<p.y<<" z:"<<p.z<<" Carpitch:"<<Carpitch<<"  Caryaw:"<<Caryaw<<endl;
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(Carpitch) , 0 , -sin(Carpitch),
                        0 , 1 , 0,
                        sin(Carpitch) , 0 , cos(Carpitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(Caryaw) , sin(Caryaw) , 0,
                     -sin(Caryaw) , cos(Caryaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);          //按z，x，y传入，即变化对应到左手坐标系
    original<<p.z,p.x,p.y;

    Eigen::VectorXd change(3,1);
    //旋转变换
    change = r_Pitch*original;
    change = r_Yaw*change;

    p.x = change(1);
    p.y = change(2);
    p.z = change(0);
    cout<<"转换后:x:"<<p.x<<"  y:"<<p.y<<" z:"<<p.z<<endl;
    return p;
}

/**
 * @brief AngleSolver::ComputeShoot         计算射击缓冲及时间
 * @param tx
 * @param ty
 * @param tz
 * @return
 */
Angle_t AngleSolver::ComputeShootTime(float tx, float ty, float distance,struct CarData CarDatas){
    //g表示重力加速度，LevelDistance表示水平距离(sqrt(pow(tz,2)+pow(tx,2)),HeightDistance为垂直高度ty，v为射速，tan_angle为所要求俯仰角的正切值
    //计算公式: -0.5g*pow(LevelDistance,2)/pow(V,2)*pow(tan_angle,2) + tan_angle*LevelDistance - 0.5g*pow(LevelDistance,2)/pow(V,2) - HeightDistance
    //计算时间使用t = LevelDistance/(v*cos_angle)


    //单位转换
    tx /= 100.0;
    ty /= 100.0;
    float tz = distance/100.0;
//    cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<distance<<endl;
    float speed = CarDatas.ShootSpeed;
//    speed=20;
    double a = -0.5*G*(pow(tz,2)+pow(tx,2));
    double b = sqrt((pow(tz,2)+pow(tx,2)))*pow(speed,2);
    double c = -0.5*G*(pow(tz,2)+pow(tx,2)) - pow(speed,2)*ty;
    //判别式
    double Discriminant = pow(a,2)+pow(b,2)-4*a*c;
//    cout<<"判别式:"<<Discriminant<<"   a:"<<a<<"   b:"<<b<<"   c:"<<c<<endl;
    Angle_t ShootBuff = {0,0,0,0};
    if(Discriminant<0)return ShootBuff;
    double angle_tan_1 = atan((-b + sqrt(Discriminant))/(2*a))*180/PI;
    double angle_tan_2 = atan((-b - sqrt(Discriminant))/(2*a))*180/PI;
//    double angle_tan = ty/b;
//    double real_angle = fabs(angle_tan - angle_tan_1)<fabs(angle_tan - angle_tan_2)?angle_tan_1:angle_tan_2;
    //角度取舍,并转换为相对角度
    cout<<"计算所得打击角度1:"<<angle_tan_1<<"  计算所得打击角度2:"<<angle_tan_2<<endl;
    if(fabs(angle_tan_1)<=fabs(angle_tan_2)&&fabs(angle_tan_1)<45){
        ShootBuff.pitch = angle_tan_1 - CarDatas.pitch;
    }else if(fabs(angle_tan_2)<45){
        ShootBuff.pitch = angle_tan_2 - CarDatas.pitch;
    }else{      //都不符合要求
        cout<<"计算解不符合实际"<<endl;
        return ShootBuff;
    }

//    real_angle = atan(real_angle)*180/PI;
//    ShootBuff.pitch = real_angle;
    ShootBuff.yaw = atan2(tx,tz)*180/CV_PI;
    cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<tz<<"yaw"<<ShootBuff.yaw<<"最小角度"<<atan2(ty,tz)*180/PI<<endl;
    //处理当前视角为后方时
//    if(distance<0){
//        if(ShootBuff.pitch>=0){
//            ShootBuff.pitch += -180;
//        }else{
//            ShootBuff.pitch += 180;
//        }
//        if(ShootBuff.yaw>=0){
//            ShootBuff.yaw += -180;
//        }else{
//            ShootBuff.yaw += 180;
//        }
//    }
//    ShootBuff.t = b/(speed*cos(ShootBuff.pitch*PI/180))*1000;
    ShootBuff.t = tz/(speed*cos(ShootBuff.pitch*PI/180))*1000;
    cout<<"击打时间:"<<ShootBuff.t<<endl;
    return ShootBuff;
}

Point3f AngleSolver::SetKF(Mat Src,ArmorDate &BestArmor, CarData CarDatas,double t){
    if(BestArmor.status == pattern::FirstFind){
        //清除波形保留信息
        DrawCurve drawCurve;
        drawCurve.ClearSaveData();

        FirstFind(BestArmor.Armor,CarDatas);
        KF_forecast.is_set_x = false;
        //状态协方差矩阵重新复位
        Eigen::MatrixXd P_in = Eigen::MatrixXd(6,6);
        P_in << 1.0, 0.0, 0.0, 0.0,0.0,0.0,
                0.0, 1.0, 0.0, 0.0,0.0,0.0,
                0.0, 0.0, 1.0, 0.0,0.0,0.0,
                0.0, 0.0, 0.0, 1.0,0.0,0.0,
               0.0, 0.0, 0.0, 0.0,1.0,0.0,
               0.0, 0.0, 0.0, 0.0,0.0,1.0;
      KF_forecast.P = P_in;
        return Point3f(BestArmor.Armor.tx,BestArmor.Armor.ty,BestArmor.Armor.tz);
    }else if(BestArmor.status == Shoot){

        if(!KF_forecast.is_set_x){
            //第一次连续射击
            cout<<"位置首次连续滤波"<<endl;
            FirstSetFilter(BestArmor.Armor,CarDatas);
        }else{
            cout<<"位置连续滤波"<<endl;
            ContinueSetFilter(BestArmor.Armor,CarDatas);
        }

    }else{
        cout<<"卡尔曼滤波状态传入出错"<<endl;
    }

    //时间为击打时间与弹道延迟之和
    double ShootTime = t + SHOOT_DELAY_TIME+Recive.getClock() - CarDatas.BeginToNowTime;
//    Eigen::MatrixXd F_in(4,4);
//    F_in<<1.0,0.0,ShootTime,0.0,
//            0.0,1.0,0.0,ShootTime,
//            0.0,0.0,1.0,0.0,
//            0.0,0.0,0.0,1.0;
//    KF_forecast.GetPrediction(F_in);
    cout<<"x速度："<<KF_forecast.x_(3)<<" y速度:"<<KF_forecast.x_(4)<<" z速度:"<<KF_forecast.x_(5)<<endl;
    cout<<"时间："<<ShootTime<<endl;

    //转换为国际单位制
//    float v_x = KF_forecast.x_(0)*10;
//     float v_y = KF_forecast.x_(1)*10;
//      float v_z = KF_forecast.x_(2)*10;

//      float a_x = KF_forecast.x_(3)*10000;
//      float a_y = KF_forecast.x_(4)*10000;
//      float a_z = KF_forecast.x_(5)*10000;

//    t= ShootTime/1000.0;

    Point3f poistion;
//    poistion.x = BestArmor.Armor.tx + KF_forecast.x_(3)/1000*ShootTime;
//    poistion.y = BestArmor.Armor.ty + KF_forecast.x_(4)/1000*ShootTime;
//    poistion.z = BestArmor.Armor.tz + KF_forecast.x_(5)/1000*ShootTime;
    poistion.x = KF_forecast.x_(0) + KF_forecast.x_(3)/1000*ShootTime;
    poistion.y = KF_forecast.x_(1) + KF_forecast.x_(4)/1000*ShootTime;
    poistion.z = KF_forecast.x_(2) + KF_forecast.x_(5)/1000*ShootTime;

    _time = ShootTime;

#ifdef USING_POISITION_A             //位置预测使用加速度,使用国际单位制
    poistion.x += (0.5*(KF_forecast.x_(3)/1000 - v_tx_old)*10*pow(ShootTime/1000.0,2))*100;
    poistion.y += (0.5*(KF_forecast.x_(4)/1000 - v_ty_old)*10*pow(ShootTime/1000.0,2))*100;
    poistion.z += (0.5*(KF_forecast.x_(5)/1000 - v_tz_old)*10*pow(ShootTime/1000.0,2))*100;
    cout<<"加速度:"<<fabs(KF_forecast.x_(3) - v_tx_old*1000)<<endl;
    if(fabs(KF_forecast.x_(3) - v_tx_old*1000)>12){
        BestArmor.Armor.IsShooting = false;
    }
    char test[100];
    sprintf(test, "a:%0.4f", fabs(KF_forecast.x_(3) - v_tx_old*1000));
    cv::putText(Src, test, cv::Point(20, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
#endif

    v_tx_old = KF_forecast.x_(3)/1000;
    v_ty_old = KF_forecast.x_(4)/1000;
    v_tz_old = KF_forecast.x_(5)/1000;



//    if(fabs(KF_forecast.x_(3))<1){
//        poistion.x = BestArmor.Armor.tx;
//        poistion.y = BestArmor.Armor.ty;
//        poistion.z = BestArmor.Armor.tz;
//    }


    Point3f p = GetAbsToRelative(poistion,CarDatas.pitch,CarDatas.yaw);
    Angle_t at = ComputeShootTime(p.x,p.y,p.z,CarDatas);
    float old_t = t;

//    while(fabs(at.t - old_t)>0.1){
//        cout<<"迭代计算角度erro:"<<fabs(at.t - t)<<endl;
//        poistion.x = KF_forecast.x_(0) + KF_forecast.x_(3)*(ShootTime - t+ at.t);
//        poistion.y = KF_forecast.x_(1) + KF_forecast.x_(4)*(ShootTime - t+ at.t);
//        poistion.z = KF_forecast.x_(2) + KF_forecast.x_(5)*(ShootTime - t+ at.t);
//        //转换会相对坐标
//        Point3f p = GetAbsToRelative(poistion,-CarDatas.pitch,-CarDatas.yaw);
//        old_t = at.t;
//        at = ComputeShootTime(p.x,p.y,p.z,CarDatas);
//    }


      cout<<"1阶滤波位置:"<<poistion.x<<"  "<<poistion.y<<"  "<<poistion.z<<endl;

//    BestArmor.Armor.tx += (v_x*t)*100;
//    BestArmor.Armor.ty += (v_y*t)*100;
//    BestArmor.Armor.tz += (v_z*t )*100;

//    BestArmor.Armor.tx +=  KF_forecast.x_(0)*t + KF_forecast.x_(3)*t*t/2;
//    BestArmor.Armor.ty +=  KF_forecast.x_(1)*t + KF_forecast.x_(4)*t*t/2;
//    BestArmor.Armor.tz +=  KF_forecast.x_(2)*t + KF_forecast.x_(5)*t*t/2;

//    if(BestArmor.Armor.tx>0)
//    BestArmor.Armor.pitch = ;
//    BestArmor.Armor.yaw = KF_forecast.x_(3)*ShootTime + BestArmor.Armor.yaw;
      return poistion;
}

/**
 * @brief AngleSolver::FirstFind    第一次发现目标
 * @param BestArmor
 */
void AngleSolver::FirstFind(RM_ArmorDate BestArmor,CarData carDatas){
    cout<<"第一次发现目标"<<endl;
    //状态保留
    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;

}

/**
 * @brief AngleSolver::FirstSetFilter 卡尔曼滤波第一次赋初值
 * @param BestArmor
 * @param pitch_old
 * @param yaw_old
 * @param t
 */
void AngleSolver::FirstSetFilter(RM_ArmorDate & BestArmor,CarData carDatas){
    cout<<"第一次滤波"<<endl;
    double t = carDatas.BeginToNowTime - carDatasOld.BeginToNowTime;        //计算两帧经过的时间
    if(t == 0){
        t = 15;         //未收到数
    }
    //目标移动速度,单位为cm/ms
    float v_tx_now = (BestArmor.tx - p_tx_old)/t;
    float v_ty_now =( BestArmor.ty - p_ty_old)/t;
    float v_tz_now = (BestArmor.tz - p_tz_old)/t;

    Eigen::VectorXd x(6,1);
    x<<BestArmor.tx,BestArmor.ty,BestArmor.tz,v_tx_now*1000,v_ty_now*1000,v_tz_now*1000;
    KF_forecast.set_x(x);

    //传参
    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;

    old_objectP.x = p_tx_old;
    old_objectP.y = p_ty_old;
    old_objectP.z = p_tz_old;

    v_tx_old = v_tx_now;
    v_ty_old = v_ty_now;
    v_tz_old = v_tz_now;

}


void AngleSolver::ContinueSetFilter(RM_ArmorDate & BestArmor,CarData carDatas){
    cout<<"连续滤波"<<endl;
    double t = carDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    //不发数的时候t=0
    if(t == 0){
        t = 15;         //单位ms
    }
    //得到真实测量值
    //目标移动速度,单位为cm/ms
    float v_tx_now = (BestArmor.tx - p_tx_old)/t;
    float v_ty_now = (BestArmor.ty - p_ty_old)/t;
    float v_tz_now = (BestArmor.tz - p_tz_old)/t;

    v = v_ty_now;
    vz = v_tz_now;

    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;



    Eigen::VectorXd z(6,1);

//    cout<<"连续滤波所得:x速度":

    z<<BestArmor.tx,BestArmor.ty,BestArmor.tz,v_tx_now*1000,v_ty_now*1000,v_tz_now*1000;

    //得到状态转移矩阵
    Eigen::MatrixXd F_in(6,6);
    F_in<<1.0, 0.0, 0.0, t/1000, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, t/1000, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, t/1000,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0,0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

//预测上一最佳状态值
    KF_forecast.Prediction(F_in);

//更新状态量
    KF_forecast.update(z,F_in);

//    if(KF_forecast.x_(3)*v_tx_now<0){
//        KF_forecast.x_(3) = v_tx_now*1000;
//    }else if(fabs(KF_forecast.x_(3))>fabs(v_tx_now*1000)){
//        KF_forecast.x_(3) = (KF_forecast.x_(3)*1/3 + v_tx_now*1000*2/3);
//    }
//    if(KF_forecast.x_(4)*v_ty_now<0){
//        KF_forecast.x_(4) = v_ty_now*1000;
//    }else if(fabs(KF_forecast.x_(4))>fabs(v_tx_now*1000)){
//        KF_forecast.x_(4) = (KF_forecast.x_(4)*1/3 + v_ty_now*1000*2/3);
//    }
//    if(KF_forecast.x_(5)*v_tz_now<0){
//        KF_forecast.x_(5) = v_tz_now*1000;
//    }else if(fabs(KF_forecast.x_(5))>fabs(v_tx_now*1000)){
//        KF_forecast.x_(5) = (KF_forecast.x_(5)*1/3 + v_tz_now*1000*2/3);
//    }


    //传参
    p_tx_old = KF_forecast.x_(0);
    p_ty_old = KF_forecast.x_(1);
    p_tz_old = KF_forecast.x_(2);





    old_carDatas = carDatas;

}

void AngleSolver::BufferSetFilter(RM_ArmorDate & BestArmor,CarData CarDatas){
    //Send为发送角度，run为一定时间t内云台转动角度量
    //add为t内目标估计运动量，add = v*t
    //新的发送量Send_now = Send + add - run
    if(!KF_forecast.is_set_x){
        BestArmor.pitch = SendPitch - (CarDatas.pitch - old_carDatas.pitch);
        BestArmor.yaw = SendYaw - (CarDatas.yaw - old_carDatas.yaw);
//        old_carDatas = CarDatas;
        return ;
    }
    double t = CarDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    BestArmor.tx =old_objectP.x + KF_forecast.x_(3)/1000*t;
    BestArmor.ty =old_objectP.y + KF_forecast.x_(4)/1000*t;
    BestArmor.tz =old_objectP.z + KF_forecast.x_(5)/1000*t;
//    BestArmor.ty += KF_forecast.x_(4)/1000*t;
//    BestArmor.tz += KF_forecast.x_(5)/1000*t;
//    BestArmor.pitch = SendPitch + KF_forecast.x_(2)/1000*t - (CarDatas.pitch - old_carDatas.pitch);
//    BestArmor.yaw = SendYaw + KF_forecast.x_(3)/1000*t - (CarDatas.yaw - old_carDatas.yaw);
    Angle_t at = ComputeShootTime(BestArmor.tx,BestArmor.ty,BestArmor.tz,CarDatas);
//    SendPitch = BestArmor.pitch;
//    SendYaw = BestArmor.yaw;
//    old_objectP.x = BestArmor.tx;
//    old_objectP.y = BestArmor.ty;
//    old_objectP.z = BestArmor.tz;
    BestArmor.pitch = at.pitch   - (CarDatas.pitch - old_carDatas.pitch);
    BestArmor.yaw = at.yaw - (CarDatas.yaw - old_carDatas.yaw);
    BestArmor.tx = old_objectP.x;
    BestArmor.ty = old_objectP.y;
    BestArmor.tz = old_objectP.z;
//    BestArmor.pitch = 0;
//    old_carDatas = CarDatas;
}

/**
 * @brief AngleSolver::ComShootPosition     陀螺击打点计算
 * @param BestArmor                                             当前目标
 * @param tuoluoData                                            陀螺数据
 * @param nowTime                                                          当前的所得图片对应时间
 *@param oldTime                                                          上一图片对应所得时间
 */
void AngleSolver::ComTuoluoShootPosition(RM_ArmorDate &BestArmor, TuoluoData tuoluoData,double nowTime,double oldTime,CarData carDatas){
//    //利用角度解算中ComputeShootTime函数计算击打时间
//    tuoluoShootTime = AngleSolver::ComputeShootTime(BestArmor.tx,BestArmor.ty,BestArmor.tz);
//    if(tuoluoShootTime.t == 0) return;              //击打时间计算无解
    //滤波过滤角速度
    if(!KF_tuoluo.is_set_x){            //第一次得到角度量
        //设置状态向量和状态转移矩阵
        Eigen::MatrixXd F(2,2);
        F<<1, 0,
                1, 1;
        //设置状态向量    [角速度,角速度变换量],角速度变换量默认初值0
        Eigen::VectorXd x(2,1);
        x<<tuoluoData.spinSpeed/(nowTime - oldTime), 0;
        //卡尔曼滤波赋初值
        KF_tuoluo.set_x(x,F);
    }else{
        //连续滤波
        Eigen::MatrixXd F(2,2);
        F<<1, 0,
                (nowTime - oldTime), 1;
        KF_tuoluo.Prediction(F);
        Eigen::VectorXd z(1,1);
        z<<tuoluoData.spinSpeed/(nowTime - oldTime);
        KF_tuoluo.update(z,F);
    }

    //计算击打时间,当前未添加迭代,原理上应添加迭代,因为当前所计算的位置并不是最终预测位置
    Angle_t tuoluoShootTime = ComputeShootTime(BestArmor.tx,BestArmor.ty,BestArmor.tz,carDatas);
    if(tuoluoShootTime.t == 0) return;             //计算无解
    //得到角度预测量      弹道时间+本帧运行时间+发弹延迟
    double bufferAngle = KF_tuoluo.x_(0)*(tuoluoShootTime.t + Recive.getClock() - nowTime + SHOOT_DELAY_TIME);

    //计算击打位置,选择击打模块
    if(fabs(bufferAngle)>=360){
        bufferAngle = bufferAngle - ((int)bufferAngle/360)*360;
    }
    float goalAngle =  tuoluoData.angle + bufferAngle;
    //防止所得角度绝对值超过360度
    if(goalAngle>=360||goalAngle<=-360){
        goalAngle = goalAngle - ((int)bufferAngle/360)*360;
    }
    //防止所得角度绝对值大于180
    if(goalAngle>=180){
        goalAngle = -360 + goalAngle;
    }else if(goalAngle<=-180){
        goalAngle = 360 + goalAngle;
    }
    float minAngle =goalAngle;
    //如果计算到的角度绝对值小于20度也就是大概在中心就击打
    if(fabs(minAngle)>20){
        //根据当前目标角度往右计算每块装甲对应角度,也就是俯视逆时针看
        for(int i = 0;i<3;i++){
            float newAngle = goalAngle + 90;
            //防止所得角度大于180
            if(newAngle>180){
                newAngle = -180 + (newAngle - 180);
            }
            if(fabs(newAngle)>fabs(minAngle)){
                minAngle = newAngle;
            }
            if(fabs(minAngle)<20)break;
        }
    }

    //根据角度,陀螺圆心及陀螺半径计算预测点
    if(minAngle>50||minAngle<-50){
        cout<<"当前陀螺预测角度计算有误!!!"<<endl;
        cout<<"当前所计算角速度:"<<KF_tuoluo.x_(0)<<"  未滤波的原速度:"<<tuoluoData.spinSpeed<<endl;
        cout<<"当前所计算的击打角度:"<<minAngle<<endl;
        return;
    }
    //计算所得结果符合一定范围内
    //无论左右均可,前提x坐标系是右大做小,与像素坐标系相同
    BestArmor.tz = tuoluoData.center.y - tuoluoData.R*cos(minAngle*PI/180);
    BestArmor.tx = tuoluoData.center.x + tuoluoData.R*sin(minAngle*PI/180);

    //计算击打抬升量
    Angle_t tuoluoShoot = ComputeShootTime(BestArmor.tx,BestArmor.ty,BestArmor.tz,carDatas);
    BestArmor.pitch = tuoluoShoot.pitch;
    BestArmor.yaw = tuoluoShoot.yaw;
}

/**
 * @brief AngleSolver::AbsToRelative        由云台当前角度和绝对坐标得到相对角度,然后通过参数返回
 * @param tx
 * @param ty
 * @param tz
 * @param Carpitch  车云台pitch角度
 * @param Caryaw    车云台yaw角度
 * @param pitch         返回识别相对pitch角度
 * @param yaw           返回识别相对yaw角度
 * @remark 车pitch和yaw角度均为0~180和0~-180
 */
void AngleSolver::AbsToRelative(const float tx, const float ty, const float tz,  float Carpitch,  float Caryaw, float &pitch, float &yaw){
    float distance = sqrt(pow(tx,2)+pow(ty,2)+pow(tz,2));           //距离
    float absPitch = asin(fabs(ty)/distance)*180/PI;                                       //pitch轴角度绝对值

    //确定pitch角度
    if(tz>=0){
        //前
        if(ty>=0){
            //上方
            pitch = absPitch;
        }else{
            //下方
            pitch = -absPitch;
        }
    }else{
        //后
        if(ty>=0){
            //上方
            pitch = 180 - absPitch;
        }else{
            //下方
            pitch = -180 + absPitch;
        }
    }

    //确定yaw角度
    if(tx>=0){
        //原点右侧
        if(tz == 0&&tx>0)       //特殊求tan分母为0的情况
            yaw = 90;

        if(tz>0){                       //一般情况
            yaw = atan(tx/tz)*180/PI;
        }else{
            yaw = 180 - atan(tx/(-tz))*180/PI;
        }

    }else{
        //原点左侧
        if(tz == 0)       //特殊求tan分母为0的情况
            yaw = -90;

        if(tz>0){                       //一般情况
            yaw = atan(tx/tz)*180/PI;
        }else{
            yaw = -180 - atan(tx/(-tz))*180/PI;
        }

    }

//    Carpitch = Carpitch*180/PI;
//    Caryaw = Caryaw*180/PI;
    Caryaw = Caryaw - (int)Caryaw/360*360;
    if(Caryaw>180)
        Caryaw -= 360;
    if(Caryaw<-180)
        Caryaw += 360;
    //由绝对角度得到相对角度
    pitch -= Carpitch;
    yaw -= Caryaw;

}

void AngleSolver::SetSecOrderKF(Mat Src,ArmorDate &BestArmor, Point3f Poisition, float v_x, float v_y, float v_z, float t){
    if(!KF_SecOrder.is_set_x){
        FirstSetSecOrderKF(Poisition,v_x,v_y,v_z);
        return ;
    }

    ContinueSetSecOrderKF(Src,BestArmor.Armor,Poisition,v_x,v_y,v_z,t);
}


void AngleSolver::FirstSetSecOrderKF( Point3f Poisition,float v_x, float v_y, float v_z){
    cout<<"第一次二阶滤波"<<endl;
    KF_SecOrder.is_set_x = true;
    Eigen::VectorXd x(6,1);
    x<<Poisition.x,Poisition.y,Poisition.z,v_x,v_y,v_z;
    KF_SecOrder.set_x(x);
}

void AngleSolver::ContinueSetSecOrderKF(Mat Src,RM_ArmorDate &BestArmor, Point3f Poisition, float v_x, float v_y, float v_z, float t){
    Eigen::VectorXd z(6,1);
    z<<Poisition.x,Poisition.y,Poisition.z,v_x,v_y,v_z;
    //得到状态转移矩阵
    Eigen::MatrixXd F_in(6,6);
    F_in<<1.0, 0.0, 0.0, t, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, t, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, t,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0,0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    KF_SecOrder.Prediction(F_in);
    KF_SecOrder.update(z,F_in);
    float distance = pow(KF_SecOrder.x_(0) - old_Poisition.x,2)+pow(KF_SecOrder.x_(1) - old_Poisition.y,2)+pow(KF_SecOrder.x_(2) - old_Poisition.z,2);
    if(distance<MIN_FORCE_DISTANCE_CHA){
        cout<<"符合预测范围标准"<<endl;
        BestArmor.tx = KF_SecOrder.x_(0);
        BestArmor.ty = KF_SecOrder.x_(1);
        BestArmor.tz = KF_SecOrder.x_(2);
//        circle(Src,Point(70,70),10,Scalar(0,255,0),-1,4);
    }else{
        cout<<"不符合预测范围标准"<<endl;
        circle(Src,Point(70,70),10,Scalar(0,0,255),-1,4);
//        cout<<"distance:"<<distance<<endl;
//        cout<<"上级位置:"<<old_Poisition.x<<" "<<old_Poisition.y<<" "<<old_Poisition.z<<endl;
//        cout<<"下级位置:"<<KF_SecOrder.x_(0)<<" "<<KF_SecOrder.x_(1)<<" "<<KF_SecOrder.x_(2)<<endl;
    }

    //状态更新
    old_Poisition = Point3f(KF_SecOrder.x_(0),KF_SecOrder.x_(1),KF_SecOrder.x_(2));
}

/**
 * @brief AngleSolver::setAngleForceParam           角度滤波参数初始化
 * @param KF                                                                        卡尔曼滤波对象
 */
void AngleSolver::setAngleForceParam(KF_two &KF){
    //状态协方差矩阵附初值

    Eigen::MatrixXd P_in = Eigen::MatrixXd(4,4);
    P_in << 1.0, 0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0,
                    0.0,0.0,1.0,0.0,
                   0.0,0.0,0.0,1.0;
    KF_AngleForeCast.P = P_in;

     //过程噪声矩阵附初值
    Eigen::MatrixXd Q_in(4,4);
    Q_in<<1.0, 0.0,0.0,0.0,
                   0.0,1.0,0.0,0.0,
                   0.0,0.0,1.0,0.0,
                   0.0,0.0,0.0,1.0;
    KF_AngleForeCast.Q = Q_in;

      //测量矩阵附初值
    Eigen::MatrixXd H_in(4,4);
    H_in<<1.0, 0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0,
                    0.0,0.0,1.0,0.0,
                    0.0,0.0,0.0,1.0;
    KF_AngleForeCast.H = H_in;

    //测量噪声矩阵附初值
    Eigen::MatrixXd R_in(4,4);
    R_in<<700.0, 0.0,0.0,0.0,
                   0.0,700.0,0.0,0.0,
                   0.0,0.0,100.0,0.0,
                   0.0,0.0,0.0,100.0;
    KF_AngleForeCast.R = R_in;

    Eigen::MatrixXd F_in(4,4);
    F_in<<1.0, 0.0,1.0,0.0,
            0.0,1.0,0.0,1.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;
    KF_AngleForeCast.F = F_in;
}

/**
 * @brief AngleSolver::angle_FirstFind              首次发现目标,初始化
 * @param BestArmor
 * @param carDatas
 */
void AngleSolver::angle_FirstFind(RM_ArmorDate BestArmor, CarData carDatas){
    //卡尔曼对象陀螺以初始化状态
    KF_AngleForeCast.is_set_x = false;
    //状态保留
    old_Pitch = BestArmor.pitch;
    old_Yaw = BestArmor.yaw;
    old_carDatas = carDatas;
}

/**
 * @brief AngleSolver::angle_FirstSetFilter
 * @param BestArmor
 * @param carDatas
 */
void AngleSolver::angle_FirstSetFilter(RM_ArmorDate &BestArmor, CarData carDatas){
    //得到车转动角度
    float runPitch = carDatas.pitch - old_carDatas.pitch;
    float runYaw = carDatas.yaw - old_carDatas.yaw;

    //得到绝对角度
    realCarPitch = -runPitch + BestArmor.pitch - old_Pitch;
    realCarYaw = -runYaw + BestArmor.yaw - old_Yaw;

    //得到速度
    float pitchSpeed = realCarPitch/(carDatas.BeginToNowTime - old_carDatas.BeginToNowTime)*1000;
    float yawSpeed = realCarYaw/(carDatas.BeginToNowTime - old_carDatas.BeginToNowTime)*1000;

    //速度记录
    for(int i = 0;i<RECORD_RUN_TIMES;i++){
        PitchRunSpeed[i] = pitchSpeed;
        YawRunSpeed[i] = yawSpeed;
    }
    RecornRunTimes = RECORD_RUN_TIMES - 1;

    //为状态向量赋初值
    Eigen::VectorXd x(4,1);
    x<<realCarPitch,realCarYaw,pitchSpeed,yawSpeed;
    KF_AngleForeCast.set_x(x);

    //状态保留
    old_Pitch = BestArmor.pitch;
    old_Yaw = BestArmor.yaw;
    old_carDatas = carDatas;

}

/**
 * @brief AngleSolver::angle_ContinueSetFilter          连续滤波
 * @param BestArmor
 * @param carDatas
 * @remark 根据当前卡尔曼滤波状态向量和两个收数所得时间差得到预测值,利用预测值和本帧的测量值更新状态向量
 */
void AngleSolver::angle_ContinueSetFilter(RM_ArmorDate &BestArmor, CarData carDatas){
//    if(IsHaveBuffer){
//        old_carDatas = buffer_old_carDatas;
//    }
    //得到经历时间
    float t = carDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    //得到状态矩阵
    Eigen::MatrixXd F = Eigen::MatrixXd(4,4);
    F<<1.0,0.0,t/1000.0,0.0,
            0.0,1.0,0.0,t/1000.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;
    KF_AngleForeCast.Prediction(F);


//得到本帧真实值
    //得到车转动角度
    float runPitch = carDatas.pitch - old_carDatas.pitch;
    float runYaw = carDatas.yaw - old_carDatas.yaw;

    //得到帧内移动量
    float pitchSpeed = runPitch + BestArmor.pitch - old_Pitch;
    float yawSpeed = runYaw + BestArmor.yaw - old_Yaw;


    //将速度存入速度记录数组,由记录值得到近几帧的估计值,减小波动影响
    PitchRunSpeed[(++RecornRunTimes)%RECORD_RUN_TIMES] = pitchSpeed;
    YawRunSpeed[(++RecornRunTimes)%RECORD_RUN_TIMES] = yawSpeed;

    //计算加权平均数
    float Weight = 1;
    float WeightSum = 0;
    float PitchSpeedSum = 0;
    float YawSpeedSum = 0;
    for(int i = 0;i<RECORD_RUN_TIMES;i++){
        PitchSpeedSum += PitchRunSpeed[(RecornRunTimes - i)%RECORD_RUN_TIMES] * Weight;
        YawSpeedSum += YawRunSpeed[(RecornRunTimes - i)%RECORD_RUN_TIMES] * Weight;
        WeightSum += Weight;
        Weight *= 0.5;
    }

    pitchSpeed = PitchSpeedSum/WeightSum;
    yawSpeed = YawSpeedSum/WeightSum;

    //得到绝对角度
    realCarPitch += pitchSpeed;
    realCarYaw += yawSpeed;
    pitchSpeed =(pitchSpeed/(carDatas.BeginToNowTime - old_carDatas.BeginToNowTime))*1000;
    yawSpeed =(yawSpeed/(carDatas.BeginToNowTime - old_carDatas.BeginToNowTime))*1000;
    cout<<"绝对角度pitch:"<<realCarPitch<<" yaw:"<<realCarYaw<<endl;
    cout<<"pitch速度:"<<pitchSpeed<<" yaw速度:"<<yawSpeed<<endl;

    //计算预测量

    //得到真实值向量
    Eigen::VectorXd z(4,1);
    z<<realCarPitch,realCarYaw,pitchSpeed,yawSpeed;

    KF_AngleForeCast.update(z,F);

    buffer_old_carDatas = old_carDatas;
    old_carDatas = carDatas;
    old_Pitch = BestArmor.pitch;
    old_Yaw = BestArmor.yaw;
}
