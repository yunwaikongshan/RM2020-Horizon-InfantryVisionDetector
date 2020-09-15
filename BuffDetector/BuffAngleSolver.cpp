/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  BuffAngleSolver.cpp
  *Author:  解佳朋
  *Version: 1.3.1.200517_RC
  *Date:  2020.05.17
  *Description: 大符识别角度计算,与位置预测和打击计算
  *Function List:
     1.BuffModeSwitch   大符识别接口,返回用于圆心计算的三个位置和当前待打击位置的数组
     2.PreDelBuff   大符识别预处理
     3.FindBestBuff    寻找并存入所有的大符目标
     4.GetShootBuff    寻找待击打的大符目标
  *Others:打击算法介绍:利用空间三点拟合找到圆心坐标,积累圆心坐标,计算平均值,得到基本准确的圆心位置.
                                               得到圆心利用两条面上向量叉乘,得到大符面的法向量,进而得到偏转角,再利用坐标转
                                               换将大符坐标转换到与底盘相机平行.利用圆心和当前识别坐标,计算每次圆心角,得到
                                               旋转角速度,利用角速度完成预测,利用抛物线规律计算打击时间得到打击位置,再次迭
                                              代计算打击位置,得到在误差允许范围内的打击点,完成打击.
**********************************************************************************/
#include<include/BuffAngleSolver.h>
#define SHOOT_DELAY_TIME 120                //发弹延迟
#define MIN_SAVE_CIRCLR_CENTER_NUMBER   3          //保留最小的圆心数量
#define MAX_SAVE_CIRCLR_CENTER_NUMBER   75          //保留最大的圆心数量
#define BUFF_R 156
#define RUN_FRAME_BLANK_NUMBER 7                                          //检测记录帧间隔
#define MIN_RUN_ANGLE 3                                                     //最小运动角度,判断当前大小符状态
Point3f CircleCenters[MAX_SAVE_CIRCLR_CENTER_NUMBER];           //圆心保留数组
int CircleCenterNumber = 0;                                                     //当前存留的圆心数量
int SaveVectorNumber = 0;                                                        //当前存留向量数
float old_BuffCentralAngle = 0;                                               //上一帧圆心角
int FrameBlankNumber = 0;
static Point3f old_center;
Point3f old_Vector;
Point3f Vectors[MAX_SAVE_CIRCLR_CENTER_NUMBER];
float ChassisToBuff_Pitch[MAX_SAVE_CIRCLR_CENTER_NUMBER];
float ChassisToBuff_Yaw[MAX_SAVE_CIRCLR_CENTER_NUMBER];
/**********************角速度滤波**************************/
KF_two KF_BuffAngleSpeed;
float old_AngleSpeed;
static CarData old_carDatas;
bool isHaveSetBuffKF = false;


BuffAngleSolver::BuffAngleSolver(){
    ChassisToPtz_x = 0;
    ChassisToPtz_y = 0;
    ChassisToPtz_z = 0;
    ChassisToPtz_Pitch = 0*PI/180;
    ChassisToPtz_Yaw = 0*PI/180;
    ChassisToPtz_Roll = 0*PI/180;

    BuffWidth = 23;
    BuffHeight = 12.7;
    if(!isHaveSetBuffKF){
        isHaveSetBuffKF = true;
        //状态协方差矩阵附初值

        Eigen::MatrixXd P_in = Eigen::MatrixXd(2,2);
        P_in << 1.0, 0.0,
                        0.0,1.0;
        KF_BuffAngleSpeed.P = P_in;

         //过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(2,2);
        Q_in<<1.0, 0.0,
                        0.0,1e-1;
        KF_BuffAngleSpeed.Q = Q_in;

          //测量矩阵附初值
        Eigen::MatrixXd H_in(1,2);
        H_in<<1.0, 0.0;
        KF_BuffAngleSpeed.H = H_in;

        //测量噪声矩阵附初值
        Eigen::MatrixXd R_in(1,1);
        R_in<<10;
        KF_BuffAngleSpeed.R = R_in;

        Eigen::MatrixXd F_in(2,2);
        F_in<<1.0,1.0,
                0.0,1.0;
        KF_BuffAngleSpeed.F = F_in;
    }
}

/**
 * @brief BuffAngleSolver::GetBuffrAngle    角度解算得到空间坐标
 * @param BestArmor
 */
void BuffAngleSolver::GetBuffrAngle( RM_BuffData &BestArmor){
    std::vector<cv::Point2f>point2D;
    std::vector<cv::Point3f>point3D;

    GetPoint2D(BestArmor,point2D);                                                                                                      //矩形转换为2d坐标
    GetPoint3D(BestArmor,point3D);                                                                                                     //矩形转换为3d坐标
    CountAngleXY(point2D,point3D,BestArmor);                                                                                         //解决pnp问题

    ChassisToPtz(BestArmor);                                                                                                                 //底盘转云台
}

/**
 * @brief GetBuffForceResult            //得到大符预测点
 * @param BestArmor
 */
void BuffAngleSolver::GetBuffShootAngle(RM_BuffData* BestArmor,BuffStatus BuffShootStatus,CarData carDatas){
    if(BuffShootStatus == BUFF_FIRST_SHOOT){
        //各个值初始化
        CircleCenterNumber = 0;
        old_BuffCentralAngle = 0;
        KF_BuffAngleSpeed.is_set_x = false;
    }
    //得到各个的坐标
    GetBuffrAngle(BestArmor[0]);
    GetBuffrAngle(BestArmor[1]);
    GetBuffrAngle(BestArmor[2]);
    GetBuffrAngle(BestArmor[3]);

    //得到大符圆心
    Point3f BuffCenter = GetBuffCenter(BestArmor);

    cout<<"所计算圆心:"<<BuffCenter<<endl;
//    CircleCenters[CircleCenterNumber%MAX_SAVE_CIRCLR_CENTER_NUMBER] = BuffCenter;
//    CircleCenterNumber++;
    if(CircleCenterNumber<MIN_SAVE_CIRCLR_CENTER_NUMBER){
        //当前记录圆心数量不足
        cout<<"所记录圆心数量不足"<<endl;
        return;
    }
    //计算平均圆心
//    BuffCenter = GetAveBuffCenter();
//    cout<<"所计算圆心:"<<BuffCenter<<endl;

    //得到法向量,和旋转角度
    float r_Pitch,r_Yaw;
    if(FrameBlankNumber==0){
        old_Vector = Point3f(CircleCenters[4].x - BuffCenter.x,CircleCenters[4].y - BuffCenter.y,CircleCenters[4].z - BuffCenter.z);
        Vectors[SaveVectorNumber%MAX_SAVE_CIRCLR_CENTER_NUMBER] = Point3f(CircleCenters[4].x - BuffCenter.x,CircleCenters[4].y - BuffCenter.y,CircleCenters[4].z - BuffCenter.z);
        SaveVectorNumber++;
    }
    if(FrameBlankNumber==RUN_FRAME_BLANK_NUMBER){
        Point3f new_Vector = Point3f(CircleCenters[4].x - BuffCenter.x,CircleCenters[4].y - BuffCenter.y,CircleCenters[4].z - BuffCenter.z);
        FrameBlankNumber = 0;
        int i = 0;
        float sum_Pitch = 0;
        float sum_Yaw = 0;
        for(;i<SaveVectorNumber&&i<MAX_SAVE_CIRCLR_CENTER_NUMBER;i++){
            Point3f NormalVector;
            NormalVector.x = new_Vector.y*Vectors[i].z - Vectors[i].y*new_Vector.z;
            NormalVector.y = new_Vector.z*Vectors[i].x - Vectors[i].y*new_Vector.x;
            NormalVector.z = new_Vector.x*Vectors[i].y - Vectors[i].x*new_Vector.y;
            //计算角度
            float pitch = atan2(NormalVector.y,NormalVector.z);
            float yaw = atan2(NormalVector.x,NormalVector.z);
            sum_Pitch += pitch;
            sum_Yaw += yaw;
        }
        r_Pitch = sum_Pitch/i;
        r_Yaw = sum_Yaw/i;
    }
    //坐标旋转至与相机平行
    ShootAdjust(BestArmor[0].tx,BestArmor[0].ty,BestArmor[0].tz,r_Pitch,r_Yaw);
    ShootAdjust(BestArmor[1].tx,BestArmor[1].ty,BestArmor[1].tz,r_Pitch,r_Yaw);
    ShootAdjust(BestArmor[2].tx,BestArmor[2].ty,BestArmor[2].tz,r_Pitch,r_Yaw);
    ShootAdjust(BestArmor[3].tx,BestArmor[3].ty,BestArmor[3].tz,r_Pitch,r_Yaw);
    ShootAdjust(BuffCenter.x,BuffCenter.y,BuffCenter.z,r_Pitch,r_Yaw);

    //利用标尺点(下边界点),计算对应圆心角度
    Point3f LowerBoundaryPoint = Point3f(BuffCenter.x,BuffCenter.y - BUFF_R,BuffCenter.z);       //下边界点,存在一定误差,考虑采用圆空间方程计算
    Point3f LefterBoundaryPoint =  Point3f(BuffCenter.x - BUFF_R,BuffCenter.y,BuffCenter.z);        //左边界点,存在一定误差
    float BuffCentralAngle = GetBuffCentralAngle(CircleCenters[4],BuffCenter,LowerBoundaryPoint,LefterBoundaryPoint);
    //得到速度
    if(BuffShootStatus == BUFF_FIRST_SHOOT){
        //速度初始化
        old_BuffCentralAngle = BuffCentralAngle;
        KF_BuffAngleSpeed.is_set_x = false;
        return;
    }
    float AngleSpeed = BuffCentralAngle - old_BuffCentralAngle;
    BuffAngleSpeedFilter(AngleSpeed,KF_BuffAngleSpeed,carDatas);
    //得到预测点
    //击打点迭代计算
    float erro = 100;
    Angle_t ShootBuffer;
    Point3f now_Poistion = Point3f(BestArmor[3].tx,BestArmor[3].ty,BestArmor[3].tz);            //未旋转位置
    ShootAdjust(now_Poistion.x,now_Poistion.y,now_Poistion.z,-r_Pitch,-r_Yaw);
    ShootBuffer = ComputeBuffShootTime(now_Poistion.x,now_Poistion.y,now_Poistion.z,carDatas);
    if(ShootBuffer.t == 0){
        ShootBuffer.t = 20;
    }
    float ForceTime = ShootBuffer.t;
    float time = SHOOT_DELAY_TIME+ForceTime+Recive.getClock() - carDatas.BeginToNowTime;
    float AngleBuff = AngleSpeed*time;
    //得到预测位置(坐标系旋转后)
    Point3f ShootPoistion = GetShootPoistion(AngleBuff,BuffCenter,BUFF_R);
    now_Poistion = Point3f(ShootPoistion.x,ShootPoistion.y,ShootPoistion.z);
    while(erro>5){
        //利用新的预测位置计算预测时间和抬升量再次计算
        ShootBuffer = ComputeBuffShootTime(now_Poistion.x,now_Poistion.y,now_Poistion.z,carDatas);
        //得到新打击点
        float now_time = SHOOT_DELAY_TIME+ForceTime+Recive.getClock() - carDatas.BeginToNowTime;
        float now_AngleBuff = AngleSpeed*now_time;
        ShootPoistion = GetShootPoistion(AngleBuff,BuffCenter,BUFF_R);
        now_Poistion = ShootPoistion;
         ShootAdjust(now_Poistion.x,now_Poistion.y,now_Poistion.z,-r_Pitch,-r_Yaw);
        erro = now_AngleBuff - AngleBuff;
        AngleBuff = now_AngleBuff;
        cout<<"打符误差计算大小:"<<erro<<endl;
    }

    ShootBuffer = ComputeBuffShootTime(now_Poistion.x,now_Poistion.y,now_Poistion.z,carDatas);
    BestArmor[3].pitch = ShootBuffer.pitch;
    BestArmor[3].yaw = ShootBuffer.yaw;
}

/**
 * @brief BuffAngleSolver::GetShootPoistion
 * @param angle     当前位置相对圆弧度角
 * @param center
 * @param r
 * @return
 * @remark 传入角度必须为弧度
 */
Point3f BuffAngleSolver::GetShootPoistion(float angle, Point3f center, float r){
    return Point3f(center.x + r*sin(angle),center.y + r*cos(angle),center.z);
}

/**
 * @brief BuffAngleSolver::BuffAngleSpeedFilter         角速度滤波
 * @param AngleSpeed
 * @param Filter
 * @param carDatas
 */
void BuffAngleSolver::BuffAngleSpeedFilter(float & AngleSpeed, KF_two Filter,CarData carDatas){
    //角速度滤波
    if(!Filter.is_set_x){
        //第一次设置滤波
        Eigen::VectorXd x(2,1);
        x<<AngleSpeed,0;
        Filter.set_x(x);
    }else{
        //连续滤波
        float a_AngleSpeed = (AngleSpeed - old_AngleSpeed)/(carDatas.BeginToNowTime - old_carDatas.BeginToNowTime);
        Eigen::VectorXd x(2,1);
        x<<AngleSpeed,a_AngleSpeed;
        Eigen::MatrixXd F(2,2);
        F<<1 , (carDatas.BeginToNowTime - old_carDatas.BeginToNowTime),
                0 , 1;
        Filter.Prediction(F);
        Filter.update(x,F);
    }
    old_AngleSpeed = AngleSpeed;
    AngleSpeed = Filter.x_(0);

}

/**
 * @brief BuffAngleSolver::GetBuffCentralAngle  得到当前位置对应旋转圆心角
 * @param ObjectPoistion
 * @param CircleCenter
 * @param LowerBoundaryPoint    下边界点
 * @param LefterBoundaryPoint       左边界点
 * @return 顺时针从0到360圆心角
 * @remark 利用边界点和余弦定理计算夹角
 */
float BuffAngleSolver::GetBuffCentralAngle(Point3f ObjectPoistion, Point3f CircleCenter, Point3f LowerBoundaryPoint, Point3f LefterBoundaryPoint){
    float angle = 0;
    if(fabs(ObjectPoistion.x - CircleCenter.x)<BUFF_R/2){
        //距离左边界点更远
        angle = GetAngle(CircleCenter,ObjectPoistion,LefterBoundaryPoint);
        if(ObjectPoistion.y - CircleCenter.y>0){
            //第1,2象限
            if(angle>90){
                angle -= 90;
            }else{
                angle += 270;
            }
        }else{
            //第3,4象限
            angle = 270 - angle;
        }

    }else{
        //距离下边界点更远
        angle = GetAngle(CircleCenter,ObjectPoistion,LowerBoundaryPoint);
        if(ObjectPoistion.x - CircleCenter.x>0){
            //第1,4象限
            angle = 180 - angle;
        }else{
            //第2,3象限
            angle = 180 + angle;
        }

    }
    return angle;
}

/**
 * @brief BuffAngleSolver::GetBuffCenter    得到大符运动圆心
 * @param BestArmor 传入的RM_BuffData数组,前三为待计算点,最后为当前待打击目标
 * @return 三维中心点
 * @remark 利用空间三点拟合的圆心坐标,使用RM_BuffData数组前三个元素的中心点估算
 */
Point3f BuffAngleSolver::GetBuffCenter(RM_BuffData *BestArmor){
    //三点拟合圆心
    vector<Point3f> pt;
    pt.push_back(Point3f(BestArmor[0].tx,BestArmor[0].ty,BestArmor[0].tz));
    pt.push_back(Point3f(BestArmor[1].tx,BestArmor[1].ty,BestArmor[1].tz));
    pt.push_back(Point3f(BestArmor[2].tx,BestArmor[2].ty,BestArmor[2].tz));
    Point3f CircleCenter = solveCenterPointOfCircle(pt);
    CircleCenters[CircleCenterNumber%MAX_SAVE_CIRCLR_CENTER_NUMBER] = CircleCenter;
    CircleCenterNumber++;
    //多次拟合取圆心平均值
    float x_sum = 0;
    float y_sum = 0;
    float z_sum = 0;
    int i = 0;
    for(;i<CircleCenterNumber&&i<MAX_SAVE_CIRCLR_CENTER_NUMBER;i++){
        x_sum += CircleCenters[i].x;
        y_sum += CircleCenters[i].y;
        z_sum += CircleCenters[i].z;
    }
    CircleCenter.x = x_sum/i;
    CircleCenter.y = y_sum/i;
    CircleCenter.z = z_sum/i;
    return CircleCenter;
}

/**
 * @brief BuffAngleSolver::GetAveBuffCenter
 * @return
 * @remark 根据当前所记录圆心坐标数量得到圆心估计值
 *                      圆心坐标存储于CircleCenters,CircleCenterNumber记录当前所记录的圆心数量
 */
Point3f BuffAngleSolver::GetAveBuffCenter(){
    float x_sum = 0;
    float y_sum = 0;
    float z_sum = 0;
    int i = 0;
    for(;i<CircleCenterNumber&&i<MAX_SAVE_CIRCLR_CENTER_NUMBER;i++){
        x_sum += CircleCenters[i].x;
        y_sum += CircleCenters[i].y;
        z_sum += CircleCenters[i].z;
    }
    return Point3f(x_sum/i,y_sum/i,z_sum/i);
}

Point3f BuffAngleSolver::GetNormalVector(Point3f new_vector,Point3f old_vector){
    Point3f NormalVector;
    NormalVector.x = new_vector.y*old_vector.z - old_vector.y*new_vector.z;
    NormalVector.y = new_vector.z*old_vector.x - old_vector.y*new_vector.x;
    NormalVector.z = new_vector.x*old_vector.y - old_vector.x*new_vector.y;
    return NormalVector;
}

/**
 * @brief BuffAngleSolver::ShootAdjust              依据计算所得法向量得旋转角,进行坐标旋转得到正对坐标
 * @param x
 * @param y
 * @param z
 * @param pitch
 * @param yaw
 * @return
 */
float BuffAngleSolver::ShootAdjust(float &x, float &y, float &z, float pitch, float yaw){
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(pitch) , 0 , -sin(pitch),
                        0 , 1 , 0,
                        sin(pitch) , 0 , cos(pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(yaw) , sin(yaw) , 0,
                     -sin(yaw) , cos(yaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);          //按z，x，y传入，即变化对应到左手坐标系
    original<<z,x,y;
    original = r_Pitch*original;
    original = r_Yaw*original;
    x = original(1);
    y = original(2);
    z = original(0);
}

/**
 * @brief BuffAngleSolver::GetPoint2D       存入2d点
 * @param BestArmor
 * @param point2D
 */
void BuffAngleSolver::GetPoint2D( RM_BuffData & BestArmor,std::vector<cv::Point2f>&point2D){

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
void BuffAngleSolver::GetPoint3D( RM_BuffData & BestArmor,std::vector<cv::Point3f>&point3D)
{
    float fHalfX=0;
    float fHalfY=0;

    fHalfX=BuffWidth/2.0;
    fHalfY=BuffHeight/2.0;

    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY,0.0));
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY,0.0));
    point3D.push_back(cv::Point3f(fHalfX,fHalfY,0.0));
    point3D.push_back(cv::Point3f(-fHalfX,fHalfY,0.0));

}

//pnp转换
void BuffAngleSolver::CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, RM_BuffData & BestArmor){
    cv::Mat rvecs=cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs=cv::Mat::zeros(3,1,CV_64FC1);

//     cv::solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs,false,SOLVEPNP_UPNP);
    solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs);

    double tx = tvecs.ptr<double>(0)[0];
    double ty = -tvecs.ptr<double>(0)[1];
    double tz = tvecs.ptr<double>(0)[2];

    BestArmor.tx = tx;
    BestArmor.ty = ty;
    BestArmor.tz = tz;


    BestArmor.pitch = atan2( BestArmor.ty, BestArmor.tz)*180/PI;
     BestArmor.yaw = atan2( BestArmor.tx, BestArmor.tz)*180/PI;
}

/**
 * @brief BuffAngleSolver::ChassisToPtz             底盘旋转至云台
 * @param BestArmor
 */
void BuffAngleSolver::ChassisToPtz(RM_BuffData &BestArmor){
    //绕roll轴旋转，即为绕z轴旋转
    Eigen::MatrixXd r_Roll(3,3);
    r_Roll<<1 , 0 , 0,
                    0 , cos(ChassisToPtz_Roll) , sin(ChassisToPtz_Roll),
                    0 , -sin(ChassisToPtz_Roll) , cos(ChassisToPtz_Roll);
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(ChassisToPtz_Pitch) , 0 , -sin(ChassisToPtz_Pitch),
                        0 , 1 , 0,
                        sin(ChassisToPtz_Pitch) , 0 , cos(ChassisToPtz_Pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(ChassisToPtz_Yaw) , sin(ChassisToPtz_Yaw) , 0,
                     -sin(ChassisToPtz_Yaw) , cos(ChassisToPtz_Yaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);          //按z，x，y传入，即变化对应到左手坐标系
    original<<BestArmor.tz,BestArmor.tx,BestArmor.ty;

    //平移变换
    Eigen::VectorXd translation(3,1);
//    cout<<"未偏移前:x:"<<change[1]<<"   y:"<<change[2]<<"   z:"<<change[0]<<endl;
    translation<<ChassisToPtz_z,ChassisToPtz_x,ChassisToPtz_y;
    original = original + translation;

    Eigen::VectorXd change(3,1);
    //旋转变换
    change =  r_Roll * original;
    change = r_Pitch*change;
    change = r_Yaw*change;

    BestArmor.tx = change(1);
    BestArmor.ty = change(2);
    BestArmor.tz = change(0);
}

Angle_t BuffAngleSolver::ComputeBuffShootTime(float tx, float ty, float distance,struct CarData CarDatas){
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
    cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<tz<<"yaw"<<ShootBuff.yaw<<endl;
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
    return ShootBuff;
}

