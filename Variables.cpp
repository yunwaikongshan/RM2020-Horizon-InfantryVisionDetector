/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  main.cpp
  *Author:  解佳朋
  *Version: 1.3.3.200312_RC
  *Date:  2020.03.12
  *Description: 主要相关联全局变量以及函数定义
**********************************************************************************/
#include<include/Variables.h>

//定义识别装甲颜色
ArmorColor armor_color = RED;

//线程锁
mutex reciveRes;                        //线程读取资源锁

 //参数读取目录
string paramFileName = "/home/xiejiapeng/xie_jia_peng/RM2020-Horizon-InfantryVisionDetector/RM2020-Horizon-InfantryVisionDetector/DebugParam.xml";

//读取数据
CarData getStm32;

string VideoPath = "/home/xiejiapeng/视频/红色步兵.avi";          //视频调试路径
string BuffVideoPath = "/home/xiejiapeng/视频/大神符7.avi";      //大符视频路径
string USBDevicPath = "/dev/video0";                                                   //长焦设备路径
string BuffDevicPath = "/dev/video1";                                                   //大符底盘相机设备路径

//阈值
int GrayValue = 130;
int RGrayWeightValue = 97;
int BGrayWeightValue = 97;


//hsv
int RLowH ;
int RHighH;

int RLowS ;
int RHighS ;

int RLowV ;
int RHighV ;

int BLowH ;
int BHighH;

int BLowS;
int BHighS;

int BLowV ;
int BHighV;

int V_ts;

//相机曝光
int UsbExpTime ;                    //usb相机曝光值
int UsbExpTimeValue;        //控制usb相机曝光值动态变化
int UsbAspTime;                    //usb相机光圈
int UsbAspTimeValue;         //控制usb相机光圈动态变化
int GXExpTime ;               //大恒相机曝光值
int GXExpTimeValue;         //控制大恒相机曝光值动态变化
int GXGain ;                             //大恒相机增益
int GXGainValue;                //控制大恒相机曝光增益值动态变化



//装甲符号
int ShootArmorNumber = -2;

//平面三点拟合得到圆心
Point2f GetCircle(Point2f point_one, Point2f point_two,Point2f point_three){
    //x1，y1为box_buff中心点，x2，y2为last_box_buff中心点
    float x1 = point_one.x;
    float x2 = point_two.x;
    float x3 = point_three.x;
    float y1 = point_one.y;
    float y2 = point_two.y;
    float y3 = point_three.y;

    //******************利用三点求圆心************
    float A1 = 2*(x2-x1);
    float B1 = 2*(y2-y1);
    float C1 = x2*x2+y2*y2-x1*x1-y1*y1;
    float A2 = 2*(x3-x2);
    float B2 = 2*(y3-y2);
    float C2 = x3*x3+y3*y3-x2*x2-y2*y2;
    float X = ((C1*B2)-(C2*B1))/((A1*B2)-(A2*B1));
    float Y = ((A1*C2)-(A2*C1))/((A1*B2)-(A2*B1));
    return Point2f(X,Y);
    //**********************************************
}

//空间3点得圆心
//来自:https://blog.csdn.net/weixin_30576827/article/details/97556185?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.channel_param
Point3f solveCenterPointOfCircle(vector<Point3f> pt)
{
    double a1, b1, c1, d1;
    double a2, b2, c2, d2;
    double a3, b3, c3, d3;

    double x1 = pt[0].x, y1 = pt[0].y, z1 = pt[0].z;
    double x2 = pt[1].x, y2 = pt[1].y, z2 = pt[1].z;
    double x3 = pt[2].x, y3 = pt[2].y, z3 = pt[2].z;

    a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
    b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
    c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
    d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

    a2 = 2 * (x2 - x1);
    b2 = 2 * (y2 - y1);
    c2 = 2 * (z2 - z1);
    d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

    a3 = 2 * (x3 - x1);
    b3 = 2 * (y3 - y1);
    c3 = 2 * (z3 - z1);
    d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

    double x = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
        /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    double y =  (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
        /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
    double z = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
        /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

    cout<<"计算所得圆心:x:"<<x<<" y:"<<y<<"   z:"<<z<<endl;
    return Point3f(x,y,z);
}


//根据余弦定理，得到夹角,a为顶角
/**
 * @brief GetAngle
 * @param a
 * @param b
 * @param c
 * @param src
 * @return
 * @remark 2维空间,a为顶角
 */
double GetAngle(Point2f a,Point2f b,Point2f c){
    double DistanceA = GetDistance(b,c);
    double DistanceB = GetDistance(a,c);
    double DistanceC = GetDistance(b,a);

    double angle_cos = (DistanceB*DistanceB+DistanceC*DistanceC-DistanceA*DistanceA)/(2*DistanceB*DistanceC);
    cout<<angle_cos<<endl;
    double angle = acos(angle_cos);
    return 180*angle/PI;
}
//三维空间,a为顶角
double GetAngle(Point3f a,Point3f b,Point3f c){
    double DistanceA = GetDistance(b,c);
    double DistanceB = GetDistance(a,c);
    double DistanceC = GetDistance(b,a);

    double angle_cos = (DistanceB*DistanceB+DistanceC*DistanceC-DistanceA*DistanceA)/(2*DistanceB*DistanceC);
    cout<<angle_cos<<endl;
    double angle = acos(angle_cos);
    return 180*angle/PI;
}

