/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  main.cpp
  *Author:  解佳朋
  *Version: 1.3.1.200128_RC
  *Date:  2020.01.28
  *Description: 主要相关联全局变量以及函数定义,具体内部已注明
**********************************************************************************/
#ifndef VARIABLES_H
#define VARIABLES_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include<math.h>
#include <fstream>
#include<mutex>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml.hpp>
//#include <X11/Xlib.h>
//#include<RemoteController.h>

using namespace cv;
using namespace std;

#define PI 3.14159
//#define DEBUG                            //是否进入调试模式
#define IMSHOW                                       //是否显示最终图像
//#define IMAGE_DRAWING                 //是否开启画面绘制


extern string VideoPath;         //视频调试路径
extern string BuffVideoPath;      //大符视频路径
extern string USBDevicPath;                                                   //长焦设备路径
extern string BuffDevicPath ;                                                   //大符底盘相机设备路径


//射击模式
enum pattern{
    FirstFind,        //首次识别
    Shoot,              //连续射击
    stop,               //非连续
    buffering        //缓冲
};

//相机类型
typedef enum{
    UsingLong,
    UsingGuang
} Camera  ;

//相机类型
typedef enum{
    BUFF_FIRST_SHOOT,
    BUFF_CONTINUE_SHOOT,
    BUFF_BUFFER_SHOOT
} BuffStatus  ;


//读取图像模式
typedef enum{
    USB,                       //usb相机
    DAHENG,             //大恒相机
    BUFF                    //大符
} ImageGetMode;

//收数结构体
struct CarData{
    float pitch = 0;
    float yaw = 0;
    float ShootSpeed = 16;
    bool IsBuffMode = false;
    double BeginToNowTime = 0;
};

//采入图像
struct ImageDate{
    Mat SrcImage;
    ImageGetMode mode;
    CarData ReciveStm32;
};

//装甲颜色
enum ArmorColor{
    RED,
    BLUE
};

//陀螺状态
enum TuoluoStatus{
    STILL,              //静止
    SWITCH,        //切换
    RUN               //同块运动
};

//陀螺转动方向
enum TuoluoRunDirection{
    LEFT,
    RIGHT
};


//灯条结构体
struct led{
    RotatedRect box;                                    //拟合椭圆
    Point2f led_on = Point2f(0,0);          //灯条上点坐标
    Point2f led_down = Point2f(0,0);    //灯条下点坐标
};

//待击打装甲信息
struct RM_ArmorDate{
    float tx = 0;
    float ty = 0;
    float tz = 0;
    float pitch = 0;
    float yaw = 0;
    bool IsSmall = true;
    bool IsTuoluo = false;          //是否为陀螺状态
    double get_waith = 0;               //对应陀螺宽度
    double distance = 0;
    led leds[2];
    Point2f point[4];
    bool IsShooting = true;
};
//待击打大符信息
struct RM_BuffData{
    float tx = 0;
    float ty = 0;
    float tz = 0;
    float pitch = 0;
    float yaw = 0;
    RotatedRect box;
    Point2f point[4];
};

//识别函数返回装甲信息
struct ArmorDate{
    RM_ArmorDate Armor;
    pattern status = stop;
};

//陀螺数据
struct TuoluoData{
    bool isTuoluo = false;                                          //是否为陀螺
    float R;                                                                      //半径
    float angle;                                                            //当前角度
    Point2f center;
    TuoluoStatus status;                                         //陀螺当前的状态
    TuoluoRunDirection runDirection;            //转动方向
    float spinSpeed;                                                //角速度
};

//击打缓冲计算返回
typedef struct{
    float pitch;
    float yaw;
    float t;                                                                //击打弹道时间
    float angle;
} Angle_t;

//定义识别装甲颜色
extern ArmorColor armor_color ;


//阈值
extern int GrayValue;
extern int GGrayValue;
extern int BinaryValue;
extern int RBGrayValue;
extern int RGrayWeightValue ;
extern int BGrayWeightValue ;

//hsv
extern int RLowH;
extern int RHighH;

extern int RLowS;
extern int RHighS ;

extern int RLowV;
extern int RHighV ;

extern int BLowH ;
extern int BHighH ;

extern int BLowS ;
extern int BHighS ;

extern int BLowV ;
extern int BHighV ;

extern int V_ts ;


//相机曝光;
extern int UsbExpTime ;
extern int UsbAspTime ;
extern int GXExpTime ;
extern int GXGain;

extern int UsbExpTimeValue ;
extern int UsbAspTimeValue ;
extern int GXExpTimeValue ;
extern int GXGainValue;

//装甲符号
extern int ShootArmorNumber;

 double GetDistance(Point2f a,Point2f b);
 double GetDistance(Point3f a,Point3f b);                        //空间两点距离
Point2f GetCircle(Point2f point_one, Point2f point_two,Point2f point_three);
Point3f solveCenterPointOfCircle(vector<Point3f> pt);           //空间3点得圆心
Point2f getArmorCenter(Point2f p[4]);               //得到装甲中心
 float getArmorAngle(Point2f p[4]);
double GetAngle(Point2f a,Point2f b,Point2f c);         //平面三点得a的夹角
double GetAngle(Point3f a,Point3f b,Point3f c);         //空间三点得a的夹角

//线程锁
extern mutex reciveRes;                        //线程读取资源锁

//读取数据
extern CarData getStm32;

//参数读取路径
extern string paramFileName;


//得到两点距离
 inline double GetDistance(Point2f a,Point2f b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

//得到空间两点距离
 inline double GetDistance(Point3f a,Point3f b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2)+pow(a.z - b.z,2));
}

//得到装甲中心
 inline Point2f getArmorCenter(Point2f p[4]){
    return Point2f((p[0].x+p[1].x+p[2].x+p[3].x)/4,(p[0].y+p[1].y+p[2].y+p[3].y)/4);
}

 //得到装甲倾斜角度
 inline float getArmorAngle(Point2f p[4]){
     return((atan2(p[0].y - p[1].y,p[1].x - p[0].x)+atan2(p[3].y - p[2].y,p[2].x - p[3].x))/2);
 }


#endif // VARIABLES_H
