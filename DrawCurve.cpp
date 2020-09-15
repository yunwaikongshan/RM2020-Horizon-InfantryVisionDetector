#include<include/Variables.h>
#include<include/DrawCurve.h>
#define SIN_POINT_NUM 400

//保留数据
float SavePoint[SIN_POINT_NUM];         //保存点位
float SecSavePoint[SIN_POINT_NUM];         //保存点位
int Times = 0;

void DrawCurve::ClearSaveData(){
    Times = 0;
}

void DrawCurve::InsertData(float Data){
    //创建用于绘制的深蓝色背景图像
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    image.setTo(cv::Scalar(100, 0, 0));

    SavePoint[Times%SIN_POINT_NUM] = Data;

    float maxData = 0;
    for(int i = 0;i<=Times&&i<SIN_POINT_NUM;i++){
        if(fabs(SavePoint[i])>maxData){
            maxData = fabs(SavePoint[i]);
        }
    }


    //计算倍率
    float bei = 10;
    if(10*maxData>image.rows/2){
        bei = (image.rows/2 - 10)/maxData;
    }
    if(10*maxData<150&&maxData!=0){
        bei = 150.0/maxData;
    }
    //输入拟合点
    std::vector<cv::Point> points;
    //存入当前记录位置的后续单位,靠前的位置
     int t = 0;
    for(int i = (Times + 1)%SIN_POINT_NUM;i<=Times&&i<SIN_POINT_NUM;i++){
        points.push_back(Point((float)image.cols/SIN_POINT_NUM*t ,image.rows/2+bei*SavePoint[i]));
        t++;
    }
    //绘制折线
//        cv::polylines(image, points, false, cv::Scalar(0, 255, 0), 1, 8, 0);
    //存入之前的元素
    for(int i  = 0;i<=Times %SIN_POINT_NUM;i++){
        points.push_back(Point( (float)image.cols/SIN_POINT_NUM*t ,image.rows/2+bei*SavePoint[i]));
        t++;
    }
    cv::polylines(image, points, false, cv::Scalar(0, 255, 0), 1, 8, 0);
    imshow("波形图",image);
    Times++;
}

void DrawCurve::InsertData(float Data1,float Data2,string s1,string s2){
    //创建用于绘制的深蓝色背景图像
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    image.setTo(cv::Scalar(100, 0, 0));
    cv::Mat image2 = cv::Mat::zeros(480, 640, CV_8UC3);
    image2.setTo(cv::Scalar(100, 0, 0));

    SavePoint[Times%SIN_POINT_NUM] = Data1;
    SecSavePoint[Times%SIN_POINT_NUM] = Data2;

    float maxData = 0;
    for(int i = 0;i<=Times&&i<SIN_POINT_NUM;i++){
        if(fabs(SavePoint[i])>maxData){
            maxData = fabs(SavePoint[i]);
        }
        if(fabs(SecSavePoint[i])>maxData){
            maxData = fabs(SecSavePoint[i]);
        }
    }

    //计算倍率
    float bei = 10;
    if(10*maxData>image.rows/2){
        bei = (image.rows/2 - 10)/maxData;
    }
    if(10*maxData<150&&maxData!=0){
        bei = 150.0/maxData;
    }

//    bei = 1;
    //输入拟合点,绘制第一条曲线
    std::vector<cv::Point> points;
    //存入当前记录位置的后续单位,靠前的位置
     int t = 0;
    for(int i = (Times + 1)%SIN_POINT_NUM;i<=Times&&i<SIN_POINT_NUM;i++){
        points.push_back(Point((float)image.cols/SIN_POINT_NUM*t ,image.rows/2-bei*SavePoint[i]));
        t++;
    }
    //绘制折线
//        cv::polylines(image, points, false, cv::Scalar(0, 255, 0), 1, 8, 0);
    //存入之前的元素
    for(int i  = 0;i<=Times %SIN_POINT_NUM;i++){
        points.push_back(Point( (float)image.cols/SIN_POINT_NUM*t ,image.rows/2-bei*SavePoint[i]));
        t++;
    }
    cv::polylines(image, points, false, cv::Scalar(0, 255, 0), 1, 8, 0);

    //输入拟合点,绘制第二条曲线
    std::vector<cv::Point> SecPoints;
    //存入当前记录位置的后续单位,靠前的位置
     t = 0;
    for(int i = (Times + 1)%SIN_POINT_NUM;i<=Times&&i<SIN_POINT_NUM;i++){
        SecPoints.push_back(Point((float)image.cols/SIN_POINT_NUM*t ,image.rows/2-bei*SecSavePoint[i]));
//        circle(image, Point((float)image.cols/SIN_POINT_NUM*t ,image.rows/2-bei*SecSavePoint[i]), 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        t++;
    }
    //存入之前的元素
    for(int i  = 0;i<=Times %SIN_POINT_NUM;i++){
        SecPoints.push_back(Point( (float)image.cols/SIN_POINT_NUM*t ,image.rows/2-bei*SecSavePoint[i]));
//        circle(image, Point((float)image.cols/SIN_POINT_NUM*t ,image.rows/2-bei*SecSavePoint[i]), 2, cv::Scalar(0, 0, 255), 2, 8, 0);
        t++;
    }
    cv::polylines(image, SecPoints, false, cv::Scalar(50, 80, 255), 1, 8, 0);
    rectangle(image,Rect(30,10,20,20),Scalar(0, 255, 0),-1,4);
    rectangle(image,Rect(30,40,20,20),Scalar(50, 80, 255),-1,4);

    cv::putText(image, s1, cv::Point(60, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 2);
    cv::putText(image, s2, cv::Point(60, 55), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 2);

    imshow("波形图",image);
//    imshow("波形图2",image2);
    char key = waitKey(1);
    if(key == 'p'){
        imwrite("/home/xiejiapeng/图片/波形图/滤波后pitch波形图.jpg",image);
    }
    Times++;
}
