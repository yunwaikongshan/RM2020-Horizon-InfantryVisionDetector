/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  main.cpp
  *Author:  //作者
  *Version: 1.3.1.200128_RC
  *Date:  2020.01.28
  *Description:  相机读取,图像处理,收发数线程初始化
**********************************************************************************/
#include <iostream>
#include<include/Variables.h>
#include<include/RemoteController.h>
#include<include/ImageProcess.h>
#include <thread>

/***
 * When I wrote this, only God and I understood what I was doing
 * Now, God only knows
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                    O\ = /O
 *                ____/`---'\____
 *              .   ' \\| |// `.
 *               / \\||| : |||// \
 *             / _||||| -:- |||||- \
 *               | | \\\ - /// | |
 *             | \_| ''\---/'' | |
 *              \ .-\__ `-` ___/-. /
 *           ___`. .' /--.--\ `. . __
 *        ."" '< `.___\_<|>_/___.' >'"".
 *       | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \ \ `-. \_ __\ /__ _/ .-` / /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 *.............................................
 *          佛祖保佑             永无BUG
 *
 **/
void getDate();

int main()
{
    //获得外部xml文件中数据
    getDate();
    //创建轨迹条窗口
#ifdef DEBUG
    cv::namedWindow("曝光");
    namedWindow("BGR");
    namedWindow("R-HSV");
    namedWindow("B-HSV");
    //曝光
    createTrackbar("usb相机曝光值", "曝光", &UsbExpTime, 200);
    createTrackbar("usb相机光圈值", "曝光", &UsbAspTime, 200);
    createTrackbar("大恒相机曝光值", "曝光", &GXExpTime, 200);
    createTrackbar("大恒相机曝光增益值", "曝光", &GXGain, 200);
    //rgb
    createTrackbar("灰度二值化阈值", "BGR", &GrayValue, 255);
    createTrackbar("R权重", "BGR", &RGrayWeightValue, 100);
    createTrackbar("B权重", "BGR", &BGrayWeightValue, 100);
    //R-hsv
    cvCreateTrackbar("RLowH", "R-HSV", &RLowH, 360); //Hue (0 - 179)
    cvCreateTrackbar("RHighH", "R-HSV", &RHighH, 360);

    cvCreateTrackbar("RLowS", "R-HSV", &RLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("RHighS", "R-HSV", &RHighS, 255);

    cvCreateTrackbar("RLowV", "R-HSV", &RLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("RHighV", "R-HSV", &RHighV, 255);
    cvCreateTrackbar("V", "R-HSV", &V_ts, 255);

    //B-hsv
    cvCreateTrackbar("BLowH", "B-HSV", &RLowH, 360); //Hue (0 - 179)
    cvCreateTrackbar("BHighH", "B-HSV", &RHighH, 360);

    cvCreateTrackbar("BLowS", "B-HSV", &RLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("BHighS", "B-HSV", &RHighS, 255);

    cvCreateTrackbar("BLowV", "B-HSV", &RLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("BHighV", "B-HSV", &RHighV, 255);

#endif
    //线程创建
    ImageProcess process;
    std::thread t1(&ImageProcess::ImageProducter,process);                  //创造图像生产线程
    std::thread t2(&ImageProcess::ImageConsumer,process);                //创造图像消费线程
    std::thread t3(&RemoteController::paraReceiver, Recive);                //创造收数线程
    std::thread t4(&RemoteController::paraGetCar, Send);                      //创造发数线程
    t3.join();
    t4.join();
    t2.join();
    t1.join();
    return 0;
}

/**
 * @brief getDate        从文件中得到数据
 */
void getDate(){

    //参数读入
    FileStorage fs_param(paramFileName,FileStorage::READ);
    if(!fs_param.isOpened()){
        cout<<"参数文件打开失败!"<<endl;
        exit(1);
    }
    //参数传入
    //传入rgb参数
    fs_param["RGB_BinaryValue"]["GrayValue"]>>GrayValue;
    fs_param["RGB_BinaryValue"]["RGrayValue"]>>RGrayWeightValue;
    fs_param["RGB_BinaryValue"]["BGrayValue"]>>BGrayWeightValue;
    //传入hsv参数
    fs_param["HSV_BinaryValue"]["RLowH"]>>RLowH;
    fs_param["HSV_BinaryValue"]["RLowS"]>>RLowS;
    fs_param["HSV_BinaryValue"]["RLowV"]>>RLowV;
    fs_param["HSV_BinaryValue"]["RHighH"]>>RHighH;
    fs_param["HSV_BinaryValue"]["RHighS"]>>RHighS;
    fs_param["HSV_BinaryValue"]["RHighV"]>>RHighV;

    fs_param["HSV_BinaryValue"]["BLowH"]>>BLowH;
    fs_param["HSV_BinaryValue"]["BLowS"]>>BLowS;
    fs_param["HSV_BinaryValue"]["BLowV"]>>BLowV;
    fs_param["HSV_BinaryValue"]["BHighH"]>>BHighH;
    fs_param["HSV_BinaryValue"]["BHighS"]>>BHighS;
    fs_param["HSV_BinaryValue"]["BHighV"]>>BHighV;
    fs_param["HSV_BinaryValue"]["V_ts"]>>V_ts;

    //传入曝光参数
    fs_param["ExpTime"]["UsbExpTime"]>>UsbExpTime;
    fs_param["ExpTime"]["DahengExpTime"]>>GXExpTime;
    fs_param["ExpTime"]["DahengGain"]>>GXGain;
    fs_param["ExpTime"]["UsbAspTime"]>>UsbAspTime;

    UsbExpTimeValue = UsbExpTime;
    GXExpTimeValue = GXExpTime;
    GXGainValue = GXGain;
    UsbAspTimeValue = UsbAspTime;

    fs_param.release();
}
