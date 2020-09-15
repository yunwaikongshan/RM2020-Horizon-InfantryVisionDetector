/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  ImageProcess.cpp
  *Author:  解佳朋
  *Version: 1.3.2.2007021_RC
  *Date:  2020.07.21
  *Description: 图片获取与处理控制
  *Function List:
     1.ImageProducter 获取图片
     2.ImageConsumer  消费图片
     3.DrawImage  绘制反馈结果
  * Others:
        利用多线程异步处理图像的获取与处理,提高对cpu的利用率,提高程序运行速度.图像的获取与处理控制采用生产者与消费者算法.
**********************************************************************************/
#include<include/ImageProcess.h>
#include<include/RemoteController.h>
#include <sys/time.h>
//模式设置
//#define CAMERA_DEBUG                                        //是否使用相机调试
//#define USING_BUFF_DETECTOR                     //是否开启大符识别
//#define USING_VIDEO_BUFF                              //大符录像调试
//#define USING_VIDEO                                        //是否使用视频调试
//#define USING_CAMERA_DAHENG              //是否使用大恒相机
#define USING_CAMERA_USB                           //是否使用usb相机


#define BUFFER 5                                            //线程间相机采集最高超过处理的帧数
ImageDate Image[BUFFER];
static volatile unsigned int proIdx = 0;    //生产id
static volatile unsigned int consIdx = 0; //消费id
double run_time = 0;        //记录时间
bool IsSwitch = false;      //是否切换摄像头
timeval s_start,s_end;


//大符状态
BuffStatus BuffShootStatus;


ImageProcess::ImageProcess(){
    proIdx = 0;
    consIdx = 0;
    s_start.tv_sec = 0;
    s_end.tv_usec = 0;
}

void ImageProcess::ImageProducter(){
//使用录像调试
#ifdef USING_VIDEO

#ifdef USING_BUFF_DETECTOR      //使用打符模式
    VideoCapture cap(BuffVideoPath);
#else       //不使用打符模式
    VideoCapture cap(VideoPath);
#endif

#endif //USING_VIDEO

//不使用录像调试
#ifndef USING_VIDEO
//使用普通usb相机
#ifdef USING_CAMERA_USB
    myown::VideoCapture cap(USBDevicPath.data(),3);
    cap.setExpousureTime(false, UsbExpTime);
//    cap.setApertureNumber(UsbAspTime - 64);           //调节光圈
    cap.setVideoFormat(640, 480, 1);
    cap.info();
    cap.startStream();
#endif //USING_CAMERA_USB

#ifdef USING_CAMERA_DAHENG
     DaHengCamera CameraTure;
     //链接设备
     CameraTure.StartDevice(1);
     //设置分辨率
     CameraTure.SetResolution(2,2);
     //控制设备开采
     CameraTure.SetStreamOn();
     //设置曝光值
     CameraTure.SetExposureTime(GXExpTime);
     //设置曝光增益
     CameraTure.SetGAIN(3,GXGain);
     //关闭自动白平衡
     CameraTure.Set_BALANCE_AUTO(0);

#endif //USING_CAMERA_DAHENG


//是否添加大符识别功能
#ifdef SHOOT_BUFF_FUNCTION
     myown::VideoCapture cap_buff(BuffDevicPath.data(),3);
     cap_buff.setExpousureTime(false, UsbExpTime);
     cap_buff.setVideoFormat(640, 480, 1);
     cap_buff.info();
     cap_buff.startStream();
#endif//SHOOT_BUFF_FUNCTION

#endif //no USING_VIDEO


     //生产循环
     while(1){
         while(proIdx - consIdx >= BUFFER);
        //挂资源锁
         reciveRes.lock();
        bool isShootBuff = getStm32.IsBuffMode;             //控制是否当前为打符模式
        //解锁
        reciveRes.unlock();
         cout<<"进入生产!"<<endl;
         ImageDate Src;
         //使用收数资源
         //挂资源锁
         reciveRes.lock();
         Src.ReciveStm32.BeginToNowTime = getStm32.BeginToNowTime;
         Src.ReciveStm32.pitch = getStm32.pitch;
         Src.ReciveStm32.yaw = getStm32.yaw;
         Src.ReciveStm32.ShootSpeed = getStm32.ShootSpeed;
//         cout<<"得到收数时间"<<getStm32.BeginToNowTime<<endl;
//         cout<<"收数pitch:"<<Src.ReciveStm32 .pitch<<"    yaw:"<<Src.ReciveStm32.yaw<<"   射速:"<<getStm32.ShootSpeed<<endl<<endl;
         //解锁
         reciveRes.unlock();

#ifdef USING_VIDEO
         //使用视频模式
#ifdef USING_CAMERA_USB
         Src.mode = USB;
#else
         Src.mode = DAHENG;
#endif
         cap>>Src.SrcImage;
#else //USING_VIDEO
         //不使用视频模式
#ifdef USING_CAMERA_DAHENG
         //使用大恒相机模式
         if(!isShootBuff){
             CameraTure.GetMat(Src.SrcImage);
             if(Src.SrcImage.cols == 0){
                 cout<<"丢帧!"<<endl;
                 continue;
             }
             Src.mode = DAHENG;
         }
#ifdef DEBUG
         //开启debug
         if(GXExpTime!=GXExpTimeValue){
             GXExpTimeValue = GXExpTime;
             CameraTure.SetExposureTime(GXExpTimeValue);
         }
         if(GXGainValue!=GXGain){
             GXGainValue = GXGain;
             CameraTure.SetGAIN(3,GXGainValue);
         }
#endif //CAMERA_DEBUG
#else //USING_CAMERA_DAHENG
         //不使用大恒相机模式
         Src.mode = USB;
         cap>>Src.SrcImage;
#ifdef DEBUG
         //开启debug
     if(UsbExpTime!=UsbExpTimeValue){
         UsbExpTimeValue = UsbExpTime;
         cap.setExpousureTime(false,UsbExpTime);
     }
     if(UsbAspTime!=UsbAspTimeValue){
         UsbAspTimeValue = UsbAspTime;
         cap.setApertureNumber(UsbAspTime - 64);
     }
#endif //CAMERA_DEBUG
#endif // noUSING_CAMERA_DAHENG
//#endif // no USING_CAMERA_DOUBLE
#endif //no USING_VIDEO

#ifdef USING_BUFF_DETECTOR
#ifdef USING_VIDEO_BUFF
     cap>>Src.SrcImage;
     if(!IsSwitch){
         IsSwitch = true;
         BuffShootStatus = BUFF_FIRST_SHOOT;
     }else{
         BuffShootStatus = BUFF_CONTINUE_SHOOT;
     }
     Src.mode = BUFF;
#else
     cap>>Src.SrcImage;
#endif//USING_VIDEO_BUFF
#endif//USING_BUFF_DETECTOR

     Image[proIdx%BUFFER] = Src;


     proIdx++;
     cout<<"生产完成:"<<proIdx-1<<endl;
     }
}

void ImageProcess::ImageConsumer(){
    while(1){
        while(consIdx>=proIdx);
        double time0=static_cast<double>(getTickCount());
        int ImageIndex = proIdx-1;              //处理最新图像
        ArmorDate BestArmorDate;

        if(Image[(ImageIndex)%BUFFER].mode != BUFF){                            //自瞄
            if(Image[(ImageIndex)%BUFFER].mode == USB||Image[(ImageIndex)%BUFFER].mode == DAHENG){      //相机
                LongFindArmor FindArmor;
                BestArmorDate = FindArmor.GetArmorDate(Image[(ImageIndex)%BUFFER].SrcImage);
            }else{          //视频
                LongFindArmor FindArmor;
                BestArmorDate = FindArmor.GetArmorDate(Image[(ImageIndex)%BUFFER].SrcImage);
            }
            //角度解算
            //用于位置滤波
            if(BestArmorDate.status == buffering){      //缓冲状态
                AngleSolver angleSolve;
                angleSolve.BufferSetFilter(BestArmorDate.Armor,Image[(ImageIndex)%BUFFER].ReciveStm32);
            }else if(BestArmorDate.status == FirstFind||BestArmorDate.status == Shoot){         //非缓冲状态
                AngleSolver angleSolve;
                    angleSolve.GetArmorAngle(Image[(ImageIndex)%BUFFER].SrcImage,BestArmorDate,UsingLong,Image[(ImageIndex)%BUFFER].ReciveStm32);
            }
        }else{                                  //能量机关
            FindBuff BuffDetector;
            RM_BuffData* Buffs = BuffDetector.BuffModeSwitch(Image[(ImageIndex)%BUFFER].SrcImage);
            if(Buffs!=( RM_BuffData*)-1){
                BuffAngleSolver BuffAngleData;
                BuffAngleData.GetBuffShootAngle(Buffs,BUFF_CONTINUE_SHOOT,Image[(ImageIndex)%BUFFER].ReciveStm32);

                for(int i = 0;i<4;i++){
                    line(Image[(ImageIndex)%BUFFER].SrcImage,Buffs[3].point[i%4],Buffs[3].point[(i+1)%4],Scalar(0,0,255),4,4);
                }
                char test[100];
                sprintf(test, "tz:%0.4f", Buffs[3].tz);
                cv::putText(Image[(ImageIndex)%BUFFER].SrcImage, test, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
                sprintf(test, "tx:%0.4f", Buffs[3].tx);
                cv::putText(Image[(ImageIndex)%BUFFER].SrcImage, test, cv::Point(Image[(ImageIndex)%BUFFER].SrcImage.cols/3, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
                sprintf(test, "ty:%0.4f", Buffs[3].ty);
                cv::putText(Image[(ImageIndex)%BUFFER].SrcImage, test, cv::Point(2*Image[(ImageIndex)%BUFFER].SrcImage.cols/3, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
                sprintf(test, "yaw:%0.4f ", Buffs[3].yaw);
                cv::putText(Image[(ImageIndex)%BUFFER].SrcImage, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
                sprintf(test, "pitch:%0.4f ", Buffs[3].pitch);
                cv::putText(Image[(ImageIndex)%BUFFER].SrcImage, test, cv::Point(Image[(ImageIndex)%BUFFER].SrcImage.cols/3, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
            }else{


            }

        }

        Send.ArmorToData(BestArmorDate,UsingLong);

#ifdef IMSHOW

        DrawImage(Image[(ImageIndex)%BUFFER].SrcImage,BestArmorDate);
        char test[100];
        sprintf(test, "getPitch:%0.4f   getYaw:%0.4f", Image[(ImageIndex)%BUFFER].ReciveStm32.pitch,Image[(ImageIndex)%BUFFER].ReciveStm32.yaw);
//        cv::putText(Image[(ImageIndex)%BUFFER].SrcImage, test, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
        imshow("绘制",Image[(ImageIndex)%BUFFER].SrcImage);
        char key = waitKey(1);
        if(key == 's'){
            //将当前参数存入xml
            SaveData();
        }
#endif

        run_time=(double)cvGetTickCount()-run_time;
        run_time = run_time/(cvGetTickFrequency()*1000);                                //t2为一帧的运行时间,也是单位时间
       printf("帧内 %gms\n",run_time);



       time0=((double)getTickCount()-time0)/getTickFrequency();
      printf("执行时间 %gms\n",time0*1000);



       run_time = (double)cvGetTickCount();            //计时,记录两次运行到此处的间隔
       consIdx = ImageIndex+1;
       cout<<"消费完成"<<endl;
//       cout<<"consIdx:"<<consIdx<<" proIdx:"<<proIdx<<endl;
    }
}

void ImageProcess::DrawImage(Mat Src,ArmorDate BestArmorDate){
    if(BestArmorDate.status == stop)
        return;
    if(BestArmorDate.status == FirstFind){
        for(int i = 0;i<4;i++){
            line(Src,BestArmorDate.Armor.point[i%4],BestArmorDate.Armor.point[(i+1)%4],Scalar(255,0,255),4,4);
        }
    }else if(BestArmorDate.status == Shoot){
        for(int i = 0;i<4;i++){
            line(Src,BestArmorDate.Armor.point[i%4],BestArmorDate.Armor.point[(i+1)%4],Scalar(0,255,0),4,4);
        }
    }else if(BestArmorDate.status == buffering){
        for(int i = 0;i<4;i++){
            line(Src,BestArmorDate.Armor.point[i%4],BestArmorDate.Armor.point[(i+1)%4],Scalar(0,0,255),4,4);
        }
    }

    //绘制
    char test[100];
    sprintf(test, "tz:%0.4f", BestArmorDate.Armor.tz);
    cv::putText(Src, test, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    sprintf(test, "tx:%0.4f", BestArmorDate.Armor.tx);
    cv::putText(Src, test, cv::Point(Src.cols/3, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    sprintf(test, "ty:%0.4f", BestArmorDate.Armor.ty);
    cv::putText(Src, test, cv::Point(2*Src.cols/3, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    sprintf(test, "yaw:%0.4f ", BestArmorDate.Armor.yaw);
    cv::putText(Src, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    sprintf(test, "pitch:%0.4f ", BestArmorDate.Armor.pitch);
    cv::putText(Src, test, cv::Point(Src.cols/3, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);

}

void ImageProcess::SaveData(){
    //参数读入
    FileStorage fs_param(paramFileName,FileStorage::WRITE);
    if(!fs_param.isOpened()){
        cout<<"参数文件打开失败!"<<endl;
        exit(1);
    }
    //参数传入
    //传入rgb参数
    fs_param<<"RGB_BinaryValue"<<"{"<<"GrayValue"<<GrayValue
                                                                                <<"RGrayValue"<<RGrayWeightValue
                                                                               <<"BGrayValue"<<BGrayWeightValue<<"}";

    //传入hsv参数
    fs_param<<"HSV_BinaryValue"<<"{"<<"RLowH"<<RLowH
                                                                                <<"RLowS"<<RLowS
                                                                                <<"RLowV"<<RLowV
                                                                                <<"RHighH"<<RHighH
                                                                                <<"RHighS"<<RHighS
                                                                                <<"RHighV"<<RHighV

                                                                                <<"BLowH"<<BLowH
                                                                                <<"BLowS"<<BLowS
                                                                                <<"BLowV"<<BLowV
                                                                                <<"BHighH"<<BHighH
                                                                                <<"BHighS"<<BHighS
                                                                                <<"BHighV"<<BHighV
                                                                                <<"V_ts"<<V_ts<<"}";

    //传入曝光参数
    fs_param<<"ExpTime"<<"{"<<"UsbExpTime"<<UsbExpTime
                                                            <<"UsbAspTime"<<UsbAspTime
                                                            <<"DahengExpTime"<<GXExpTime
                                                            <<"DahengGain"<<GXGain<<"}";

    fs_param.release();
}



