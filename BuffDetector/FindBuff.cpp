/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  FindBuff.cpp
  *Author:  解佳朋
  *Version: 1.3.1.200517_RC
  *Date:  2020.05.17
  *Description: 大符识别
  *Function List:
     1.BuffModeSwitch   大符识别接口
     2.PreDelBuff   大符识别预处理
     3.FindBestBuff    寻找并存入所有的大符目标
     4.GetShootBuff    寻找待击打的大符目标
**********************************************************************************/
#include<include/FindBuff.h>
/********************大符调试********************************/
#define PI 3.14159
#define BUFF_W_RATIO_H 1.9              //最终大符内轮廓长宽比
#define BUFF_AREA_RATIO 710.0       //最终大符内轮廓面积与图片面积像素比
#define BUFFER_BUFF_BOX 4             //大符内轮廓存储缓冲数量
#define BUFF_CIRCLE_BOX    3             //圆形计算所需个数,应比总数量少1,最后一位为当前识别目标
#define BUFF_MIN_DISTANCE 100             //两次记录最短间距
#define BUFF_ANGLE_NUM 7             //两次记录最短间距
#define TIME_ANGLE_MIN 0.5             //最小角度差
#define G 9.80665
/*************************************************************/
RM_BuffData BuffBox[BUFFER_BUFF_BOX];       //存储最近几帧的大符信息
int BuffNum = 0;                                                              //当前存储Buff数量

/**
 * @brief FindBuff::BuffModeSwitch
 * @param Src
 * @return 返回大符识别矩形
 * @remark 大符识别接口,传入当前帧图像,进行图像处理和寻找目标,返回最终待击打矩形,搭配相应滤光片使用使图像更稳定
 */
 RM_BuffData* FindBuff::BuffModeSwitch(Mat Src){
    Mat dst;
    RM_BuffData Buff;
    PreDelBuff(Src,dst);
    vector<RotatedRect> BuffClump = FindBestBuff(Src,dst);
    if(BuffClump.size()<=0){
        cout<<"当前大符未识别到目标";
        return (RM_BuffData*)-1;
    }

    RotatedRect BuffObject  = GetShootBuff(BuffClump,Src);
    Buff.point[0] = Point2f(BuffObject.center.x - BuffObject.size.width/2,BuffObject.center.y - BuffObject.size.height/2);
    Buff.point[1] = Point2f(BuffObject.center.x + BuffObject.size.width/2,BuffObject.center.y - BuffObject.size.height/2);
    Buff.point[2] = Point2f(BuffObject.center.x + BuffObject.size.width/2,BuffObject.center.y + BuffObject.size.height/2);
    Buff.point[3] = Point2f(BuffObject.center.x - BuffObject.size.width/2,BuffObject.center.y + BuffObject.size.height/2);
    Buff.box = BuffObject;

    //存入数组,进入分析
    if(BuffNum == 0){
        BuffNum++;
        //初始化数组内容
        for(int i = 0;i<BUFFER_BUFF_BOX;i++){
            BuffBox[i] = Buff;
        }
    }else{
        //已初始化,进入连续计算
        int index = -1;
        float max_distance = 0;
        //寻找最远的序号
        for(int i = 0;i<3;i++){
            float distance = GetDistance(BuffObject.center,BuffBox[i].box.center);
            if(distance>BUFF_MIN_DISTANCE&&distance>max_distance){
                index = i;
                max_distance = distance;
            }
        }
        if(index!=-1&&GetDistance(BuffObject.center,BuffBox[(index+1)%BUFF_CIRCLE_BOX].box.center)>BUFF_MIN_DISTANCE
                &&GetDistance(BuffObject.center,BuffBox[(index+2)%BUFF_CIRCLE_BOX].box.center)>BUFF_MIN_DISTANCE){
            //符合条件存入数组
            BuffBox[index] = Buff;
            BuffNum++;                  //指向下一个位置
        }

        BuffBox[BUFFER_BUFF_BOX-1] = Buff;
    }

    for(int t = 0;t<3;t++){
        for(int i = 0;i<4;i++){
            line(Src,BuffBox[t].point[(i+1)%4],BuffBox[t].point[i%4],Scalar(255,0,0),1,4);
        }
    }

    if(BuffNum<3)
        return  (RM_BuffData*)-1;
    return BuffBox;
}

/**
 * @brief FindBuff::PreDelBuff
 * @param Src
 * @param dst
 * @return
 */
void FindBuff::PreDelBuff(Mat Src, Mat &dst){
    double t = (double)cvGetTickCount();            //计时
    cvtColor(Src,dst,CV_RGB2GRAY);
    threshold(dst,dst,70,255,CV_THRESH_BINARY);
    cv::Mat gray_element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7));
//    dilate(dst,dst,gray_element);
//    erode(dst,dst,gray_element);
//    imshow("灰度二值化",dst);
//    vector<Mat> spli;
    Mat hsv;
    Mat mask;
    cvtColor(Src,hsv,CV_RGB2HSV);
//    split(hsv,spli);
    inRange(hsv, Scalar(0, 10, 46), Scalar(180, 60, 255), mask);
    dilate(mask,mask,gray_element);
//    imshow("mask",mask);
    dst = dst - mask;
    dilate(dst,dst,gray_element);
    Mat element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    erode(dst,dst,element);

    t=(double)cvGetTickCount()-t;
    t = t/(cvGetTickFrequency()*1000);                                //t2为一帧的运行时间,也是单位时间
   printf("used time is %gms\n",t);
//    imshow("分割",dst);

}

/**
 * @brief FindBuff::FindBestBuff
 * @param Src
 * @return
 */
vector<RotatedRect> FindBuff::FindBestBuff(Mat Src,Mat & dst){
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<RotatedRect>box_buffs;
    //寻找全部轮廓,用于计算内轮廓数量
    findContours(dst,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    bool success = false;                               //记录是否成功找到符合要求扇叶
    for(size_t i = 0;i<hierarchy.size();i++){
        if(hierarchy[i][2] == -1)continue;
        if(contours[i].size()<6)continue;
        RotatedRect box = fitEllipse(contours[i]);

        float shanye_bili = box.size.height/box.size.width;
        if(shanye_bili>2.9||(shanye_bili<1.7&&shanye_bili>1.5)||shanye_bili<1.1||box.size.area()<Src.rows*2)continue;
        ellipse(Src, box, Scalar(255,0,255), 5, CV_AA);
        int * nei_lunkuo = (int *)malloc(contours.size()*sizeof(int));
        memset(nei_lunkuo, 0, contours.size()*sizeof(int));
        //判断第一个内轮廓是否符合标准
        if(contours[hierarchy[i][2]].size()>=6){
            RotatedRect first_box = fitEllipse(contours[hierarchy[i][2]]);

//            cout<<"首个扇叶bili:"<<(float)first_box.size.height/(float)first_box.size.width<<endl;
//            cout<<"首个面积比1:"<<box.size.area()/first_box.size.area()<<endl;
//            cout<<"首个轮廓面积："<<(Src.cols*Src.rows)/box.size.area()<<endl;
            if(box.size.area()/first_box.size.area()<10)
                *(nei_lunkuo + hierarchy[i][2]) = 1;
        }
        int j = hierarchy[i][2];
        while(hierarchy[j][0]!=-1){
            if(contours[hierarchy[j][0]].size()<6){
                j = hierarchy[j][0];
                continue;
            }
            RotatedRect box2 = fitEllipse(contours[hierarchy[j][0]]);
            if(box.size.area()/box2.size.area()>10){
                j = hierarchy[j][0];
                continue;
            }
            *(nei_lunkuo + hierarchy[j][0]) = 1;
            j = hierarchy[j][0];
        }
        int z = hierarchy[i][2];
        while(hierarchy[z][1]!=-1){
            if(contours[hierarchy[z][1]].size()<6){
                z = hierarchy[z][1];
                continue;
            }
            RotatedRect box2 = fitEllipse(contours[hierarchy[j][0]]);
            if(box.size.area()/box2.size.area()>10){
                z = hierarchy[z][1];
                continue;
            }
            if(box.size.area()/box2.size.area()>7)continue;
            *(nei_lunkuo + hierarchy[z][1]) = 1;
            z = hierarchy[z][1];
        }
        int num = 0;
        for(int t = 0;t<contours.size();t++){
            if(*(nei_lunkuo + t) == 1){
                num++;
            }
        }
//        cout<<"内轮廓数量:"<<num<<endl;
        if(num == 1){
            success = true;
            if(contours[hierarchy[i][2]].size()<6)continue;
            RotatedRect box_buff = fitEllipse(contours[hierarchy[i][2]]);
            if(box_buff.angle<5||box_buff.angle>175){
                if(box.angle>5&&box.angle<175)continue;
            }else{
                if(!((tan(box_buff.angle*PI/180)*tan(box.angle*PI/180) + 1)<0.3||(box_buff.angle - box.angle)<5))continue;
            }
//            cout<<"轮廓面积最终比："<<(Src.cols*Src.rows)/box_buff.size.area()<<endl;
            box_buffs.push_back(box_buff);
            ellipse(Src, box_buff, Scalar(255,0,0), 5, CV_AA);
        }
        free(nei_lunkuo);
//        if(num == 2){
//            if(contours[hierarchy[i][2]].size()<6)continue;
//            RotatedRect box = fitEllipse(contours[hierarchy[i][2]]);

//            ellipse(dst, box, Scalar(0,255,0), 5, CV_AA);
//            if(hierarchy[hierarchy[i][2]][0] != -1){
//                if(contours[hierarchy[hierarchy[i][2]][0]].size()<6)continue;
//                RotatedRect box = fitEllipse(contours[hierarchy[hierarchy[i][2]][0]]);

//                ellipse(dst, box, Scalar(0,0,255), 5, CV_AA);
//            }
//        }

    }
//    cout<<"完成"<<endl;
    imshow("绘制ing",Src);
//    //保存录像
//    outputVideo<<dst;
//    if(!success)
//        waitKey();
    return box_buffs;
}

/**
 * @brief FindBuff::GetShootBuff
 * @param box_buffs
 * @param Src
 * @return
 */
RotatedRect FindBuff::GetShootBuff(vector<RotatedRect> box_buffs,Mat Src){
    if(box_buffs.size() == 1)
        return box_buffs[0];
    //为最终得到的大符内轮廓打分
    int *grade = (int *)malloc(box_buffs.size()*sizeof(int));
    memset(grade, 0, box_buffs.size()*sizeof(int));
    for(int i = 0;i<box_buffs.size();i++){
        *(grade+i) = 100*(1 - fabs(box_buffs[i].size.height/box_buffs[i].size.width - BUFF_W_RATIO_H)/BUFF_W_RATIO_H);
        cout<<fabs((Src.cols*Src.rows)/box_buffs[i].size.area())<<endl;
        *(grade+i) += 100*(1 - fabs((Src.cols*Src.rows)/box_buffs[i].size.area() - BUFF_AREA_RATIO)/BUFF_AREA_RATIO);
    }
    int max_grade = *grade;
    int max_xuhao = 0;
    for(int t = 1;t<box_buffs.size();t++){
        if(*(grade+t)>max_grade){
            max_grade = *(grade+t);
            max_xuhao = t;
        }
    }
    free(grade);
    return box_buffs[max_xuhao];
}

