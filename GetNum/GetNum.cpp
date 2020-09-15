/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  GetNum.cpp
  *Author:  解佳朋
  *Version: 1.2.3.191017_RC
  *Date:  2020.07.24
  *Description: 利用svm分类器进行装甲数字识别
  *Function List:
     1.GetNumber:   数字识别传入接口
  * Others:
        所传入Src为装甲识别后所得ROI
**********************************************************************************/
#include<include/GetNum.h>
#include <opencv2/objdetect/objdetect.hpp>
#define sum 324
//#define SAVE_PICTURE

svm_model *svmModel=svm_load_model("/home/xiejiapeng/xie_jia_peng/RM2020-Horizon-InfantryVisionDetector/RM2020-Horizon-InfantryVisionDetector/svmTrainFile/100.txt");


//height为灯条高度
int GetNumber(Mat Src,Point2f center,float height ,float angle){

    float y_on = center.y - height;
    float y_down = center.y  + height;

    float x_left = center.x - height;
    float x_right = center.x + height;

    if(angle>90){
        angle = -(180 - angle);
    }

    if(y_on<0){
        y_on = 0;
    }
    if(y_down>Src.rows){
        y_down = Src.rows;
    }
    if(x_left<0){
        x_left = 0;
    }
    if(x_right > Src.cols){
        x_right = Src.cols;
    }

    Mat armor_roi = Src(Rect(x_left,y_on,x_right - x_left,y_down - y_on));

    if(angle>5||angle<175){
        Mat rot_mat = getRotationMatrix2D(Point(armor_roi.cols/2,armor_roi.cols/2), angle, 1.0);//求旋转矩阵
        Size dst_sz(armor_roi.cols,armor_roi.rows);
        warpAffine(armor_roi, armor_roi, rot_mat, dst_sz,INTER_NEAREST);//原图像旋转
    }

  Mat src_num = armor_roi(Rect(armor_roi.cols/5,0,armor_roi.cols*3/5,armor_roi.rows));

  // **********gamma算法
    float fGamma=1/1.7;
    unsigned char lut[256];
    for( int i = 0; i < 256; i++ )
    {
        lut[i] = saturate_cast<uchar>(pow((float)(i/255.0), fGamma) * 255.0f);
    }
    MatIterator_<Vec3b> it, end;
    for( it = src_num.begin<Vec3b>(), end = src_num.end<Vec3b>(); it != end; it++ )
    {
        //(*it)[0] = pow((float)(((*it)[0])/255.0), fGamma) * 255.0;
        //(*it)[1] = pow((float)(((*it)[1])/255.0), fGamma) * 255.0;
//        (*it)[2] = pow((float)(((*it)[2])/255.0), 1.3) * 255.0f;
        (*it)[0] = lut[((*it)[0])];
        (*it)[1] = lut[((*it)[1])];
        (*it)[2] = lut[((*it)[2])];
    }



#ifdef SAVE_PICTURE
    time_t rawtime;
    struct tm *ptminfo;

    time(&rawtime);
    ptminfo = localtime(&rawtime);

    struct timeval tv;
    gettimeofday(&tv,NULL);

    cout<<"年："<<ptminfo->tm_year+1900<<endl;

    string time = to_string(ptminfo->tm_year+1900)+to_string(ptminfo->tm_mon + 1)+to_string(ptminfo->tm_mday)+to_string(ptminfo->tm_hour)
            +to_string(ptminfo->tm_min)+to_string(ptminfo->tm_sec)+to_string(tv.tv_usec);
     String ImageName = "/home/xiejiapeng/图片/4/"+time+".jpg";
     imwrite(ImageName,result1);
     cout<<"保存成功！"<<endl;

#endif

//     src_num.resize(28,28);
     resize(src_num,src_num,Size(28,28));
//     imshow("resize",src_num);
     cvtColor(src_num,src_num,CV_RGB2GRAY);

     int gray_ave_number = 0;

     int rows=src_num.rows;
     int cols=src_num.cols*src_num.channels();
     for(int i=0; i<rows; i++)
     {
         uchar *p=src_num.ptr<uchar>(i);
         for(int j=0; j<cols;j++)
         {
             gray_ave_number += *p++;
         }
     }
     gray_ave_number /= (rows*cols);

     for(int i=0; i<rows; i++)
     {
         uchar *p=src_num.ptr<uchar>(i);
         for(int j=0; j<cols;j++)
         {
             if(*p>gray_ave_number){
                 *p++ = 255;
             }else{
                 *p++ = 0;
             }
         }
     }

     threshold(src_num,src_num,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU);
//     imshow("二值数字",src_num);


//     double t = (double)cvGetTickCount();            //计时

     HOGDescriptor hog(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);
     //HOG检测器，用来计算HOG描述子的
//            int DescriptorDim;//HOG描述子的维数，由图片大小、检测窗口大小、块大小、细胞单元中直方图bin个数决定

         vector<float> descriptors;//HOG描述子向量
                 hog.compute(src_num, descriptors, Size(7, 7));//计算HOG描述子，检测窗口移动步长(8,8)
//        size_t sum = SrcImage.cols;
     svm_node* features = new svm_node[sum+1];

     for(size_t i = 0;i<descriptors.size();i++){
         features[i].index = i+1;
         features[i].value = descriptors[i];
     }


     features[sum].index = -1;
     int predictValue = 0;                      //最终识别数值
     double a[6];
     double Probability = svm_predict_probability(svmModel, features,a);

     int k = (int)Probability - 1;
     if(a[k]>0.9){
         predictValue = Probability;
     }else if(a[k]>=0.6){
         predictValue = 0;
     }else{
         predictValue = -1;
     }

     cout<<"识别数字："<<Probability<<"      概率:"<<a[k]<<endl;
//     t=(double)cvGetTickCount()-t;
//     t = t/(cvGetTickFrequency()*1000);

//     cout<<"数字识别时间："<<t<<"ms"<<endl;

//     return 3;
    return (int)predictValue;
}

