/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  LongFindArmor.cpp
  *Author:  解佳朋
  *Version: 1.3.2.200208_RC
  *Date:  2020.02.08
  *Description: 装甲识别
  *Function List:
     1.GetArmorDate 装甲识别接口
     2.PreDeal  图像预处理函数
     3.GetLeds  得到灯条
     4.GetArmor 得到装甲集合
     5.GetBestArmor 打分得到最适宜装甲
     6.SetROI   设置图像ROI
  * Others:
        包含两种识别模式,一种利用颜色分量排除掉灯条之外的颜色,然后与灰度二值图取与,得到较为干净的画面,
        另一种采用hsv通道分割,得到灯条位置,与灰度图进行位置的匹配,得到合适灯条.
        识别过程中采用彩色相机+滤光片辅助获得图片,对图片进行处理.
**********************************************************************************/
#include<include/LongFindArmor.h>

#define USING_RGB
//#define USING_HSV
//#define USING_ROI
#define SCORE_MIN 40      //分数许可最小值
#define LOST_MAX 8          //掉帧缓冲
#define MIN_DISTANCE 120 //两次相同目标距离

int lost_number = LOST_MAX;          //记录掉帧次数
Point last_left_on = Point(0,0);            //roi左上坐标点

#ifdef USING_ROI
Rect RoiRect;                                                          //存储roi矩形
bool IsSetRoi = false;                                          //记录是否设置过roi矩形
#endif //USING_ROI
#ifndef USING_NUMDECTOR
RM_ArmorDate Armor_old;                 //保存上一帧装甲
#endif //USING_NUM_DECTOR

/**
 * @brief LongFindArmor::LongFindArmor
 */
LongFindArmor::LongFindArmor(){

}

/**
 * @brief LongFindArmor::GetArmorDate 识别接口函数
 * @param Src 传入的rgb图像
 * @return 识别结果
 */
ArmorDate LongFindArmor::GetArmorDate(Mat Src){
#ifdef USING_ROI
    if(!IsSetRoi){
        //初始化
        RoiRect = Rect(0,0,Src.cols,Src.rows);
        IsSetRoi = true;
    }
    Mat Roi = Src(RoiRect);

#else
    Mat Roi = Src;
#endif//是否使用ROI
    Mat gray;
#ifdef USING_RGB
    PreDeal(Roi,gray);
#endif//是否使用RGB色彩空间识别
#ifdef USING_HSV
    Mat binary;
    PreDeal(Src,gray,binary);
#endif//是否使用HSV色彩空间识别模式

    ArmorDate BestArmorDate;                                    //记录识别数据
    //控制识别流程,顺序不可更改
    if(GetLeds(Roi,gray)&&GetArmor()&&GetBestArmor(Src,BestArmorDate)){
        //成功识别到目标
        lost_number = 0;
        Armor_old = BestArmorDate.Armor ;
    }else{
        //未识别到目标
        if(++lost_number < LOST_MAX){                   //缓冲
            BestArmorDate.Armor = Armor_old;
            BestArmorDate.status = buffering;
        }else{                                                                        //掉帧
            BestArmorDate.status = stop;
            ShootArmorNumber = -1;
            Armor_old.point[0] = Point2f(0,0);
        }
    }
#ifdef USING_ROI
    SetROI(Src,BestArmorDate,RoiRect);
#endif//是否使用ROI
    return BestArmorDate;
}

/**
 * @brief LongFindArmor::PreDeal    使用rgb颜色空间识别,利用比例关系减少误识别
 * @param Src   相机捕捉原图像
 * @param gray 处理后二值图
 */
void LongFindArmor::PreDeal(Mat Src, Mat &gray){
    vector<Mat> rgb;
    split(Src,rgb);
    Mat binary = Mat::zeros(Src.size(),CV_8UC1);
    //颜色分割,减少干扰
    if(armor_color == RED){
        for(int i = 0;i<binary.rows;i++){
            uchar* bin = binary.ptr<uchar>(i);
            uchar* r = rgb[2].ptr<uchar>(i);
            uchar* g = rgb[1].ptr<uchar>(i);
            uchar* b= rgb[0].ptr<uchar>(i);
            for(int j = 0;j<binary.cols;j++){
    //            bin[j] =((((float)r[j]/(float)(g[j]+b[j]/2))*100)>=GGrayValue&&g[j]>r[j]/2)?255:0;
                bin[j] = b[j]<(g[j]+r[j])*((float)RGrayWeightValue/100)?255:0;
            }
        }
        gray = rgb[2]>GrayValue;
    }else{
        for(int i = 0;i<binary.rows;i++){
            uchar* bin = binary.ptr<uchar>(i);
            uchar* r = rgb[2].ptr<uchar>(i);
            uchar* g = rgb[1].ptr<uchar>(i);
            uchar* b= rgb[0].ptr<uchar>(i);
            for(int j = 0;j<binary.cols;j++){
    //            bin[j] =((((float)r[j]/(float)(g[j]+b[j]/2))*100)>=GGrayValue&&g[j]>r[j]/2)?255:0;
                bin[j] = b[j]<(g[j]+r[j])*((float)BGrayWeightValue/100)?255:0;
            }
        }
        gray = rgb[0]>GrayValue;
    }

    Mat gray_element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
//    dilate(binary,binary,gray_element);
//    erode(binary,binary,gray_element);
//    dilate(gray,gray,gray_element);


#ifdef DEBUG
    imshow("灰度二值图",gray);
    imshow("颜色分割二值图",binary);
#endif
    gray -= binary;
//    dilate(gray,gray,gray_element);
//    erode(gray,gray,gray_element);

    blur(gray,gray,Size(5,5));
   medianBlur(gray,gray,3);
//   blur(gray,gray,Size(5,5));
//        GaussianBlur(gray,gray,Size(3,3),0,0);


#ifdef DEBUG
    imshow("合并",gray);
#endif

}

/**
 * @brief LongFindArmor::PreDeal        常规方案识别，两幅图找共同位置
 * @param Src 原图像
 * @param gray 灰度二值图
 * @param binary hsv空间分割图像,用于下一步的两个图像对照
 */
void LongFindArmor::PreDeal(Mat Src, Mat &gray, Mat &binary){
    Mat hsv_image;
    cvtColor(Src,hsv_image,CV_RGB2HSV);
    vector<Mat> hsv;
    split(hsv_image,hsv);
    gray = hsv[2]>V_ts;

    if(armor_color == RED){
        inRange(Src, Scalar(RLowH, RLowS, RLowV), Scalar(RHighH, RHighS, RHighV), binary); //Threshold the image
    }else{
        inRange(Src, Scalar(BLowH, BLowS, BLowV), Scalar(BHighH, BHighS, BHighV), binary); //Threshold the image
    }

//    medianBlur(binary,binary,3);
//    GaussianBlur(binary,binary,Size(5,5),0,0);
#ifdef DEBUG
    imshow("HSV_V二值化",gray);
    imshow("HSV分割",binary);
#endif
}

/**
 * @brief LongFindArmor::GetLeds
 * @param gray
 * @return 是否找到了两个以上的灯条
 */
bool LongFindArmor::GetLeds(Mat Src,Mat gray){
    std::vector<std::vector<cv::Point>>gray_contours;                       //存储灰度轮廓，用于处理，获得完整灯条
    cv::findContours(gray,gray_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,last_left_on);
//    cv::findContours(gray,gray_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,last_left_on);

    for(size_t i = 0;i<gray_contours.size();i++){
        if(gray_contours[i].size()<5)continue;
        RotatedRect box = fitEllipse(gray_contours[i]);
        //根据角度信息筛选
        if(fabs(box.angle - 90)<45)continue;
        //根据长宽比例筛选
        if(box.size.height/box.size.width>15||box.size.height/box.size.width<1.3)continue;
//        ellipse(Src, box, Scalar(0,0,255), 5, CV_AA);
        led led_new;
        led_new.box = box;
        int fuhao = 1;
        if(box.angle>90)
            fuhao = -1;
        led_new.led_on = Point2f(box.center.x + fuhao*box.size.height*sin(box.angle*PI/180)/2.0,box.center.y - fuhao*box.size.height*cos(box.angle*PI/180)/2.0);
        led_new.led_down = Point2f(box.center.x - fuhao*box.size.height*sin(box.angle*PI/180)/2.0,box.center.y + fuhao*box.size.height*cos(box.angle*PI/180)/2.0);
#ifdef IMAGE_DRAWING
        ellipse(Src, box, Scalar(0,0,255), 5, CV_AA);
#endif
        leds.push_back(led_new);
    }
    if(leds.size()>=2)
        return true;
    else{
        cout<<"灯条少于2"<<endl;
        return false;
    }
}

/**
 * @brief LongFindArmor::GetLeds        第二种方案通过两幅图寻找灯条,一幅颜色空间分割图像用于定位灯条,和灰度图中查找的轮廓比照
 * @param Src
 * @param gray提取灯条信息
 * @param binary定位灯条位置
 * @return 是否找到了两个以上的灯条
 */
bool LongFindArmor::GetLeds(Mat Src, Mat gray, Mat binary){
    std::vector<std::vector<cv::Point>>gray_contours;                       //存储灰度轮廓，用于处理，获得完整灯条
    std::vector<std::vector<cv::Point>>binary_contours;                       //存储hsv相减后轮廓，用于定位
    cv::findContours(gray,gray_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,last_left_on);
    findContours(binary,binary_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,last_left_on);

    vector<RotatedRect> hsv_leds;
    //遍历binary_contours，将拟合得椭圆存入hsv_leds
    for(size_t t = 0;t<binary_contours.size();t++){
        if(binary_contours[t].size()<5)continue;
        RotatedRect box = fitEllipse(binary_contours[t]);
#ifdef IMAGE_DRAWING
        ellipse(Src, box, Scalar(0,0,255), 5, CV_AA);
#endif
        hsv_leds.push_back(box);
    }

    for(size_t j = 0; j < gray_contours.size(); j++){
        if(gray_contours[j].size() <5 || 1e5 < gray_contours[j].size()) continue;                                             //判断gray_contours轮廓点数是否符合范围

        //椭圆拟合,根据灯条角度与比例筛选灯条，保留适宜目标
        RotatedRect box = fitEllipse(gray_contours[j]);
        //根据角度信息筛选
        if(fabs(fabs(box.angle) - 90)<45)continue;
        //根据长宽比例筛选
        if(box.size.height/box.size.width>12||box.size.height/box.size.width<2.5)continue;
        //定位，查找hsv_leds中是否有灯条与之匹配
        size_t t = 0;
        for(;t<hsv_leds.size();t++){
            if(GetDistance(hsv_leds[t].center,box.center)<10)break;
        }
        if(t == hsv_leds.size())continue;
        led led_new;
        led_new.box = box;
        int fuhao = 1;
        if(box.angle>90)
            fuhao = -1;
        led_new.led_on = Point2f(box.center.x + fuhao*box.size.height*sin(box.angle*PI/180)/2.0,box.center.y - fuhao*box.size.height*cos(box.angle*PI/180)/2.0);
        led_new.led_down = Point2f(box.center.x - fuhao*box.size.height*sin(box.angle*PI/180)/2.0,box.center.y + fuhao*box.size.height*cos(box.angle*PI/180)/2.0);

        //绘制
#ifdef IMAGE_DRAWING
        ellipse(Src, box, Scalar(0,0,255), 5, CV_AA);
#endif
        leds.push_back(led_new);
    }
    if(leds.size()>=2)
        return true;
    else{
        cout<<"灯条数少于两根"<<endl;
        return false;
    }
}


/**
 * @brief 将leds中灯体组合，寻找正确装甲板存入ArmorDate
 * @return bool 返回是否找到装甲板
 */
bool LongFindArmor::GetArmor(){
    //将内部灯条按中心x坐标升序排列
    sort(leds.begin(),leds.end(),campare_led);
    for(size_t i = 0;i<leds.size();i++){
        for(size_t j = i+1;j<leds.size();j++){
            //过大就终止循环
            if(leds[j].box.center.x - leds[i].box.center.x>leds[i].box.size.height*5)break;
            //根据角度筛选
            if((leds[i].box.angle<90&&leds[j].box.angle<90)||(leds[i].box.angle>90&&leds[j].box.angle>90)){
                if(fabs(leds[i].box.angle - leds[j].box.angle)>10)continue;
            }else if(leds[i].box.angle<90&&leds[j].box.angle>90){
                if(leds[i].box.angle - leds[j].box.angle > -173)continue;
            }else if(leds[i].box.angle>90&&leds[j].box.angle<90){
                 if(leds[j].box.angle - leds[i].box.angle > -173)continue;
            }else continue;


            //根据高度差筛选，极值15
            double height_cha_max = 15>(leds[i].box.size.height + leds[j].box.size.height)/10?(leds[i].box.size.height + leds[j].box.size.height)/10:15;
            if(fabs(leds[i].box.size.height - leds[j].box.size.height)>height_cha_max)continue;

            //根据宽度差筛选,极值15
            double width_on = GetDistance(leds[i].led_on,leds[j].led_on);
            double width_down = GetDistance(leds[j].led_down,leds[i].led_down);
            double width_cha_max = 15>(width_down + width_on)/15?(width_down + width_on)/15:15;
            if(fabs(width_down - width_on)>width_cha_max)continue;
            //根据平行角度差筛选,将y差值的符号给x可得装甲角得到灯条角,并排除水平0度
            double angle_armor = atan2(fabs(leds[i].led_on.y - leds[j].led_on.y),(leds[j].led_on.x - leds[i].led_on.x))*180/PI;
            if((leds[i].box.angle>90&&leds[j].box.angle<90)||(leds[i].box.angle<90&&leds[j].box.angle>90)){
                //防止内八或外八

                if(angle_armor>8)continue;


            }else if(leds[i].box.angle>90&&leds[j].box.angle>90){
                if((angle_armor + (leds[i].box.angle + leds[j].box.angle)/2)<162||(angle_armor + (leds[i].box.angle + leds[j].box.angle)/2)>210)continue;
            }else if(leds[i].box.angle<90&&leds[j].box.angle<90){
                if(fabs(angle_armor - (leds[i].box.angle + leds[j].box.angle)/2)>18)continue;
            }else{
                if(angle_armor>7)continue;
            }
            //定义装甲对象
            RM_ArmorDate Armor;
            //根据比例筛选
            double bili = ((width_on + width_down)/2)/((leds[i].box.size.height + leds[j].box.size.height)/2);
//            double height_x = (leds[j].box.center.x - leds[i].box.center.x)/((leds[i].box.size.height + leds[j].box.size.height)/2);            //高比x差值
            if(bili>=small_min_ratio&&bili<=small_max_ratio){
                Armor.IsSmall = true;
            }else if(bili>=big_min_ratio&&bili<=big_max_ratio){
                Armor.IsSmall = false;
            }else{
                //不符合比例条件
                continue;
            }
            //当内部还有灯条是便认为当前装甲为伪装甲
            int t = j - i;
            bool is_sure = false;           //记录是否找到
            float y_min , y_max;
            y_min = leds[i].led_on.y<leds[j].led_on.y?leds[i].led_on.y:leds[i].led_on.y ;
            y_max = leds[i].led_down.y>leds[j].led_down.y?leds[i].led_down.y:leds[i].led_down.y ;
            for(size_t z = i+1;z - i<t;z++){
                if(((leds[z].box.center.y>=y_min&&leds[z].box.center.y<=y_max)||(leds[z].led_on.y<=y_max&&leds[z].led_on.y>=y_min)||(leds[z].led_down.y<=y_max&&leds[z].led_down.y>=y_min))){
                    is_sure = true;
                    break;
                }
            }


            //筛选完毕将点存入*****0    1******,由于已排序，必定i在左侧
            //**********************3     2******
            Armor.point[0] = leds[i].led_on;
            Armor.point[1] = leds[j].led_on;
            Armor.point[2] = leds[j].led_down;
            Armor.point[3] = leds[i].led_down;

            //传参
            Armor.leds[0] = leds[i];
            Armor.leds[1] = leds[j];

            //装甲确定完毕
            ArmorDates.push_back(Armor);
        }
    }
    if(ArmorDates.size()<1){
        cout<<"装甲数小于1个"<<endl;
//        exit(0);
        return false;
    }else{
        return true;
    }
}

/**
 * @brief LongFindArmor::GetBestArmor   从装甲集合中挑选最合适装甲
 * @param BestArmor
 * @return
 */
bool LongFindArmor::GetBestArmor(Mat Src,ArmorDate &BestArmor){
    //判断是否仅识别到一个装甲
    if(ArmorDates.size() == 1){
        if(Armor_old.point[0].x == 0&&Armor_old.point[0].y == 0){
            float angle = getArmorAngle(ArmorDates[0].point);
            int ArmorNumber = GetNumber(Src,getArmorCenter(ArmorDates[0].point),(ArmorDates[0].leds[0].box.size.height+ArmorDates[0].leds[1].box.size.height)/2,angle);
            if(ArmorNumber == 2){
                cout<<"发现工程"<<endl;
                return false;
            }
            BestArmor.status = FirstFind;
        }else if(GetDistance(Armor_old.point[0],ArmorDates[0].point[0])<MIN_DISTANCE){
            BestArmor.status = Shoot;
        }else{
            //进入数字检测
            float angle = getArmorAngle(BestArmor.Armor.point);
            int ArmorNumber = GetNumber(Src,getArmorCenter(ArmorDates[0].point),(ArmorDates[0].leds[0].box.size.height+ArmorDates[0].leds[1].box.size.height)/2,angle);
            if(ArmorNumber == -1)
                return false;
            if(ArmorNumber == 2){
                cout<<"发现工程"<<endl;
                return false;
            }else if(ArmorNumber == ShootArmorNumber){
                BestArmor.status == Shoot;
            }else{
                ShootArmorNumber = ArmorNumber;
                BestArmor.status = FirstFind;
            }
        }
        BestArmor.Armor = ArmorDates[0];
        Armor_old = BestArmor.Armor;
        return true;
    }
    //第一次识别，前为中断状态
    if(Armor_old.point[0].x == 0&&Armor_old.point[0].y == 0){
        //按装甲高度排序
        sort(ArmorDates.begin(),ArmorDates.end(),campare_Armor);
        for(size_t i = 0;i<ArmorDates.size();i++){
            //进入数字检测
            float angle = getArmorAngle(BestArmor.Armor.point);
            int ArmorNumber = GetNumber(Src,getArmorCenter(ArmorDates[i].point),(ArmorDates[i].leds[0].box.size.height+ArmorDates[i].leds[1].box.size.height)/2,angle);
            if(ArmorNumber == -1)
                continue;
            if(ArmorNumber == 2){
                cout<<"发现工程"<<endl;
                continue;
            }
            //传参
            BestArmor.Armor = ArmorDates[i];
            Armor_old = BestArmor.Armor;
            BestArmor.status = FirstFind;
            ShootArmorNumber = ArmorNumber;
            return true;
        }
        return false;
    }
    //之前已存在目标,打分
    float score_max = SCORE_MIN;            //记录分数极值，SCORE_MIN为标志最小值
    int Index =  -1;                                               //存储最高分序号
    for(size_t i = 0;i<ArmorDates.size();i++){
         float score = 100 - (GetDistance(Armor_old.point[0] ,ArmorDates[i].point[0] )/ArmorDates[i].leds[0].box.size.height)*10;
        score += 100 - (fabs(Armor_old.leds[0].box.size.height -ArmorDates[i].leds[0].box.size.height )/Armor_old.leds[0].box.size.height)*100;
        if(score>=score_max){
            score_max = score;
            Index = i;
        }
    }
    if(Index!=-1){
        //找到符合标志的目标
        BestArmor.status = Shoot;
        BestArmor.Armor = ArmorDates[Index];
        Armor_old = ArmorDates[Index];
        return true;
    }else{
        //未找到，按装甲高度排序,发现数字相同则取连续,否则取最大装甲
        sort(ArmorDates.begin(),ArmorDates.end(),campare_Armor);
        int max_index = -1;
        int number = 0;
        for(size_t i = 0;i<ArmorDates.size();i++){
            //进入数字检测
            float angle = getArmorAngle(BestArmor.Armor.point);
            int ArmorNumber = GetNumber(Src,getArmorCenter(ArmorDates[i].point),(ArmorDates[i].leds[0].box.size.height+ArmorDates[i].leds[1].box.size.height)/2,angle);
            if(ArmorNumber == -1)
                continue;
            if(ArmorNumber == 2){
                cout<<"发现工程"<<endl;
                continue;
            }
            if(max_index == -1){
                max_index = i;
                number = ArmorNumber;
            }
            if(ArmorNumber == ShootArmorNumber){
                BestArmor.status = Shoot;
                BestArmor.Armor = ArmorDates[Index];
                Armor_old = ArmorDates[Index];
                return true;
            }
        }
        if(max_index != -1){
            BestArmor.status = FirstFind;
            BestArmor.Armor = ArmorDates[Index];
            Armor_old = ArmorDates[Index];
            ShootArmorNumber = number;
            return true;
        }
        return false;
    }

}

/**
 * @brief LongFindArmor::SetROI     设置图像下一帧ROI
 * @param Src
 * @param BestArmor
 * @param roi
 */
void LongFindArmor::SetROI(Mat Src, ArmorDate BestArmor, Rect &roi){

    if(BestArmor.status == stop){
        roi = Rect(0,0,Src.cols,Src.rows);
        last_left_on = Point(0,0);
        return;
    }else if(BestArmor.status == buffering){
        return;
        //缓冲时roi扩大
        int x = roi.x - Src.cols/10;
        int y = roi.y - Src.rows/10;
        //防止越界
        if(x<0)
            x = 0;
        if(y<0)
            y = 0;
        int width = roi.width + Src.cols/5;
        int height = roi.height + Src.rows/5;

        if(x+width>Src.cols){
            width = Src.cols - x;
        }
        if(y+height>Src.rows){
            height = Src.rows - y;
        }
        roi =  Rect(x,y,width,height);
        last_left_on = Point(x,y);
        return;
    }
    int width = 3*(int)GetDistance(BestArmor.Armor.leds[0].box.center,BestArmor.Armor.leds[1].box.center);
    int height = 3*(int)(BestArmor.Armor.leds[0].box.size.height + BestArmor.Armor.leds[1].box.size.height)/2;
    Point2f center = getArmorCenter(BestArmor.Armor.point);
    int x = (int)center.x - width/2;
    int y = (int)center.y - height/2;
    //防止越界
    if(x<0)
        x = 0;
    if(y<0)
        y = 0;
    if(x+width>Src.cols){
        width = Src.cols - x;
    }
    if(y+height>Src.rows){
        height = Src.rows - y;
    }
    last_left_on = Point(x,y);
    roi =  Rect(x,y,width,height);
    cout<<"object"<<center<<"   左上"<<last_left_on<<endl;

#ifdef IMAGE_DRAWING
#ifdef USING_ROI
    rectangle (Src,  roi,Scalar(0, 0, 255), 1, 8, 0);
#endif
#endif//是否绘制

}

/**
 * @brief LongFindArmor::fillHole       漫水填充函数
 * @param srcBw     输入单通道图像
 * @param dstBw     输出图像
 */
void LongFindArmor::fillHole(const Mat srcBw, Mat &dstBw){
        Size m_Size = srcBw.size();
        Mat Temp=Mat::zeros(m_Size.height+2,m_Size.width+2,srcBw.type());//延展图像
        srcBw.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));

        cv::floodFill(Temp, Point(0, 0), Scalar(255));

        Mat cutImg;//裁剪延展的图像
        Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);

        dstBw = srcBw | (~cutImg);
}


