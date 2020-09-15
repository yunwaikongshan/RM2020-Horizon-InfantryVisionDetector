/*********************************************************************************
  *Copyright(C),2018-2020,华北理工大学Horizon战队All Rights Reserved
  *FileName:  LongFindArmor.h
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
#ifndef LONGFINDARMOR_H
#define LONGFINDARMOR_H
#include<include/Variables.h>
#include<include/AngleSolver.h>
#include<include/GetNum.h>

bool campare_led(led a,led b);                                                                                //灯条筛选函数
bool campare_Armor(RM_ArmorDate a,RM_ArmorDate b);                       //装甲高度筛选函数
bool campare_Armor_distance(RM_ArmorDate a,RM_ArmorDate b);   //装甲与上一帧装甲距离筛选函数

class LongFindArmor{
public:
    ArmorDate GetArmorDate(Mat Src);
    LongFindArmor();
    void fillHole(const Mat srcBw, Mat &dstBw);             //孔洞填充
private:
    //装甲长宽比范围
    float small_max_ratio = 2.65;
    float small_min_ratio = 0.9;
    float big_max_ratio = 6.0;
    float big_min_ratio = 2.7;

    vector<led> leds;
    vector<RM_ArmorDate> ArmorDates;
    void PreDeal(Mat Src,Mat & gray);
    void PreDeal(Mat Src,Mat & gray,Mat & binary);
    bool GetLeds(Mat Src,Mat gray);
    bool GetLeds(Mat Src,Mat gray,Mat binary);
    bool GetArmor();
    bool GetBestArmor(Mat Src,ArmorDate & BestArmor);
    void SetROI(Mat Src,ArmorDate BestArmor,Rect & roi);
};


/**
 * @brief 灯条根据x轴坐标在数组中升序排列
 * @param a     低位地址   ，     b       高位地址
 * @return bool 升序格式
 */
inline bool campare_led(led a,led b){
    return a.box.center.x<b.box.center.x;
}

/**
 * @brief campare_Armor 将装甲按高度从大到小排序
 * @param a 低位地址
 * @param b 高位地址
 * @return 降序格式
 */
inline bool campare_Armor(RM_ArmorDate a,RM_ArmorDate b){
    return (a.leds[0].box.size.height+a.leds[1].box.size.height)>(b.leds[0].box.size.height+b.leds[1].box.size.height);
}


#endif // LONGFINDARMOR_H
