#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H
#include "linux/videodev2.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
namespace myown {
/**
 * @brief The VideoCapture class
 * @param VIDIOC_QUERYCAP 查询设备的属性
 * @param VIDIOC_ENUM_FMT 帧格式
 * @param VIDIOC_S_FMT 设置视频帧格式，对应struct v4l2_format
 * @param VIDIOC_G_FMT 获取视频帧格式等
 * @param VIDIOC_REQBUFS 请求/申请若干个帧缓冲区，一般为不少于3个
 * @param VIDIOC_QUERYBUF 查询帧缓冲区在内核空间的长度和偏移量
 * @param VIDIOC_QBUF 将申请到的帧缓冲区全部放入视频采集输出队列
 * @param VIDIOC_STREAMON 开始视频流数据的采集
 * @param VIDIOC_DQBUF 应用程序从视频采集输出队列中取出已含有采集数据的帧缓冲区
 * @param VIDIOC_STREAMOFF 应用程序将该帧缓冲区重新挂入输入队列
 * @param VIDIOC_S_CTRL 设置新的命令值
 * @param
 */
class VideoCapture
{
public:
    VideoCapture(const char * device, int buffer_size_) : buffer_size(buffer_size_){
        fd = open(device,O_RDWR); //open the capture
         video_height = 0;
         video_width = 0;
        buffer_id = 0;
//        int video_width,video_height;
        mb = new MapBuffer[buffer_size];
    }
    ~VideoCapture();
    bool startStream();
    bool closeStream();
    bool setVideoFormat(int width, int height, bool mjpg);
    bool setExpousureTime(bool auto_exp, int t);
    bool info();
    friend bool closedCapture(VideoCapture &cap);
    VideoCapture& operator >> (cv::Mat & image);
    bool setApertureNumber(int value);                  //设置光圈


private:
     bool initMMap();
    void cvtRaw2Mat(const void* data, cv::Mat &image);

    inline bool refreshVideoFormat();
    int xioctl(int fd,int request,void *arg);
private:
    struct MapBuffer
    {
        void *ptr;
        unsigned int size;
    };


    unsigned int format;

    int fd;
    int video_width,video_height;
    int buffer_size,buffer_id;
    MapBuffer *mb;
};






}

#endif // VIDEOCAPTURE_H
