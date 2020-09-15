#include "include/VideoCapture.h"
namespace myown {
VideoCapture & VideoCapture::operator >> (cv::Mat & image)
{

    struct v4l2_buffer bufferinfo = {0};
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_id;
    if(ioctl(fd,VIDIOC_DQBUF,&bufferinfo) < 0){
        perror("Error:VIDIOC_DQBUF error!");
        exit(1);//非正常退出
    }

    cvtRaw2Mat(mb[buffer_id].ptr,image);

    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_id;
    // 申请下一次
    if(ioctl(fd,VIDIOC_QBUF,&bufferinfo) < 0){
        perror("Error:VIDIOC_QBUF error!");
        exit(1);
    }
    ++buffer_id;
    buffer_id = buffer_id >= buffer_size ? buffer_id - buffer_size : buffer_id;

    return *this;

}
void VideoCapture::cvtRaw2Mat(const void * data, cv::Mat &image)
{
    if(format == V4L2_PIX_FMT_MJPEG){
        cv::Mat src(video_height,video_width,CV_8UC3,(void *)data);
        image = cv::imdecode(src,1);    //解码
    }else if (format == V4L2_PIX_FMT_YUYV) {
        cv::Mat yuyv(video_height,video_width,CV_8UC2, (void *)data);
        cv::cvtColor(yuyv, image, CV_YUV2BGR_YUYV);
    }
}
bool VideoCapture::setVideoFormat(int width, int height, bool mjpg)
{
//    if(video_height == height && video_width == width && format == )
//        return true;
    video_height = height;
    video_width = width;
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.pixelformat = mjpg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
        std::cout<<"YES"<<std::endl<<std::endl;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;//任何一个格式都支持．驱动选择使用哪一个格式依赖于硬件能力，以及请求的image尺寸
    if(xioctl(fd,VIDIOC_S_FMT,&fmt) == -1){ /* 设置捕获视频的格式 */
        std::cout<<"Error: Set format error!"<<std::endl;
        return false;
    }
    return true;

}

/**
 * @brief VideoCapture::setApertureNumber           设置亮度
 * @param value     亮度值
 * @brief 调节范围:-64~64
 * @return
 */
bool VideoCapture::setApertureNumber(int value){

    struct v4l2_control control_s;
    control_s.id=V4L2_CID_BRIGHTNESS;;
        control_s.value=value;
        if(ioctl(fd,VIDIOC_S_CTRL,&control_s)==-1)
        {
            perror("ioctl");
            exit(EXIT_FAILURE);
        }
        return true;

//    struct v4l2_queryctrl  Setting;
//    Setting.id = V4L2_EXPOSURE_MANUAL;
////    Setting.type = V4L2_EXPOSURE_MANUAL;
//     ioctl(fd, VIDIOC_QUERYMENU,&Setting);
//    std::cout<<"最小值:"<<Setting.minimum<<"    最大值:"<<Setting.maximum<<std::endl;

//    struct v4l2_control control_s;
//    control_s.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;

//    control_s.value = V4L2_EXPOSURE_APERTURE_PRIORITY;

//    if(xioctl(fd,VIDIOC_G_CTRL,&control_s)){
//        perror("Error:Set manual error!");
//        return false;
//    }

//    control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
//    control_s.value = 100;
//    if(xioctl(fd,VIDIOC_G_CTRL,&control_s)){ /* 设置新的命令值 */
//        printf("Set Time Exposure error!\n");
//        return false;
//    }
//    std::cout<<"光圈设置成功"<<std::endl;
//    return true;
}

bool VideoCapture::setExpousureTime(bool auto_exp, int t)
{

    if(auto_exp)
    {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_AUTO;
        if(xioctl(fd,VIDIOC_S_CTRL,&control_s) < 0){
            printf("Set Auto Exposure error!\n");
            return false;
        }
    }else {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_MANUAL;
        if(xioctl(fd,VIDIOC_S_CTRL,&control_s)){
            perror("Error:Set manual error!");
            return false;
        }


        control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control_s.value = t;
        if(xioctl(fd,VIDIOC_S_CTRL,&control_s)){ /* 设置新的命令值 */
            printf("Set Time Exposure error!\n");
            return false;
        }
        std::cout<<"手动曝光设置成功!"<<std::endl;
    }
    return true;
}
bool VideoCapture::startStream()
{
    refreshVideoFormat();
    if(initMMap() == false)
        return false;

    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd, VIDIOC_STREAMON, &type) == -1){
        perror("Error:VIDIOC_STREAMON!");
        return false;
    }
    return true;
}
inline bool VideoCapture::refreshVideoFormat()
{   //查询格式
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd,VIDIOC_G_FMT,&fmt) == -1){ /* 获取设置支持的视频格式 */
        perror("Error:Querying fmt error!\n");//查询格式错误
        return false;
    }
    video_height = fmt.fmt.pix.height;
    video_width = fmt.fmt.pix.width;
    format = fmt.fmt.pix.pixelformat;
    return true;
}
bool VideoCapture::initMMap()
{
    struct v4l2_requestbuffers bufrequest = {0};
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = buffer_size;

    if(ioctl(fd,VIDIOC_REQBUFS,&bufrequest) == -1){
        perror("Error:VIDIOC_REQBUFS!");
        return false;
    }
    for(int i = 0; i < buffer_size; i++)
    {
        struct v4l2_buffer bufferinfo = {0};
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;

        if(ioctl(fd,VIDIOC_QUERYBUF,&bufferinfo) == -1){
            perror("Error:VIDIO_QUERYBUF!");
            return false;
        }
        //mmap返回值：成功返回映射的虚拟内存地址的起始地址，失败返回 MAP_FAILED
        mb[i].ptr = mmap(NULL, //映射放到哪里(虚拟地址)，一般传NULL，让内核自己决定
                     bufferinfo.length,
                     PROT_READ | PROT_WRITE,//页内容可以被读取 页可以被写入
                     MAP_SHARED, //与其它所有映射这个对象的进程共享映射空间。对共享区的写入，相当于输出到文件。直到msync()或者munmap()被调用，文件实际上不会被更新。
                     fd,
                     bufferinfo.m.offset);
        mb[i].size = bufferinfo.length;
        if(mb[i].ptr == MAP_FAILED){
            perror("Error:MAP_FAILED@");
            return false;
        }
        memset(mb[i].ptr,0,bufferinfo.length);//按字节对内存块进行初始化wei 0
        // Put the buffer in the incoming queue.
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;
        if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) == -1){
            perror("Error:VIDIOC_QBUF!");
            return false;
        }

    }
    return true;
}

bool VideoCapture::info()
{
    struct v4l2_capability caps = {0};
    if(xioctl(fd,VIDIOC_QUERYCAP, &caps) == -1){
        perror("Error:VIDIOC_QUERYCAP!");
        return false;
    }

    printf( "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d.%d\n"
            "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);

    struct v4l2_cropcap cropcap = {0};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd,VIDIOC_CROPCAP,&cropcap) == -1){
        perror("Error:Querying Cropping Capanilities\n");
        return false;
    }
    printf( "Camera Cropping:\n"
            "  Bounds: %dx%d+%d+%d\n"
            "  Default: %dx%d+%d+%d\n"
            "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
            strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
            c = fmtdesc.flags & 1? 'C' : ' ';
            e = fmtdesc.flags & 2? 'E' : ' ';
            printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
            fmtdesc.index++;
    }
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
        perror("Querying Pixel Format\n");
        return false;
    }
    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    printf( "Selected Camera Mode:\n"
            "  Width: %d\n"
            "  Height: %d\n"
            "  PixFmt: %s\n"
            "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);
    struct v4l2_streamparm streamparm = {0};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(fd, VIDIOC_G_PARM, &streamparm)) {
        perror("Querying Frame Rate\n");
        return false;
    }
    printf( "Frame Rate:  %f\n====================\n",
            (float)streamparm.parm.capture.timeperframe.denominator /
            (float)streamparm.parm.capture.timeperframe.numerator);
}
int VideoCapture::xioctl(int fd, int request, void *arg)
{
    int r;
    do r = ioctl(fd,request,arg);
    while(r == -1 && EINTR == errno);
    return r;
}
 bool closedCapture(VideoCapture &cap)
{
    cap.closeStream();
    close(cap.fd);
    delete [] cap.mb;
}
bool VideoCapture::closeStream()
{
    buffer_id = 0;
    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd,VIDIOC_STREAMOFF,&type) == -1){
        perror("Error:VIDIOC_STREAMOFF Error!");
        return false;
    }
    for(int i = 0; i < buffer_size; i++){
        munmap(mb[i].ptr,mb[i].size);
    }
    return true;
}
VideoCapture::~VideoCapture(){
    close(fd);
    delete [] mb;
}

}
