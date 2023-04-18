//INCLUDE
#include <ros/ros.h>
#include <iostream>
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

//CODE
using namespace dji_osdk_ros;

AVCodecContext*       pCodecCtx;
AVCodec*              pCodec;
AVCodecParserContext* pCodecParserCtx;
SwsContext*           pSwsCtx;
AVFrame* pFrameYUV;
AVFrame* pFrameRGB;
uint8_t* rgbBuf;
size_t   bufSize;

bool ffmpeg_init()
{
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(NULL);
    if (!pCodecCtx)
    {
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, NULL) < 0)
    {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx)
    {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV)
    {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB)
    {
        return false;
    }

    pSwsCtx = NULL;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
}

void show_rgb(CameraRGBImage img, char* name)
{
  std::cout << "#### Got image from:\t" << std::string(name) << std::endl;
  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cvtColor(mat, mat, cv::COLOR_RGB2BGR);
  imshow(name,mat);
  cv::waitKey(1);
}

void decodeToDisplay(uint8_t *buf, int bufLen)
{
    uint8_t* pData   = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

    AVPacket pkt;
    av_init_packet(&pkt);
    while (remainingLen > 0)
    {
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData        += processedLen;

        if (pkt.size > 0)
        {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture)
            {
                //DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            }
            else
            {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                //DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if(NULL == pSwsCtx)
                {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, NULL, NULL, NULL);
                }

                if(NULL == rgbBuf)
                {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t*) av_malloc(bufSize);
                    avpicture_fill((AVPicture*)pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if(NULL != pSwsCtx && NULL != rgbBuf)
                {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;
                    cv::Mat mat(pFrameRGB->height, pFrameRGB->width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->width * 3);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
                    cv::imshow("camera_stream_node", mat);
                    cv::waitKey(1);
                }
            }
        }
    }
    av_free_packet(&pkt);
}


void fpvCameraStreamCallBack(const sensor_msgs::Image& msg)
{
  CameraRGBImage img;
  img.rawData = msg.data;
  img.height  = msg.height;
  img.width   = msg.width;
  char Name[] = "FPV_CAM";
  show_rgb(img, Name);
  std::cout<<"height is"<<msg.height<<std::endl;
  std::cout<<"width is"<<msg.width<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_stream_node");
    ros::NodeHandle nh;

    /*! H264 flow init and H264 decoder init */
//    auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
//    dji_osdk_ros::SetupCameraH264 setupCameraH264_;
//    ffmpeg_init();

    /*! RGB flow init */
    auto setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
    auto fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, fpvCameraStreamCallBack);
    dji_osdk_ros::SetupCameraStream setupCameraStream_;

    setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
    setupCameraStream_.request.start = 1;
    setup_camera_stream_client.call(setupCameraStream_);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(20).sleep();

    setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
    setupCameraStream_.request.start = 0;
    setup_camera_stream_client.call(setupCameraStream_);

    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
    ros::waitForShutdown();

    return 0;
}
