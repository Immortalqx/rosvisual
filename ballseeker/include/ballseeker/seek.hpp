#pragma once
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "vector"
#include "sstream"
#include "string"
#include "yaml-cpp/yaml.h"
#include <dynamic_reconfigure/server.h>
#include <ballseeker/SetParamConfig.h>

const int red_ball = 0;
const int yellow_ball = 1;
const int blue_ball = 2;
const int no_save=3;

const int hsv_process=0;
const int bgr_process=1;

class seek {
public:
    seek(int service_id = 0);

    void getstart();        //从摄像头获取图片、双边滤波、转化hsv
    void display();         //找球并展示最终效果（调试时可以展示中间效果）

private:
    int bgr_min[3];
    int bgr_max[3];
    int hsv_min[3];
    int hsv_max[3];

    cv::Mat src;
    cv::Mat bgr_dst;
    cv::Mat hsv_dst;

    cv::Mat see_red[3];
    cv::Mat see_yellow[3];
    cv::Mat see_blue[3];

    cv::VideoCapture capture;

    YAML::Node config;
    std::string filepath;

    dynamic_reconfigure::Server<ballseeker::SetParamConfig> server;
    dynamic_reconfigure::Server<ballseeker::SetParamConfig>::CallbackType f;

    static int save_type;
    static int dynamic_bgr_min[3];
    static int dynamic_bgr_max[3];
    static int dynamic_hsv_min[3];
    static int dynamic_hsv_max[3];

    void updateparam(int type_of_ball = 3);
    void saveparam();
    bool isSetParam(int type_of_ball = 3);

    void process(cv::Mat *input, int type_of_ball);

    static void dynamic_callback(ballseeker::SetParamConfig &config, uint32_t level);
};
