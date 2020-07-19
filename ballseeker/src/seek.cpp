#include "../include/ballseeker/seek.hpp"

seek::seek(int service_id)
{
    //设置摄像机参数的放这儿先
    capture.open(service_id);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    filepath="/home/npu-lqx/catkin_cv/src/ballseeker/param/";

    //设置回调函数，也先放这儿
    f = boost::bind(&dynamic_callback, _1, _2);
    server.setCallback(f);
}

seek::~seek()
{
    //先空着
}

void seek::getstart()
{
    //从摄像头中获取图片，并且初步处理
    capture>>src;
    cv::bilateralFilter(src,bgr_dst,20,1000,1000);
    cv::cvtColor(bgr_dst,hsv_dst,CV_BGR2HSV);
}

void seek::display()
{
    //检测小球并展示处理过程和最终效果
    process(see_red,red_ball,bgr_process);
    process(see_yellow,yellow_ball);
    process(see_blue,blue_ball);

    cv::imshow("SEE_RED_BALL",see_red);
    cv::imshow("SEE_YELLOW_BALL",see_yellow);
    cv::imshow("SEE_BLUE_BALL",see_blue);
    cv::imshow("FINALLY",src);

    cv::waitKey(30);
}

void seek::process(cv::Mat& input, int type_of_ball, int type_of_process)
{
    //具体的处理过程，包括更新参数、阈值化、膨胀、检测轮廓、检测最小包含矩形、画圆
    updateparam(type_of_ball);      //更新参数

    switch (type_of_process)        //检测以何种颜色空间来阈值化
    {
    case hsv_process:
        cv::inRange(hsv_dst,cv::Scalar(hsv_min[0],hsv_min[1],hsv_min[2]),
            cv::Scalar(hsv_max[0],hsv_max[1],hsv_max[2]),input);
        break;
    case bgr_process:
        cv::inRange(bgr_dst,cv::Scalar(bgr_min[0],bgr_min[1],bgr_min[2]),
            cv::Scalar(bgr_max[0],bgr_max[1],bgr_max[2]),input);
        break;
    default:
        break;
    }

    //膨胀
    cv::Mat element =cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));   
    cv::dilate(input, input, element);

    //轮廓检测
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Rect> rects;

    cv::findContours(input, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    //检测最小矩阵并且筛选其中接近正方形的一部分
    for (int i = 0; i < contours.size(); i++)
    {
        rects.push_back(cv::boundingRect(contours[i]));
        if ((rects.back().height < 100 || rects.back().width < 100) ||
            fabsf(rects.back().height - rects.back().width) > 20)
            rects.erase(rects.end() - 1);
    }

    //根据筛选出的矩阵确定圆心和画圆
    for (int i = 0; i < rects.size(); i++)
    {
        double R = (rects[i].height + rects[i].width) / 4;
        double x = rects[i].x + R;
        double y = rects[i].y + R;
        cv::Point center(x, y);
        cv::circle(src, center, R + 5, cv::Scalar(0, 0, 255), 5);
    }
}

void seek::updateparam(int type_of_ball)
{
    //根据待处理小球的种类来更新参数
    if(isSetParam(type_of_ball))    //检测参数是否已经通过rqt_reconfigure调整
        return;
    
    std::string finalpath;

    switch (type_of_ball)
    {
    case red_ball:
        finalpath=filepath+"red.yaml";
        break;
    case yellow_ball:
        finalpath=filepath+"yellow.yaml";
        break;
    case blue_ball:
        finalpath=filepath+"blue.yaml";
        break;
    default:
        return;
    }

    config=YAML::LoadFile(finalpath);

    hsv_min[0]=config["h_min"].as<int>();
    hsv_min[1]=config["s_min"].as<int>();
    hsv_min[2]=config["v_min"].as<int>();
    hsv_max[0]=config["h_max"].as<int>();
    hsv_max[1]=config["s_max"].as<int>();
    hsv_max[2]=config["v_max"].as<int>();

    bgr_min[0]=config["b_min"].as<int>();
    bgr_min[1]=config["g_min"].as<int>();
    bgr_min[2]=config["r_min"].as<int>();
    bgr_max[0]=config["b_max"].as<int>();
    bgr_max[1]=config["g_max"].as<int>();
    bgr_max[2]=config["r_max"].as<int>();

/*
    //之前硬编码保存的参数
    switch (type_of_ball)
    {
    case red_ball:
        hsv_min[0]= 0 ;hsv_min[1]= 105 ;hsv_min[2]= 108 ;
        hsv_max[0]= 11 ;hsv_max[1]= 255 ;hsv_max[1]= 255 ;
        break;
    case yellow_ball:
        hsv_min[0]= 18 ;hsv_min[1]= 56 ;hsv_min[2]= 177 ;
        hsv_max[0]= 71 ;hsv_max[1]= 187 ;hsv_max[1]= 255 ;
        break;
    case blue_ball:
        hsv_min[0]= 85 ;hsv_min[1]= 129 ;hsv_min[2]= 128 ;
        hsv_max[0]= 124 ;hsv_max[1]= 255 ;hsv_max[1]= 255 ;
        break;
    default:
        break;
    }
*/
}

bool seek::isSetParam(int type_of_ball)
{
    //检测是否通过rqt_reconfigure改变了参数，并且更新已经改变的参数
    if(type_of_ball==save_type)
    {
        bgr_min[0]=dynamic_bgr_min[0];
        bgr_min[1]=dynamic_bgr_min[1];
        bgr_min[2]=dynamic_bgr_min[2];
        bgr_max[0]=dynamic_bgr_max[0];
        bgr_max[1]=dynamic_bgr_max[1];
        bgr_max[2]=dynamic_bgr_max[2];

        hsv_min[0]=dynamic_hsv_min[0];
        hsv_min[1]=dynamic_hsv_min[1];
        hsv_min[2]=dynamic_hsv_min[2];
        hsv_max[0]=dynamic_hsv_max[0];
        hsv_max[1]=dynamic_hsv_max[1];
        hsv_max[2]=dynamic_hsv_max[2];

        saveparam();                    //无处安放的saveparam()函数
                                        //静态成员函数无法调用非静态的成员函数

        return true;
    }
    else
    {
        return false;
    }
}

void seek::saveparam()  
{
    //保存参数
    std::ofstream file;
    std::string finalpath;

    switch (save_type)
    {
    case red_ball:
        finalpath=filepath+"red.yaml";
        break;
    case yellow_ball:
        finalpath=filepath+"yellow.yaml";
        break;
    case blue_ball:
        finalpath=filepath+"blue.yaml";
        break;
    case no_save:
        return;
    default:
        return;
    }
    
    //下面的参数保存与dynamic_reconfigure关联！
    config=YAML::LoadFile(finalpath);
    file.open(finalpath.c_str());       //这里得弄个c_str()!?

    config["b_min"]=dynamic_bgr_min[0];
    config["g_min"]=dynamic_bgr_min[1];
    config["r_min"]=dynamic_bgr_min[2];
    config["b_max"]=dynamic_bgr_max[0];
    config["g_max"]=dynamic_bgr_max[1];
    config["r_max"]=dynamic_bgr_max[2];

    config["h_min"]=dynamic_hsv_min[0];
    config["s_min"]=dynamic_hsv_min[1];
    config["v_min"]=dynamic_hsv_min[2];
    config["h_max"]=dynamic_hsv_max[0];
    config["s_max"]=dynamic_hsv_max[1];
    config["v_max"]=dynamic_hsv_max[2];

    file.flush();                       //虽然不知道干啥用。。。
    file << config;
    file.close();
}

void seek::dynamic_callback(ballseeker::SetParamConfig &config, uint32_t level)
{
    //rqt_reconfigure的回调函数
    save_type=config.savetype;

    dynamic_hsv_min[0]=config.h_min;
    dynamic_hsv_min[1]=config.s_min;
    dynamic_hsv_min[2]=config.v_min;
    dynamic_hsv_max[0]=config.h_max;
    dynamic_hsv_max[1]=config.s_max;
    dynamic_hsv_max[2]=config.v_max;
    
    dynamic_bgr_min[0]=config.b_min;
    dynamic_bgr_min[1]=config.g_min;
    dynamic_bgr_min[2]=config.r_min;
    dynamic_bgr_max[0]=config.b_max;
    dynamic_bgr_max[1]=config.g_max;
    dynamic_bgr_max[2]=config.r_max;

}

void seek::creatbar()
{
    //创建滑动条进行调参
    cv::namedWindow("SetParam");
	cv::createTrackbar("minH", "SetParam", &hsv_min[0], 180, on_change);
	cv::createTrackbar("minS", "SetParam", &hsv_min[1], 255, on_change);
	cv::createTrackbar("minV", "SetParam", &hsv_min[2], 255, on_change);
	cv::createTrackbar("maxH", "SetParam", &hsv_max[0], 180, on_change);
	cv::createTrackbar("maxS", "SetParam", &hsv_max[1], 255, on_change);
	cv::createTrackbar("maxV", "SetParam", &hsv_max[2], 255, on_change);
	on_change(0, 0);
}

void seek::on_change(int, void*)
{
    //空的回调函数
}
