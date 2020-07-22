#include "../include/ballseeker/seek.hpp"
#include "ros/ros.h"

int seek::save_type=no_save;
int seek::dynamic_bgr_min[3];
int seek::dynamic_bgr_max[3];
int seek::dynamic_hsv_min[3];
int seek::dynamic_hsv_max[3];

int main(int argc,char** argv)
{
    ros::init(argc,argv,"seekernode");
    ros::NodeHandle n;

    seek icu(1);

    while(ros::ok())
    {
        icu.getstart();
        icu.display();
        ros::spinOnce();
    }

    return 0;
}
