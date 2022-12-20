//
// Created by skywoodsz on 2022/12/18.
//


#include "b1_high_level/b1_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "b1_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    B1Control b1_control(nh, nh_private);

    ros::spin();
    return 0;
}