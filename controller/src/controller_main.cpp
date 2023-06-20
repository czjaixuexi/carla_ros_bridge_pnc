/**
 * @file controller_main.cpp
 * @author czj 
 * @brief 
 * @version 0.1
 * @date 2023-06-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>

#include "controller_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference_line");
    carla_pnc::ControllerNode controller_node;
    ROS_INFO("The controller is starting to operate");
    controller_node.MainLoop();
    return 0;
}