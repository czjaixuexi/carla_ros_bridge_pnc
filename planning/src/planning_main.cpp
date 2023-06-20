/**
 * @file planning_main.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>

#include "planning_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    
    carla_pnc::PlanningNode planning_node;
    
    ROS_INFO("Start Planning");
    
    planning_node.MainLoop();

    return 0;
}