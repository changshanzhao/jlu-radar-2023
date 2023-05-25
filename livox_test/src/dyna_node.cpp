#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "livox_test/configdConfig.h"

void cb(livox_test::configdConfig& conf, uint16_t level){
    ROS_INFO("Param changed : %5d, %5d", conf.max_intensity,conf.min_intensity);
}

int main(int argc, char**  argv){
    ros::init(argc, argv, "dyna_node");
    dynamic_reconfigure::Server<livox_test::configdConfig> server;
    dynamic_reconfigure::Server<livox_test::configdConfig>::CallbackType cbType;
    cbType = boost::bind(&cb, _1, _2);
    server.setCallback(cbType);

    ros::spin();

    return 0;
}