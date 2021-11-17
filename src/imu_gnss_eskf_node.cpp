#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_gnss_eskf_node");
    ros::NodeHandle nh;

    ROS_INFO("Hello world!");
}
