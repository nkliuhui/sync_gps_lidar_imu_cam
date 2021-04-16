#include <ros/ros.h>
#include <string>
#include <sstream>
#include <serial/serial.h>
// 解析时间--计算pc和gps的时间延迟
#include <ctime>

#include <time.h>
#include <string.h>
#include <stdlib.h> 
#include <sensor_msgs/NavSatFix.h>

std::string serial_port;
int baudRate;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");

    ros::NodeHandle node;

    ros::Publisher pub_gps = node.advertise<sensor_msgs::NavSatFix>("/gps_foo", 10);

    ros::Rate loop(10);

    // dasdasdasdasdasds

    sensor_msgs::NavSatFix msg;

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();

        msg.header.frame_id = "gps";
        msg.header.stamp = ros::Time::now();

        pub_gps.publish(msg);
        
    }

    return 0;
}