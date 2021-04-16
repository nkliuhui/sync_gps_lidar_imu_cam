#include <ros/ros.h>
#include <string>
#include <sstream>
#include <serial/serial.h>
// 解析时间--计算pc和gps的时间延迟
#include <ctime>

#include <time.h>
#include <string.h>
#include <stdlib.h> 

#include <thread>

#include "gps_driver/TicToc.h"

typedef struct {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
} TIME;


std::string serial_port;
int baudRate;
bool ros_ok = false;
std::string qian_zui = "$GPRMC";

struct tm p = tm();

struct timeval tv;
struct timezone tz;

TIME tody;

void ros_detect_status()
{
    ros::Rate loop(10);
    while (ros::ok())
    {
        // ros::spinOnce();
        loop.sleep();
    }

    ros_ok = false;
}

void readAndSet(std::string& msg)
{
    if(msg.size() < 16) return;
    if(msg.substr(0, 6) == qian_zui)
    {
        // 设置系统时间
        p.tm_year = tody.year - 1900;
        p.tm_mon = tody.month - 1; 
        p.tm_mday = tody.day;
        p.tm_hour = (msg[7]-'0')*10 + (msg[8]-'0') + 8;
        p.tm_min = (msg[9]-'0')*10 + (msg[10]-'0');
        p.tm_sec = (msg[11]-'0')*10 + (msg[12]-'0');
        time_t utc_t = mktime(&p);

        tv.tv_sec = utc_t;
        tv.tv_usec = 0;

        if(settimeofday(&tv, &tz) < 0)
        {
            printf("setting error! \n");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpsSyneNode");

    ros::NodeHandle node;

    // 获取参数
    node.param("/gps/syne_SerialPort", serial_port, std::string("/dev/ttyUSB0"));
    node.param("/gps/syne_BaudRate", baudRate, 115200);// 波特率

    // 打开串口
    serial::Serial ser;
    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(baudRate);
        // 超时时间 -- 推测为非阻塞方式
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        ser.setTimeout(to);
        ser.open();
    }
    catch(serial::IOException& err)
    {
        ROS_ERROR("open port failed!!!");
        return 0;
    }

    if(ser.isOpen()) // 确保打开
        ROS_INFO("serial open successfully!");
    else
    {
        return 0;
    }

    std::thread ros_detect(ros_detect_status);

    ros_ok = true;

    size_t num = 0;
    gettimeofday (&tv , &tz);//获取时区保存tz中

    {
        time_t timep;
        struct tm* p;
        time(&timep);
        p = localtime(&timep); //获取UTC时间
        tody.year = 1900 + p->tm_year;
        tody.month = 1 +p->tm_mon;
        tody.day = p->tm_mday;
    }

    while (1)
    {
        if(num = ser.available()){
            std::string msg = ser.read(num);
            readAndSet(msg);
            // ROS_INFO_STREAM("msg: "<<msg);
        }

        usleep(500);

        if(!ros_ok) break;
    }

    ser.close();// 关闭串口
    
// while(ros::ok()){

    // TicToc t_set_time;
    // struct timeval tv;
    // struct timezone tz;
    // gettimeofday (&tv , &tz);//获取时区保存tz中

    // // 设置系统时间
    // struct tm p = tm();
    // p.tm_year = 2020 - 1900;
    // p.tm_mon = 3 - 1;
    // p.tm_mday = 5;
    // p.tm_hour = 3;
    // p.tm_min = 30;
    // p.tm_sec = 40;
    // time_t utc_t = mktime(&p);

    // tv.tv_sec = utc_t;
    // tv.tv_usec = 0;

    // if(settimeofday(&tv, &tz) < 0)
    // {
    //     printf("setting error! \n");
    // }

    // double td = t_set_time.Toc();    
    // printf("time:  %.6f\n", td);
// }
    return 0;
}