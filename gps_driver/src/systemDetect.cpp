#include <ros/ros.h>
// #include <Eigen/Eigen>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

bool timestamp_align = false;
bool gps_device_init = false;

double time_lidar = 0.0;
double time_gps = -200.0;

int sys_status = -1;
int sat_status = -1;

double time_delay = 0.0;

double time_delay_vec[20];
double delta_time = 0.5;
int count_ = 0;

double last_gps_time = 0.0;
bool isInit = false;

bool global_init_flag = false;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gpsIn)
{
    time_gps = gpsIn->header.stamp.toSec();
    double time_ros = ros::Time::now().toSec();
    time_delay = time_ros - time_gps; // 软时间对齐

    sys_status = gpsIn->status.service; // 系统状态
    sat_status = gpsIn->status.status; // 解的状态

    if(!isInit) {
        last_gps_time = time_gps;
        isInit = true;
    }
    if(time_gps - last_gps_time > 1.0 && time_delay<10.0 && isInit && gps_device_init)
    {// 这个不对, ros传输不需要时间吗....
        time_delay_vec[(count_%20)] = time_delay;
        count_++;
        // ROS_WARN_STREAM("herre");
        last_gps_time = time_gps;
    }
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& pointsIn)
{
    time_lidar = pointsIn->header.stamp.toSec();
}

void RevInitCmd(const std_msgs::Int32ConstPtr& msgIn)
{
    global_init_flag = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_detect_node");
    ros::NodeHandle node;

    ros::Subscriber subGPS = node.subscribe<sensor_msgs::NavSatFix>("/GPS/gps_chc", 1, gpsCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subLidar = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, lidarCallback, ros::TransportHints().tcpNoDelay());
    // 定义server--等待接收按钮指令
    ros::Subscriber sub_init_button = node.subscribe<std_msgs::Int32>("/InitSystem", 2, RevInitCmd);
    ros::Rate loop_(200);
    while (ros::ok())
    {
        ros::spinOnce();
        if(!global_init_flag)
        { // 如果未初始化, 就卡着
            loop_.sleep();
            continue;
        }
        // 接收到按钮
        if(fabs(time_lidar-time_gps) < 0.5 && !timestamp_align)
        {
            ROS_INFO("timestamp aligned!!!");
            timestamp_align = true;
        }
        if(sys_status == 2 && sat_status == 4 && !gps_device_init)
        {
            ROS_INFO("system ok and fix resolve----");
            gps_device_init = true;
        }

        if(timestamp_align && gps_device_init && count_>10)
        {
            int num_ = count_>20? 20:count_;
            double sum_ = 0;
            for(int i=0; i<num_; ++i)
                sum_ += time_delay_vec[i];
            double ave_time_delay = sum_ / num_;
            ROS_INFO("system is ok, time delay: %lf s..", ave_time_delay);
            ros::param::set("/ros_minue_gps_time", ave_time_delay);
            ros::shutdown();
        }
        loop_.sleep();
    }

    return 0;
}
