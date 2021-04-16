#include "gps_driver/gpsResolve.h"
#include "gps_driver/TicToc.h"

GPSResolve::GPSResolve(ros::NodeHandle& node_)
    : node(node_)
{
    pubGPS_m = node.advertise<gps_driver::GPCHC>("/huace/gpchc", 500); // 发布华测的融合数据
    pubGPS_s = node.advertise<sensor_msgs::NavSatFix>("/GPS/gps_chc", 500); // 发布标准的数据
    // pubGPS_gpgga = node.advertise<sensor_msgs::NavSatFix>("/GPGGA", 10); // 发布标准的数据
    pubIMU_filter = node.advertise<sensor_msgs::Imu>("/GPS/imu_data", 500); // 发布滤波后的IMU数据
    node.param("/gps/time_delay", time_delay, -18.0);
    node.param("/gps/frame_id", gps_frame_id, std::string("/gps_imu_baselink"));
}

GPSResolve::~GPSResolve()
{
    ;
}

// second to date
tm GPSResolve::sec2Date(const long& sec)
{
  time_t now(sec);
  return *(localtime(&now));
}

// date to string
std::string GPSResolve::date2Str(const tm& date)
{
  std::string str;
  str += std::to_string(1900+date.tm_year);
  str += '-';
  str += std::to_string(1 + date.tm_mon);
  str += '-';
  str += std::to_string(date.tm_mday);
  str += '-';
  str += std::to_string(date.tm_hour);
  str += '-';
  str += std::to_string(date.tm_min);
  str += '-';
  str += std::to_string(date.tm_sec);
  
  return str;
}

// UTC时间--时间转秒数，相对于1900.1.1 00:00:00
time_t GPSResolve::convert_str_to_tm(char * str_time)
{
    struct tm tt;
    memset(&tt,0,sizeof(tt));
    tt.tm_year=atoi(str_time)-1900;
    tt.tm_mon=atoi(str_time+5)-1;
    tt.tm_mday=atoi(str_time+8);
    tt.tm_hour=atoi(str_time+11);
    tt.tm_min=atoi(str_time+14);
    tt.tm_sec=atoi(str_time+17);
    return mktime(&tt);
    //28800是一个8小时，加上它得到北京时间
}

// 判断闰年
bool GPSResolve::is_leap_year(unsigned year)
{
	if (!(year % 400))
		return true;

	if (!(year % 100))
		return false;

	if (!(year % 4))
		return true;

	return false;
}

// 总体解析数据格式
void GPSResolve::ResolveGPSData(std::string& msg_)
{
    if(msg_.empty()) // 数据为空则返回
        return;
    if(flag_==GPSDataType::None && msg_.size()<6)
        return;
    msg_size = msg_.size();//统计总长度
    count_id = 0;
    if(flag_==GPSDataType::None) // 在新的起始时查找，防止错误发生
    {
        // 找起始标志
        start = msg_.find(gstart);
        if(start == -1) return;// 代表找不到
        count_id = start;
    }
    std::string whole_str;
    while (count_id < msg_size)
    {
        if(flag_==GPSDataType::None){// 开始的查找确定这里没有错误
            // 获取前7个字符
            std::string type_ = msg_.substr(count_id+1, 5);
            count_id += 7;
            if(type_=="GPCHC")
                flag_ = GPSDataType::GPCHC;
            else if(type_=="GPGGA")
                flag_ = GPSDataType::GPGGA;
            else if(type_=="GPRMC")
                flag_ = GPSDataType::GPRMC;
            else if(type_=="GPVTG")
                flag_ = GPSDataType::GPVTG;
            else if(type_=="GPXYZ")
                flag_ = GPSDataType::GPXYZ;
            else if(type_=="GPNEU")
                flag_ = GPSDataType::GPNEU;
            else // 超出范围
            {
                nonvalidNum++;
                printf("over predefine data type! - %d - \n", nonvalidNum);
                /*if(count_id >= msg_size)*/ ROS_ERROR_STREAM("counid:"<<count_id<<".msg size:"<<msg_size<<".first error"<<type_);
                if(count_id < msg_size) ROS_WARN_STREAM("kkk");
                break;
            }
        }
        // 找到结尾
        if(count_id >= msg_size) break;
        end = msg_.find(gend, count_id);
        if(end == -1)
        {
            cache_str.insert(cache_str.size(), msg_.substr(count_id));
            break;
        }
        else
        {
            cache_str.insert(cache_str.size(), msg_.substr(count_id, end-count_id+1));
            count_id = end +2;
            // std::cout<<"cache str: "<<cache_str<<'\n';
            // 对应的解析数据
            if(flag_ == GPSDataType::GPGGA)
            { // 先不用
                // GGA_data gga_data;
                // ResolveGGA(cache_str, gga_data);
                // if(gga_data.isValid)
                // {
                //     // printf("dta: %s\n", cache_str.c_str());
                //     // printf("gga: %d,%d,%lf,%lf,%lf,%d+++\n",
                //     //             gga_data.hour,
                //     //             gga_data.min,
                //     //             gga_data.sec,
                //     //             gga_data.lan,
                //     //             gga_data.lon,
                //     //             gga_data.state_);
                //     // double ros_time = ros::Time::now().toSec();
                //     // 标准数据
                //     ros_gpgga_msg.header.stamp = ros::Time::now();
                //     ros_gpgga_msg.header.frame_id = gps_frame_id;

                //     ros_gpgga_msg.latitude = gga_data.lan;
                //     ros_gpgga_msg.longitude = gga_data.lon;
                //     ros_gpgga_msg.altitude = 0.0;
                //     ros_gpgga_msg.status.status = 5;

                //     pubGPS_gpgga.publish(ros_gpgga_msg);
                // }
                // else
                // {
                //     printf("invalid data \n");
                // }
            }
            else if(flag_ == GPSDataType::GPCHC)
            {
                CHC_data chc_data;
                TicToc t_process_chc;
                ResolveCHC(cache_str, chc_data);
                if(chc_data.isValid)
                {
                    // printf("dta: %s\n", cache_str.c_str());
                    // printf("chc: %ld,%lf,%lf,%lf,%lf,%d+++\n",
                    //             chc_data.week,
                    //             chc_data.sec,
                    //             chc_data.lat,
                    //             chc_data.lon,
                    //             chc_data.alt,
                    //             chc_data.sat_status);
                    // 合成数据 -- 相对于UTC时间-18s, 然后在+8h变成北京时间
                    double timestamp = gps_base_time + chc_data.week*week_sec + chc_data.sec + time_diff_8h + time_delay;
                    // double ros_time = ros::Time::now().toSec();
                    // printf("gps time: %lf \nros time: %lf \n", timestamp, ros_time);
                    // std::string gps_date = date2Str(sec2Date(long(timestamp)));
                    // std::string ros_date = date2Str(sec2Date(long(ros_time)));
                    // std::cout<<"gps: "<<gps_date<<'\n'<<"ros: "<<ros_date<<'\n';

                    ros_chc_msg.header.frame_id = gps_frame_id;
                    ros_chc_msg.header.stamp = ros::Time().fromSec(timestamp); // 北京时间

                    ros_chc_msg.heading = chc_data.heading; // 姿态
                    ros_chc_msg.pitch = chc_data.pitch;
                    ros_chc_msg.roll = chc_data.roll;

                    ros_chc_msg.gyo_x = chc_data.gyo_x; // 陀螺仪角速率
                    ros_chc_msg.gyo_y = chc_data.gyo_y;
                    ros_chc_msg.gyo_z = chc_data.gyo_z;

                    ros_chc_msg.acc_x = chc_data.acc_x; // 加速度计线加速度
                    ros_chc_msg.acc_y = chc_data.acc_y;
                    ros_chc_msg.acc_z = chc_data.acc_z;

                    ros_chc_msg.lat = chc_data.lat; // 经纬度和高程
                    ros_chc_msg.lon = chc_data.lon;
                    ros_chc_msg.alt = chc_data.alt;

                    ros_chc_msg.NSV1 = chc_data.NSV1; // 卫星数量
                    ros_chc_msg.NSV2 = chc_data.NSV2;

                    ros_chc_msg.sat_status = chc_data.sat_status; // 卫星数据状态
                    ros_chc_msg.sys_status = chc_data.sys_status;
                    // 标准数据
                    ros_std_msg.header = ros_chc_msg.header; // 发布ros数据
                    ros_std_msg.latitude = ros_chc_msg.lat;
                    ros_std_msg.longitude = ros_chc_msg.lon;
                    ros_std_msg.altitude = ros_chc_msg.alt;
                    ros_std_msg.status.status = 0;

                    ros_imu_data.header = ros_chc_msg.header; // 发布滤波后的IMU数据
                    ros_imu_data.angular_velocity.x = ros_chc_msg.gyo_x;
                    ros_imu_data.angular_velocity.y = ros_chc_msg.gyo_y;
                    ros_imu_data.angular_velocity.z = ros_chc_msg.gyo_z;
                    ros_imu_data.linear_acceleration.x = ros_chc_msg.acc_x;
                    ros_imu_data.linear_acceleration.y = ros_chc_msg.acc_y;
                    ros_imu_data.linear_acceleration.z = ros_chc_msg.acc_z;
                    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(ros_chc_msg.roll, ros_chc_msg.pitch, ros_chc_msg.heading);
                    ros_imu_data.orientation.x = quat.x;
                    ros_imu_data.orientation.y = quat.y;
                    ros_imu_data.orientation.z = quat.z;
                    ros_imu_data.orientation.w = quat.w;
                    // 发布数据
                    pubGPS_m.publish(ros_chc_msg);
                    pubGPS_s.publish(ros_std_msg);
                    pubIMU_filter.publish(ros_imu_data);
                    if(t_process_chc.Toc() > 3.0)
                        ROS_WARN_STREAM("process time: "<< t_process_chc.Toc()<<" ms");
                    if(timestamp - last_time > 0.015)
                        ROS_WARN_STREAM("serial data lost");
                    last_time = timestamp;
                }
            }
            /* ** sth else ** */
            

            cache_str.clear();
            flag_ = GPSDataType::None;
        }
        
    }
}

// 总体解析数据格式
void GPSResolve::ResolveData(std::string& msg_)
{ // 处理思路是保证获取到完整的一帧再去解析
    if(msg_.empty()) // 数据为空则返回
        return;
// ROS_INFO_STREAM("msg:"<<msg_);
    msg_size = msg_.size();//统计总长度
    count_id = 0;

    if(!isInit){ // 初始帧处理
        end = msg_.find(gend, count_id);
        if(end == -1) return; // 等待结束符再开始

        count_id += 2;
        isInit = true;
        cache_str.clear();
    }
    if(isToFindEnd)
    {
        end = msg_.find(gend, count_id); // 再找结束符
        if(end != -1)
        {
            // std::cout<<"str0:"<<cache_str<<"\n";
            std::string tmp_str = cache_str;
            cache_str.insert(cache_str.size(), msg_.substr(count_id, end-count_id+2));
            count_id = end + 2;
            isToFindEnd = false;
            { // 解析--由于分隔符被隔断, 可能会包含两个串的情况, 从这里处理简单一点
                size_t id_s = cache_str.find(gstart);
                size_t id_e = cache_str.find(gend, id_s+5);
                if(id_s ==-1 || id_e==-1)
                {
                    ROS_ERROR("frame error");
                }
                { // 解析一次 
                    CHC_data chc_data;
                    TicToc t_process_chc;
                    ResolveCHC(cache_str.substr(id_s, id_e-id_s+2), chc_data);
                    if(chc_data.isValid)
                    {
                        double timestamp = gps_base_time + chc_data.week*week_sec + chc_data.sec + time_diff_8h + time_delay;
                        // double ros_time = ros::Time::now().toSec();
                        // printf("gps time: %lf \nros time: %lf \n", timestamp, ros_time);
                        // std::string gps_date = date2Str(sec2Date(long(timestamp)));
                        // std::string ros_date = date2Str(sec2Date(long(ros_time)));
                        // std::cout<<"gps: "<<gps_date<<'\n'<<"ros: "<<ros_date<<'\n';

                        ros_chc_msg.header.frame_id = gps_frame_id;
                        ros_chc_msg.header.stamp = ros::Time().fromSec(timestamp/*+ros_minus_gps_delay*/); // 北京时间

                        ros_chc_msg.heading = chc_data.heading; // 姿态
                        ros_chc_msg.pitch = chc_data.pitch;
                        ros_chc_msg.roll = chc_data.roll;

                        ros_chc_msg.gyo_x = chc_data.gyo_x; // 陀螺仪角速率
                        ros_chc_msg.gyo_y = chc_data.gyo_y;
                        ros_chc_msg.gyo_z = chc_data.gyo_z;

                        ros_chc_msg.acc_x = chc_data.acc_x; // 加速度计线加速度
                        ros_chc_msg.acc_y = chc_data.acc_y;
                        ros_chc_msg.acc_z = chc_data.acc_z;

                        ros_chc_msg.lat = chc_data.lat; // 经纬度和高程
                        ros_chc_msg.lon = chc_data.lon;
                        ros_chc_msg.alt = chc_data.alt;

                        ros_chc_msg.NSV1 = chc_data.NSV1; // 卫星数量
                        ros_chc_msg.NSV2 = chc_data.NSV2;

                        ros_chc_msg.sat_status = chc_data.sat_status; // 卫星数据状态
                        ros_chc_msg.sys_status = chc_data.sys_status;
                        // 标准数据
                        ros_std_msg.header = ros_chc_msg.header; // 发布ros数据
                        ros_std_msg.latitude = ros_chc_msg.lat;
                        ros_std_msg.longitude = ros_chc_msg.lon;
                        ros_std_msg.altitude = ros_chc_msg.alt;
                        ros_std_msg.status.status = int8_t(ros_chc_msg.sat_status);
                        ros_std_msg.status.service = ros_chc_msg.sys_status;
                        ros_std_msg.position_covariance[0] = ros_chc_msg.roll;
                        ros_std_msg.position_covariance[1] = ros_chc_msg.pitch;
                        ros_std_msg.position_covariance[2] = ros_chc_msg.heading;

                        ros_imu_data.header = ros_chc_msg.header; // 发布滤波后的IMU数据
                        ros_imu_data.angular_velocity.x = ros_chc_msg.gyo_x;
                        ros_imu_data.angular_velocity.y = ros_chc_msg.gyo_y;
                        ros_imu_data.angular_velocity.z = ros_chc_msg.gyo_z;
                        ros_imu_data.linear_acceleration.x = ros_chc_msg.acc_x;
                        ros_imu_data.linear_acceleration.y = ros_chc_msg.acc_y;
                        ros_imu_data.linear_acceleration.z = ros_chc_msg.acc_z;
                        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(ros_chc_msg.roll, ros_chc_msg.pitch, ros_chc_msg.heading);
                        ros_imu_data.orientation.x = quat.x;
                        ros_imu_data.orientation.y = quat.y;
                        ros_imu_data.orientation.z = quat.z;
                        ros_imu_data.orientation.w = quat.w;
                        // 发布数据
                        pubGPS_m.publish(ros_chc_msg);
                        pubGPS_s.publish(ros_std_msg);
                        pubIMU_filter.publish(ros_imu_data);
                        if(t_process_chc.Toc() > 3.0)
                            ROS_WARN_STREAM("process time: "<< t_process_chc.Toc()<<" ms");
                        if(timestamp - last_time > 0.015)
                            ROS_WARN_STREAM("serial data lost");
                        last_time = timestamp;
                    }
                }
                size_t id_s2 = cache_str.find(gstart, id_e+2);
                if(id_s2!=-1){ 
                    size_t id_e2 = cache_str.find(gend, id_s2);
                    if(id_s2!=-1)
                    {
                        CHC_data chc_data;
                        TicToc t_process_chc;
                        ResolveCHC(cache_str.substr(id_s2, id_e2-id_s2+2), chc_data);
                        if(chc_data.isValid)
                        {
                            double timestamp = gps_base_time + chc_data.week*week_sec + chc_data.sec + time_diff_8h + time_delay;
                            // double ros_time = ros::Time::now().toSec();
                            // printf("gps time: %lf \nros time: %lf \n", timestamp, ros_time);
                            // std::string gps_date = date2Str(sec2Date(long(timestamp)));
                            // std::string ros_date = date2Str(sec2Date(long(ros_time)));
                            // std::cout<<"gps: "<<gps_date<<'\n'<<"ros: "<<ros_date<<'\n';

                            ros_chc_msg.header.frame_id = gps_frame_id;
                            ros_chc_msg.header.stamp = ros::Time().fromSec(timestamp/*+ros_minus_gps_delay*/); // 北京时间

                            ros_chc_msg.heading = chc_data.heading; // 姿态
                            ros_chc_msg.pitch = chc_data.pitch;
                            ros_chc_msg.roll = chc_data.roll;

                            ros_chc_msg.gyo_x = chc_data.gyo_x; // 陀螺仪角速率
                            ros_chc_msg.gyo_y = chc_data.gyo_y;
                            ros_chc_msg.gyo_z = chc_data.gyo_z;

                            ros_chc_msg.acc_x = chc_data.acc_x; // 加速度计线加速度
                            ros_chc_msg.acc_y = chc_data.acc_y;
                            ros_chc_msg.acc_z = chc_data.acc_z;

                            ros_chc_msg.lat = chc_data.lat; // 经纬度和高程
                            ros_chc_msg.lon = chc_data.lon;
                            ros_chc_msg.alt = chc_data.alt;

                            ros_chc_msg.NSV1 = chc_data.NSV1; // 卫星数量
                            ros_chc_msg.NSV2 = chc_data.NSV2;

                            ros_chc_msg.sat_status = chc_data.sat_status; // 卫星数据状态
                            ros_chc_msg.sys_status = chc_data.sys_status;
                            // 标准数据
                            ros_std_msg.header = ros_chc_msg.header; // 发布ros数据
                            ros_std_msg.latitude = ros_chc_msg.lat;
                            ros_std_msg.longitude = ros_chc_msg.lon;
                            ros_std_msg.altitude = ros_chc_msg.alt;
                            ros_std_msg.status.status = int8_t(ros_chc_msg.sat_status);
                            ros_std_msg.status.service = ros_chc_msg.sys_status;
                            ros_std_msg.position_covariance[0] = ros_chc_msg.roll;
                            ros_std_msg.position_covariance[1] = ros_chc_msg.pitch;
                            ros_std_msg.position_covariance[2] = ros_chc_msg.heading;

                            ros_imu_data.header = ros_chc_msg.header; // 发布滤波后的IMU数据
                            ros_imu_data.angular_velocity.x = ros_chc_msg.gyo_x;
                            ros_imu_data.angular_velocity.y = ros_chc_msg.gyo_y;
                            ros_imu_data.angular_velocity.z = ros_chc_msg.gyo_z;
                            ros_imu_data.linear_acceleration.x = ros_chc_msg.acc_x;
                            ros_imu_data.linear_acceleration.y = ros_chc_msg.acc_y;
                            ros_imu_data.linear_acceleration.z = ros_chc_msg.acc_z;
                            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(ros_chc_msg.roll, ros_chc_msg.pitch, ros_chc_msg.heading);
                            ros_imu_data.orientation.x = quat.x;
                            ros_imu_data.orientation.y = quat.y;
                            ros_imu_data.orientation.z = quat.z;
                            ros_imu_data.orientation.w = quat.w;
                            // 发布数据
                            pubGPS_m.publish(ros_chc_msg);
                            pubGPS_s.publish(ros_std_msg);
                            pubIMU_filter.publish(ros_imu_data);
                            if(t_process_chc.Toc() > 3.0)
                                ROS_WARN_STREAM("process time: "<< t_process_chc.Toc()<<" ms");
                            if(timestamp - last_time > 0.015)
                                ROS_WARN_STREAM("serial data lost");
                            last_time = timestamp;
                        }
                    }
                    else
                    {
                        ROS_WARN_STREAM("frame error2");
                    }
                    
                }
                cache_str.clear();
            }

        }
        else // 结束符被分割开来, 也会找不到
        {
            ROS_WARN_STREAM("non find");
            cache_str.insert(cache_str.size(), msg_.substr(count_id));
            return;
        }
        
    }

    while(count_id < msg_size)
    {
        // 先找开头
        start = msg_.find(gstart, count_id);
        if(start != -1) // 找到了开头
        {
            count_id = start;
            end = msg_.find(gend, count_id); // 再找结束符
            if(end != -1)
            {
                // 表明是一帧完整的数据
                cache_str.insert(cache_str.size(), msg_.substr(count_id, end-count_id+2));
                count_id = end + 2; // 调到下一帧开头的位置
                { // 解析
                    size_t id_s = cache_str.find(gstart);
                    size_t id_e = cache_str.find(gend, id_s+5);
                    if(id_s ==-1 || id_e==-1)
                    {
                        ROS_ERROR("frame error");
                    }
                    size_t id_s2 = cache_str.find(gstart, id_e);
                    size_t id_e2 = cache_str.find(gend, id_s2);
                    if(id_s2!=-1 || id_e2!=-1){
                        ROS_ERROR("two flag find2");
                        ROS_WARN_STREAM("str2:"<<cache_str<<"\n");
                    }
                    
                    CHC_data chc_data;
                    TicToc t_process_chc;
                    ResolveCHC(cache_str.substr(id_s, id_e-id_s+2), chc_data);
                    if(chc_data.isValid)
                    {
                        double timestamp = gps_base_time + chc_data.week*week_sec + chc_data.sec + time_diff_8h + time_delay;
                        // double ros_time = ros::Time::now().toSec();
                        // printf("gps time: %lf \nros time: %lf \n", timestamp, ros_time);
                        // std::string gps_date = date2Str(sec2Date(long(timestamp)));
                        // std::string ros_date = date2Str(sec2Date(long(ros_time)));
                        // std::cout<<"gps: "<<gps_date<<'\n'<<"ros: "<<ros_date<<'\n';

                        // // 计算时间偏差
                        // if(!get_delay_ros_gps_time)
                        // {
                        //     ros_minus_gps_delay = ros_time_at_first_gps - timestamp;
                        //     get_delay_ros_gps_time = true;
                        // }
                        ros_chc_msg.header.frame_id = gps_frame_id;
                        ros_chc_msg.header.stamp = ros::Time().fromSec(timestamp/*+ros_minus_gps_delay*/); // 北京时间

                        ros_chc_msg.heading = chc_data.heading; // 姿态
                        ros_chc_msg.pitch = chc_data.pitch;
                        ros_chc_msg.roll = chc_data.roll;

                        ros_chc_msg.gyo_x = chc_data.gyo_x; // 陀螺仪角速率
                        ros_chc_msg.gyo_y = chc_data.gyo_y;
                        ros_chc_msg.gyo_z = chc_data.gyo_z;

                        ros_chc_msg.acc_x = chc_data.acc_x; // 加速度计线加速度
                        ros_chc_msg.acc_y = chc_data.acc_y;
                        ros_chc_msg.acc_z = chc_data.acc_z;

                        ros_chc_msg.lat = chc_data.lat; // 经纬度和高程
                        ros_chc_msg.lon = chc_data.lon;
                        ros_chc_msg.alt = chc_data.alt;

                        ros_chc_msg.NSV1 = chc_data.NSV1; // 卫星数量
                        ros_chc_msg.NSV2 = chc_data.NSV2;

                        ros_chc_msg.sat_status = chc_data.sat_status; // 卫星数据状态
                        ros_chc_msg.sys_status = chc_data.sys_status;
                        // 标准数据
                        ros_std_msg.header = ros_chc_msg.header; // 发布ros数据
                        ros_std_msg.latitude = ros_chc_msg.lat;
                        ros_std_msg.longitude = ros_chc_msg.lon;
                        ros_std_msg.altitude = ros_chc_msg.alt;
                        ros_std_msg.status.status = int8_t(ros_chc_msg.sat_status);
                        ros_std_msg.status.service = ros_chc_msg.sys_status;
                        ros_std_msg.position_covariance[0] = ros_chc_msg.roll;
                        ros_std_msg.position_covariance[1] = ros_chc_msg.pitch;
                        ros_std_msg.position_covariance[2] = ros_chc_msg.heading;

                        ros_imu_data.header = ros_chc_msg.header; // 发布滤波后的IMU数据
                        ros_imu_data.angular_velocity.x = ros_chc_msg.gyo_x;
                        ros_imu_data.angular_velocity.y = ros_chc_msg.gyo_y;
                        ros_imu_data.angular_velocity.z = ros_chc_msg.gyo_z;
                        ros_imu_data.linear_acceleration.x = ros_chc_msg.acc_x;
                        ros_imu_data.linear_acceleration.y = ros_chc_msg.acc_y;
                        ros_imu_data.linear_acceleration.z = ros_chc_msg.acc_z;
                        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(ros_chc_msg.roll, ros_chc_msg.pitch, ros_chc_msg.heading);
                        ros_imu_data.orientation.x = quat.x;
                        ros_imu_data.orientation.y = quat.y;
                        ros_imu_data.orientation.z = quat.z;
                        ros_imu_data.orientation.w = quat.w;
                        // 发布数据
                        pubGPS_m.publish(ros_chc_msg);
                        pubGPS_s.publish(ros_std_msg);
                        pubIMU_filter.publish(ros_imu_data);
                        if(t_process_chc.Toc() > 3.0)
                            ROS_WARN_STREAM("process time: "<< t_process_chc.Toc()<<" ms");
                        if(timestamp - last_time > 0.015)
                            ROS_WARN_STREAM("serial data lost");
                        last_time = timestamp;
                    }
                    cache_str.clear();
                }
            }
            else
            { // 找到开头找不到结尾就缓存起来
                cache_str.insert(cache_str.size(), msg_.substr(count_id));
                isToFindEnd = true;// 记录一下---下一次找结尾
                break;
            }
            
        }
        else // 找不到开头
        {
            cache_str.insert(cache_str.size(), msg_.substr(count_id));
            isToFindEnd = true;// 记录一下---下一次找结尾
            break;
        }
        
    }

    // if(flag_==GPSDataType::None) // 在新的起始时查找，防止错误发生
    // {
    //     // 找起始标志
    //     start = msg_.find(gstart);
    //     if(start == -1) return;// 代表找不到
    //     count_id = start;
    // }
    // std::string whole_str;
    // while (count_id < msg_size)
    // {
    //     if(flag_==GPSDataType::None){// 开始的查找确定这里没有错误
    //         // 获取前7个字符
    //         std::string type_ = msg_.substr(count_id+1, 5);
    //         count_id += 7;
    //         if(type_=="GPCHC")
    //             flag_ = GPSDataType::GPCHC;
    //         else if(type_=="GPGGA")
    //             flag_ = GPSDataType::GPGGA;
    //         else if(type_=="GPRMC")
    //             flag_ = GPSDataType::GPRMC;
    //         else if(type_=="GPVTG")
    //             flag_ = GPSDataType::GPVTG;
    //         else if(type_=="GPXYZ")
    //             flag_ = GPSDataType::GPXYZ;
    //         else if(type_=="GPNEU")
    //             flag_ = GPSDataType::GPNEU;
    //         else // 超出范围
    //         {
    //             nonvalidNum++;
    //             printf("over predefine data type! - %d - \n", nonvalidNum);
    //             /*if(count_id >= msg_size)*/ ROS_ERROR_STREAM("counid:"<<count_id<<".msg size:"<<msg_size<<".first error"<<type_);
    //             if(count_id < msg_size) ROS_WARN_STREAM("kkk");
    //             break;
    //         }
    //     }
    //     // 找到结尾
    //     if(count_id >= msg_size) break;
    //     end = msg_.find(gend, count_id);
    //     if(end == -1)
    //     {
    //         cache_str.insert(cache_str.size(), msg_.substr(count_id));
    //         break;
    //     }
    //     else
    //     {
    //         cache_str.insert(cache_str.size(), msg_.substr(count_id, end-count_id+1));
    //         count_id = end +2;
    //         // std::cout<<"cache str: "<<cache_str<<'\n';
    //         // 对应的解析数据
    //         if(flag_ == GPSDataType::GPGGA)
    //         { // 先不用
    //             // GGA_data gga_data;
    //             // ResolveGGA(cache_str, gga_data);
    //             // if(gga_data.isValid)
    //             // {
    //             //     // printf("dta: %s\n", cache_str.c_str());
    //             //     // printf("gga: %d,%d,%lf,%lf,%lf,%d+++\n",
    //             //     //             gga_data.hour,
    //             //     //             gga_data.min,
    //             //     //             gga_data.sec,
    //             //     //             gga_data.lan,
    //             //     //             gga_data.lon,
    //             //     //             gga_data.state_);
    //             //     // double ros_time = ros::Time::now().toSec();
    //             //     // 标准数据
    //             //     ros_gpgga_msg.header.stamp = ros::Time::now();
    //             //     ros_gpgga_msg.header.frame_id = gps_frame_id;

    //             //     ros_gpgga_msg.latitude = gga_data.lan;
    //             //     ros_gpgga_msg.longitude = gga_data.lon;
    //             //     ros_gpgga_msg.altitude = 0.0;
    //             //     ros_gpgga_msg.status.status = 5;

    //             //     pubGPS_gpgga.publish(ros_gpgga_msg);
    //             // }
    //             // else
    //             // {
    //             //     printf("invalid data \n");
    //             // }
    //         }
    //         else if(flag_ == GPSDataType::GPCHC)
    //         {
    //             CHC_data chc_data;
    //             TicToc t_process_chc;
    //             ResolveCHC(cache_str, chc_data);
    //             if(chc_data.isValid)
    //             {
    //                 // printf("dta: %s\n", cache_str.c_str());
    //                 // printf("chc: %ld,%lf,%lf,%lf,%lf,%d+++\n",
    //                 //             chc_data.week,
    //                 //             chc_data.sec,
    //                 //             chc_data.lat,
    //                 //             chc_data.lon,
    //                 //             chc_data.alt,
    //                 //             chc_data.sat_status);
    //                 // 合成数据 -- 相对于UTC时间-18s, 然后在+8h变成北京时间
    //                 double timestamp = gps_base_time + chc_data.week*week_sec + chc_data.sec + time_diff_8h + time_delay;
    //                 // double ros_time = ros::Time::now().toSec();
    //                 // printf("gps time: %lf \nros time: %lf \n", timestamp, ros_time);
    //                 // std::string gps_date = date2Str(sec2Date(long(timestamp)));
    //                 // std::string ros_date = date2Str(sec2Date(long(ros_time)));
    //                 // std::cout<<"gps: "<<gps_date<<'\n'<<"ros: "<<ros_date<<'\n';

    //                 ros_chc_msg.header.frame_id = gps_frame_id;
    //                 ros_chc_msg.header.stamp = ros::Time().fromSec(timestamp); // 北京时间

    //                 ros_chc_msg.heading = chc_data.heading; // 姿态
    //                 ros_chc_msg.pitch = chc_data.pitch;
    //                 ros_chc_msg.roll = chc_data.roll;

    //                 ros_chc_msg.gyo_x = chc_data.gyo_x; // 陀螺仪角速率
    //                 ros_chc_msg.gyo_y = chc_data.gyo_y;
    //                 ros_chc_msg.gyo_z = chc_data.gyo_z;

    //                 ros_chc_msg.acc_x = chc_data.acc_x; // 加速度计线加速度
    //                 ros_chc_msg.acc_y = chc_data.acc_y;
    //                 ros_chc_msg.acc_z = chc_data.acc_z;

    //                 ros_chc_msg.lat = chc_data.lat; // 经纬度和高程
    //                 ros_chc_msg.lon = chc_data.lon;
    //                 ros_chc_msg.alt = chc_data.alt;

    //                 ros_chc_msg.NSV1 = chc_data.NSV1; // 卫星数量
    //                 ros_chc_msg.NSV2 = chc_data.NSV2;

    //                 ros_chc_msg.sat_status = chc_data.sat_status; // 卫星数据状态
    //                 ros_chc_msg.sys_status = chc_data.sys_status;
    //                 // 标准数据
    //                 ros_std_msg.header = ros_chc_msg.header; // 发布ros数据
    //                 ros_std_msg.latitude = ros_chc_msg.lat;
    //                 ros_std_msg.longitude = ros_chc_msg.lon;
    //                 ros_std_msg.altitude = ros_chc_msg.alt;
    //                 ros_std_msg.status.status = 0;

    //                 ros_imu_data.header = ros_chc_msg.header; // 发布滤波后的IMU数据
    //                 ros_imu_data.angular_velocity.x = ros_chc_msg.gyo_x;
    //                 ros_imu_data.angular_velocity.y = ros_chc_msg.gyo_y;
    //                 ros_imu_data.angular_velocity.z = ros_chc_msg.gyo_z;
    //                 ros_imu_data.linear_acceleration.x = ros_chc_msg.acc_x;
    //                 ros_imu_data.linear_acceleration.y = ros_chc_msg.acc_y;
    //                 ros_imu_data.linear_acceleration.z = ros_chc_msg.acc_z;
    //                 geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(ros_chc_msg.roll, ros_chc_msg.pitch, ros_chc_msg.heading);
    //                 ros_imu_data.orientation.x = quat.x;
    //                 ros_imu_data.orientation.y = quat.y;
    //                 ros_imu_data.orientation.z = quat.z;
    //                 ros_imu_data.orientation.w = quat.w;
    //                 // 发布数据
    //                 pubGPS_m.publish(ros_chc_msg);
    //                 pubGPS_s.publish(ros_std_msg);
    //                 pubIMU_filter.publish(ros_imu_data);
    //                 if(t_process_chc.Toc() > 3.0)
    //                     ROS_WARN_STREAM("process time: "<< t_process_chc.Toc()<<" ms");
    //                 if(timestamp - last_time > 0.015)
    //                     ROS_WARN_STREAM("serial data lost");
    //                 last_time = timestamp;
    //             }
    //         }
    //         /* ** sth else ** */
            

    //         cache_str.clear();
    //         flag_ = GPSDataType::None;
    //     }
        
    // }
}

// 解析GGA数据
void GPSResolve::ResolveGGA(std::string& data_, GGA_data& gga_data)
{
    // printf("dta: %s\n", data_.c_str());
    // 根据逗号分割
    size_t count_ = 0;
    int comma_pos = data_.find(",");
    if(data_.substr(count_, comma_pos-count_).empty())
    {
        count_ = comma_pos+1;
        comma_pos = data_.find(",", count_);
    }
    // tmp_str.clear();
    // 提取时间，这个必有
    {   // 时
        std::stringstream ss;
        ss << data_.substr(count_, 2); count_ += 2;
        ss >> gga_data.hour;
        // printf("hour: %d, ", gga_data.hour);
    }
    {   // 分
        std::stringstream ss;
        ss << data_.substr(count_, 2); count_ += 2;
        ss >> gga_data.min;
        // printf("min: %d, ", gga_data.min);
    }
    {   // 秒
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> gga_data.sec;
        // printf("sec: %lf, ", gga_data.sec);
    }

    // 提取纬度
    comma_pos = data_.find(',', count_);
    if(comma_pos == int(count_))
    {
        gga_data.isValid = false;
        return; // 无效数据
    }
    int tmp_ = 0;
    { // 度
        std::stringstream ss;
        ss << data_.substr(count_, 2); count_ += 2;
        ss >> tmp_;
    }
    { // 分
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> gga_data.lan;
    }
    gga_data.lan = gga_data.lan/60.0 + tmp_;
    gga_data.lan = (data_[count_] == 'N'? gga_data.lan : -gga_data.lan);
    count_ += 2;

    // 提取经度
    comma_pos = data_.find(',', count_);
    {   // 度
        std::stringstream ss;
        ss << data_.substr(count_, 3); count_ += 3;
        ss >> tmp_;
    }
    { // 分
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> gga_data.lon;
    }
    gga_data.lon = gga_data.lon/60.0 + tmp_;
    gga_data.lon = (data_[count_]=='E'? gga_data.lon : -gga_data.lon);
    count_ += 2;

    // 解的质量
    if(data_[count_]>'9' || data_[count_]<'0')
    {
        gga_data.isValid = false;
        return;
    }
    gga_data.state_ = GPSState_GGA( data_[count_]-'0' );
    gga_data.isValid = true;
}

// 解析CHC数据
void GPSResolve::ResolveCHC(const std::string& data_, CHC_data& chc_data)
{

    // 根据逗号分割
    size_t count_ = 0;
    int comma_pos = data_.find(",", count_);
    if(data_.substr(count_, comma_pos-count_) != "$GPCHC")
    {
        ROS_WARN_STREAM("non valid frame chc");
        return;
    }

    count_ = comma_pos + 1;
    comma_pos = data_.find(",", count_);

    // if(data_.substr(count_, comma_pos-count_).empty())
    // {
    //     count_ = comma_pos+1;
    //     comma_pos = data_.find(",", count_);
    // }

    // tmp_str.clear();
    // 提取GPS week
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.week;
    }
    // 提取秒数
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.sec;
    }
    // 提取航向角
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.heading;
        chc_data.heading = chc_data.heading * deg2Rad_factor; // 弧度
        // 调整与IMU标识一致--东北天系--航向与谁的夹角不是夹角
        chc_data.heading = /*M_PI/2.0*/ - chc_data.heading;
        while (chc_data.heading > M_PI)
            chc_data.heading -= 2.0*M_PI;
        while (chc_data.heading < -M_PI)
            chc_data.heading += 2.0*M_PI;
    }
    // 提取俯仰角
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.pitch;
        chc_data.pitch = chc_data.pitch * deg2Rad_factor; // 弧度
        while (chc_data.pitch > M_PI)
            chc_data.pitch -= 2.0*M_PI;
        while (chc_data.pitch < -M_PI)
            chc_data.pitch += 2.0*M_PI;
    }
    // 提取横滚角
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.roll;
        chc_data.roll = chc_data.roll * deg2Rad_factor; // 弧度
        while (chc_data.roll > M_PI)
            chc_data.roll -= 2.0*M_PI;
        while (chc_data.roll < -M_PI)
            chc_data.roll += 2.0*M_PI;
    }

    // { // ---调整与IMU标识一致--东北天系--这写错了--忽略
    //     double tmp_swap = chc_data.pitch;
    //     chc_data.pitch = chc_data.roll;
    //     chc_data.roll = tmp_swap;
    // }

    // 提取陀螺角速度
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.gyo_x;
        chc_data.gyo_x *= deg2Rad_factor;
    }
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.gyo_y;
        chc_data.gyo_y *= deg2Rad_factor;
    }
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.gyo_z;
        chc_data.gyo_z *= deg2Rad_factor;
    }
    // 提取加速度
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.acc_x;
        chc_data.acc_x *= G_gps;
    }
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.acc_y;
        chc_data.acc_y *= G_gps;
    }
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.acc_z;
        chc_data.acc_z *= G_gps;
    }
    // 纬度
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.lat;
    }
    // 经度
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.lon;
    }
    // 高程
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.alt;
    }
    // ENU速度
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.v_e;
    }
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.v_n;
    }
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.v_u;
    }
    // 车辆速度
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.v;
    }
    // NSV1卫星数
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.NSV1;
    }
    // NSV2卫星数
    comma_pos = data_.find(',', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.NSV2;
    }
    // 卫星及系统状态
    comma_pos = data_.find(',', count_);
    chc_data.sat_status = uint16_t(data_[count_] - '0');
    chc_data.sys_status = uint16_t(data_[count_+1] - '0');
    count_ = comma_pos+1;
    // 差分延时--忽略
    comma_pos = data_.find(',', count_); count_ = comma_pos+1;
    // warming
    comma_pos = data_.find('*', count_);
    {
        std::stringstream ss;
        ss << data_.substr(count_, comma_pos-count_); count_ = comma_pos+1;
        ss >> chc_data.warming;
    }
    { // 校验位
        count_ += 2;
    }
    { // 固定包尾
        if(data_.substr(count_, 2) != "\r\n")
        {
            chc_data.isValid = false;
            ROS_WARN_STREAM("non valid frame fixed end flag");
            return;
        }

    }
    // // 判断是否有效
    // if( (chc_data.warming&0x01)==0x01 || 
    //     (chc_data.warming&0x03)==0x03 || (chc_data.warming&0x04)==0x04)
    // {
    //     chc_data.isValid = false;
    //     return;
    // }
    chc_data.isValid = true;
}

/* ****************************以下为主函数****************************** */
std::string serial_port; // 串口名
int baudRate; // 波特率
std::string msg_in; // 串口读入的数据

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpsResolving_node");
    ros::NodeHandle node;

    GPSResolve gps_resolve(node);

    // char str_2[64]="1980-01-06 00:00:00";
    // // // printf("ros time: %lf \n", ros::Time::now().toSec());
    // long gps_base_time = gps_resolve.convert_str_to_tm(str_2);
    // // printf("gps base time: %ld\n", gps_base_time);
    // long gps_time = 2114*7*24*3600+461302;
    // printf("gps sec: %ld\n", gps_time+gps_base_time);
    // std::string str__ = gps_resolve.date2Str(gps_resolve.sec2Date(gps_base_time+gps_time));
    // std::cout<<"gps time: "<< str__ << '\n';

    // 获取参数
    node.param("/gps/SerialPort", serial_port, std::string("/dev/ttyUSB0"));
    node.param("/gps/BaudRate", baudRate, 230400);// 波特率

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

    // 循环
    ros::Rate loop_(200); // 数据最快是100hz
    while (ros::ok())
    {
        if(ser.available())
        {
            // 读取数据---数据非完整
            // double start_ = ros::Time().now().toSec();
            if(!gps_resolve.isInit)
                gps_resolve.ros_time_at_first_gps = ros::Time().now().toSec();
            msg_in = ser.read(ser.available());
            // printf("msg: %s\n", msg_in.c_str());
            // gps_resolve.ResolveGPSData(msg_in);
            gps_resolve.ResolveData(msg_in);
            // std::cout<<"cum: "<< ros::Time().now().toSec()-start_ << '\n';
        }
        // else
        // {
        //     ROS_ERROR("device was removed!");
        //     break;
        // }
        
        ros::spinOnce();
        loop_.sleep();
    }
    
    ser.close();// 关闭串口

    // // char str_[64]="2020-07-17 21:22:59";
    
    // // long ros_time = ros::Time::now().toSec();
    // // std::string str__ = date2Str(sec2Date(gps_base_time));
    
    // // printf("self: %ld \n", convert_str_to_tm(str_) + convert_str_to_tm(str_2));

    return 0;
}