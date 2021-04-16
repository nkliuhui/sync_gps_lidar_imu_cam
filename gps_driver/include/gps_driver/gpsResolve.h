/* 这是基于ros serial库编写的用于解析"华测GPS"的程序 */

#include <ros/ros.h>
#include "gps_driver/GPCHC.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "tf/transform_datatypes.h"
#include <string>
#include <sstream>
#include <serial/serial.h>
// 解析时间--计算pc和gps的时间延迟
#include <ctime>

#include <time.h>
#include <string.h>
#include <stdlib.h> 

#define G_gps 9.80665 // 重力加速度标准值

class GPSResolve
{
    /* 定义所需数据类型 */
    enum GPSDataType
    {// 标记剩余的字符属于哪种数据格式--因为数据接收不一定是一帧完整的
        None = 0, // 一帧数据接收完成
        GPCHC = 1,
        GPGGA, // 定好基准自动编号
        GPRMC,
        GPVTG,
        GPXYZ,
        GPNEU
    };

    enum GPSState_GGA
    {// 定义GPS_GGA状态
        unLoc = 0, //未定位
        single_point, //单点
        SBAS, // 伪距/SBAS
        invalid_PPS, // 无效PPS
        RTK_solid, // RTK 固定解
        RTK_float, // RTK 浮动解
        estimating, // 正在估算
        wtf, // 手动启动基准站
        RTK_kuanxiang, // RTK宽巷解
        weiju // 伪距(诺瓦泰615)
    };

    struct GGA_data
    {// 定义GGA数据格式
        // UTC时间，时分秒, 需要+8才转换成北京时间
        int hour;
        int min;
        double sec;
        // 纬度
        double lan;
        // 经度
        double lon;
        // GPS状态
        GPSState_GGA state_;
        // 数据是否有效
        bool isValid = false;
    };

    struct CHC_data
    {// 定义CHC数据格式
        // GPS周计时法
        long week; // 相对于1980.1.6(周日)的整周数
        double sec; // 相对于本周日(国外意义上的周日)的秒数，一周起始
        double heading; // 航向角--弧度
        double pitch; // 俯仰角
        double roll; // 横滚角
        double gyo_x; // 陀螺角速度
        double gyo_y;
        double gyo_z;
        double acc_x; // 加速度计
        double acc_y;
        double acc_z;
        double lat; // 纬度
        double lon; // 经度
        double alt; // 高程
        double v_e; // 东向速度
        double v_n; // 北向速度
        double v_u; // 天向速度
        double v; // 车辆速度
        long NSV1; // 主天线1的卫星数
        long NSV2; // 主天线2的卫星数
        // 状态分两个半字节
        uint16_t sat_status; // 卫星状态
        uint16_t sys_status; // 系统状态
        // 差分延时
        // ---略过
        // bit0-GPS消息 bit1-车辆信息 bit2-陀螺错误 bit3-加表错误 0表示正常
        uint16_t warming; // 反映系统状态？？？--使用 “位与” 的方式可判断
        // 校验
        // ---略过
        // 制表符回车
        bool isValid = false; // 是否有效 
        double timestamp = 0.0;
    };
  /* 构造函数与析构函数 */
  public:
    GPSResolve(ros::NodeHandle& node);
    ~GPSResolve();

  /* 方法 */
  public:
    /************* 函数声明 ***************/
        // 解析数据采取总分结构进行--即先判断是那种数据，然后对应处理
    tm sec2Date(const long& sec);
    std::string date2Str(const tm& date);
    time_t convert_str_to_tm(char * str_time);
    bool is_leap_year(unsigned year);

    void ResolveGPSData(std::string& msg_); // 总体处理
    void ResolveData(std::string& msg_);
    void ResolveGGA(std::string& data_, GGA_data& gga_data); // 处理GGA
    void ResolveCHC(const std::string& data_, CHC_data& chc_data); // 处理CHC

  /* 变量 */
  public:
    std::string gstart = "$GP";
    std::string gend = "\r\n";
    size_t msg_size = 0;
    size_t count_id = 0;
    int start = -1;
    int end = -1;
    std::string cache_str;
    // std::string tmp_str;

    // 因为串口的缘故导致其不太完整，过长或过短
    bool read_GGA_done = false;
    GPSDataType flag_ = GPSDataType::None;

    // UTC时间比北京时间晚8小时
    long time_diff_8h = 28800;
    // GPS以周为单位计时--相对于1980年1月6日
    long gps_base_time = 315936000; // 1980.1.6相对于1900.1.1的秒数 00:00:00
    // 角度转弧度的因子项
    double deg2Rad_factor = M_PI/180.0;
    // 一周的秒数
    long week_sec = 604800;
    // gps相对于网络时间延迟 (典型值18s, gps快)
    double time_delay;
    // 计算GPS与当前系统的时间偏差--统一到当前系统时间下
    double ros_minus_gps_delay = 0.0; // 单位: 秒
    // 记录第一帧GPS到达时的系统时间
    double ros_time_at_first_gps = 0.0;
    // 计算出了延时的标志位
    bool get_delay_ros_gps_time = false;

    // ros项
    ros::Publisher pubGPS_m; // 自定义的数据
    ros::Publisher pubGPS_s; // 发布标准数据
    ros::Publisher pubGPS_gpgga; // 发布标准数据
    ros::Publisher pubIMU_filter; // 发布滤波后的IMU数据
    ros::NodeHandle node;
    gps_driver::GPCHC ros_chc_msg;
    sensor_msgs::NavSatFix ros_std_msg;
    sensor_msgs::NavSatFix ros_gpgga_msg;
    sensor_msgs::Imu ros_imu_data;
    std::string gps_frame_id; // GPS输出信息相对于坐标系id

    double last_time = 0.0;
    size_t nonvalidNum = 0;
    bool isInit = false;
    bool isToFindEnd = false;
    bool getNewRosTime = true;
};