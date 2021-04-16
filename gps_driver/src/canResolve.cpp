// 在此基础上进行修改

//  pcanread.cpp
//
//  ~~~~~~~~~~~~
//
//  PCANBasic Example: Simple Read
//
//  ~~~~~~~~~~~~
//
//  ------------------------------------------------------------------
//  Author : Thomas Haber (thomas@toem.de)
//  Last change: 18.06.2010
//
//  Language: C++
//  ------------------------------------------------------------------
//
//  Copyright (C) 1999-2010  PEAK-System Technik GmbH, Darmstadt
//  more Info at http://www.peak-system.com
//  ------------------------------------------------------------------
//
// linux@peak-system.com
// www.peak-system.com
//
//  ------------------------------------------------------------------
//  History:
//  07-11-2013 Stephane Grosjean
//  - Move DWORD definition from "unsigned long" to "__u32" to run on 64-bits
//    Kernel
//  - Change initital bitrate from 250K to 500K (default pcan driver bitrate)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//

#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <asm/types.h>
#include <iostream>

#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*

#include "PCANBasic.h" // 这个经过修改, 将其中的函数声明使用extern重新声明了

#include <string>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "gps_driver/IMURaw.h"

#include "gps_driver/TicToc.h"

#define G_gps 9.80665 // 重力加速度标准值

struct RawCanData
{
    typedef double float64;
    typedef int int32;

    double timestamp; // 时间

    float64 raw_gyo_x; // # 角速率
    float64 raw_gyo_y;
    float64 raw_gyo_z;

    float64 raw_acc_x; // # 线加速度
    float64 raw_acc_y;
    float64 raw_acc_z;

    int32 sys_state; // # system state
    int32 gps_num_stats_used; // # 主天线使用的卫星数
    int32 sate_stuts; // # 卫星状态
    int32 gps_num_stats2_used; // # 辅天线使用的卫星数
    float64 gps_age; // # 差分延时
    int32 gps_num_sats; // # 主天线搜星数
    int32 gps_num_sats2; // # 辅天线搜星数

    float64 latitude; // # 原始经纬度和大地高
    float64 lontitude;
    float64 altitude;

    float64 pose_E_sigma; // # 东北天 位置 sigma值 标准差?
    float64 pose_N_sigma;
    float64 pose_U_sigma;

    float64 vel_E; // # 东北天速度
    float64 vel_N;
    float64 vel_U;
    float64 vel_body; // # 车体速度

    float64 vel_E_sigma; // # 东北天 速度 sigma值 标准差?
    float64 vel_N_sigma;
    float64 vel_U_sigma;
    float64 vel_body_sigma; 

    float64 acc_X; // # 车辆坐标系加速度
    float64 acc_Y;
    float64 acc_Z;

    float64 angle_heading; // # 车辆姿态 unit: rad
    float64 angle_pitch;
    float64 angle_roll;

    float64 angle_heading_sigma; // # 车辆姿态 unit: rad
    float64 angle_pitch_sigma;
    float64 angle_roll_sigma;

    float64 ang_rate_X; // # 车辆坐标系角速度 unit: rad/s
    float64 ang_rate_Y;
    float64 ang_rate_Z;
    
    friend std::ostream & operator<<( std::ostream & os,const RawCanData& data)
    {
        os << "gyo: " << data.raw_gyo_x <<" ,"<< data.raw_gyo_y <<" ," << data.raw_gyo_z <<" ;\n"
           << "acc: " << data.raw_acc_x << " ,"<< data.raw_acc_y << " ,"<< data.raw_acc_z << " ;\n"
           << "sys state: "<< data.sys_state << " , sat_num1: "<<data.gps_num_stats_used<<" ,sate_status: "<< data.sate_stuts
           <<" , sat_num2: "<< data.gps_num_stats2_used << " , gps_age: "<<data.gps_age <<" , gps_num_sat1: "<<data.gps_num_sats
           <<" , gps_num_sat2: "<<data.gps_num_sats2<<" \n"
           << "lla: "<< data.latitude << " ,"<< data.lontitude << " ,"<< data.altitude << " ;\n"
           << "pose sigma: "<< data.pose_E_sigma << " ," << data.pose_N_sigma<< " ," <<data.pose_U_sigma << " ;\n"
           << "vel: "<< data.vel_E <<" ," <<data.vel_N <<" ," << data.vel_U <<" ,"<< data.vel_body <<" ;\n"
           << "vel sigma: "<< data.vel_E_sigma << " ,"<<data.vel_N_sigma <<" ," << data.vel_U_sigma << " ,"<< data.vel_body_sigma <<" ;\n"
           << "acc_XYZ: "<< data.acc_X <<" ," << data.acc_Y <<" "<<data.acc_Z << " ;\n"
           << "ang_ori: "<< data.angle_heading <<" ," << data.angle_pitch <<" ,"<<data.angle_roll <<" ;\n"
           << "ang_ori sigma: "<< data.angle_heading_sigma <<" ," << data.angle_pitch_sigma <<" ,"<< data.angle_roll_sigma <<" ;\n"
           << "ang rate: "<< data.ang_rate_X <<" ," <<data.ang_rate_Y << " ,"<<data.ang_rate_Z <<" \n";

        return os;
    }
};


class CanResolve
{
  public:
    CanResolve(ros::NodeHandle& nh);
    ~CanResolve() {}

    void ResolveFrame(TPCANMsg& Message);
    void resolve(TPCANMsg& Message);

    // second to date
    tm sec2Date(const long& sec)
    {
    time_t now(sec);
    return *(localtime(&now));
    }

    // date to string
    std::string date2Str(const tm& date)
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
  
  private:
    ros::NodeHandle node;
    ros::Publisher pubRawIMU;
    ros::Publisher pubGPS_;
    ros::Publisher pubCanData;

    bool isBegin;
    // bool isEnd;
    bool isInit;
    // uint16_t flag_;
    int lastID;
    RawCanData oneFrameData;

    // UTC时间比北京时间晚8小时
    long time_diff_8h = 28800;
    // GPS以周为单位计时--相对于1980年1月6日
    long gps_base_time = 315936000; // 1980.1.6相对于1900.1.1的秒数 00:00:00
    // 一周的秒数
    long week_sec = 604800;
    // gps相对于网络时间延迟 (典型值18s, gps快)
    double time_delay;

    // 角度转弧度的因子项
    double deg2Rad_factor = M_PI/180.0;

    std::string gps_frame_id; // GPS输出信息相对于坐标系id
    sensor_msgs::Imu ros_imu_raw;
    sensor_msgs::NavSatFix ros_gps_can;
    gps_driver::IMURaw ros_can_data;

    size_t nonvalidNum = 0;
    bool occurError = false;
    double last_time = 0.0;
};

CanResolve::CanResolve(ros::NodeHandle& nh)
    : node(nh)
    , isInit(false)
    , isBegin(false)
    // , flag_(0)
    , lastID(0)
{
    node.param("/gps/time_delay", time_delay, -18.0);
    node.param("/gps/frame_id", gps_frame_id, std::string("/gps_imu_baselink"));

    pubRawIMU = node.advertise<sensor_msgs::Imu>("/GPS/imu_data_raw", 120);
    pubGPS_ = node.advertise<sensor_msgs::NavSatFix>("/GPS/gps_can", 120);
    pubCanData = node.advertise<gps_driver::IMURaw>("/GPS/can_data", 120);
}

void CanResolve::ResolveFrame(TPCANMsg& Message)
{
    // printf("ID: %d\n", Message.ID);
    if(Message.ID == DWORD(1))
    {
        ROS_INFO("connection can device -- gps!");
        isInit = true;
        return;
    }
    if(!isInit){ // 防止意外发生
        ROS_WARN("connection signal not recived"); 
        return;
    }

    if(Message.ID == DWORD(800)){ // 做校验
        if(!isBegin){
            isBegin = true;
            lastID = 814;

            // DWORD a = 0;
            // DWORD tmp = DWORD(Message.DATA[0]);
            // a |= tmp<<8;
            // tmp = DWORD(Message.DATA[1]);
            // a |= tmp;
            // printf("/n a: %d.\n", a);
            if(occurError)
            {
                nonvalidNum ++;
                occurError = false;
                ROS_WARN_STREAM("can data resolve error: "<<nonvalidNum);
            }
        }
        else
        {
            ROS_ERROR("sth wrong occur");
        }
    }
    resolve(Message);
}

void CanResolve::resolve(TPCANMsg& Message)
{
    if(!isBegin) return;

    switch (int(Message.ID))
    {
        case 800:
            if(lastID != 814){
                isBegin = false;
                ROS_ERROR("frame id error 800");
                occurError = true;
            }
            lastID = 800;
            {
                int gps_week = 0; // gps周
                int tmp_ = int(Message.DATA[0]);
                gps_week |= tmp_<<8;
                tmp_ = int(Message.DATA[1]);
                gps_week |= tmp_;

                uint32_t gps_sec = 0; // gps秒
                tmp_ = int(Message.DATA[2]);
                gps_sec |= tmp_ << 24;
                tmp_ = int(Message.DATA[3]);
                gps_sec |= tmp_ << 16;
                tmp_ = int(Message.DATA[4]);
                gps_sec |= tmp_ << 8;
                tmp_ = int(Message.DATA[5]);
                gps_sec |= tmp_;
                double gps_sec_ = 0.001 * double(gps_sec);

                // 合成数据 -- 相对于UTC时间-18s, 然后在+8h变成北京时间
                oneFrameData.timestamp = gps_base_time + gps_week*week_sec + gps_sec_ + time_diff_8h + time_delay;
            }
            break;

        case 801:
            if(lastID != 800){
                isBegin = false;
                ROS_ERROR("frame id error 801");
                occurError = true;
            }
            lastID = 801;
            {
                int a = 0; u_char tmp_uchar;
                int tmp_ = int(Message.DATA[0]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[1]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[2];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.raw_gyo_x = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = int(Message.DATA[2]);
                a |= (tmp_&0x0f) << 16;
                tmp_ = int(Message.DATA[3]);
                a |= tmp_<<8;
                tmp_ = int(Message.DATA[4]);
                a |= tmp_;
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.raw_gyo_y = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = int(Message.DATA[5]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[6]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[7];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.raw_gyo_z = double(a) * 0.01 * deg2Rad_factor;
            }
            break;

        case 802:
            if(lastID != 801){
                isBegin = false;
                ROS_ERROR("frame id error 802");
                occurError = true;
            }
            lastID = 802;
            {
                int a = 0; u_char tmp_uchar;
                int tmp_ = int(Message.DATA[0]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[1]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[2];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.raw_acc_x = double(a) * 0.0001 * G_gps;

                a = 0; tmp_ = int(Message.DATA[2]);
                a |= (tmp_&0x0f) << 16;
                tmp_ = int(Message.DATA[3]);
                a |= tmp_<<8;
                tmp_ = int(Message.DATA[4]);
                a |= tmp_;
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.raw_acc_y = double(a) * 0.0001 * G_gps;

                a = 0; tmp_ = int(Message.DATA[5]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[6]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[7];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.raw_acc_z = double(a) * 0.0001 * G_gps;
                // printf("acc x: %f, y: %f, z: %f\n norm: %f\n", acc_x, acc_y, acc_z, sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z));
            }
            break;

        case 803:
            if(lastID != 802){
                isBegin = false;
                ROS_ERROR("frame id error 803");
                occurError = true;
            }
            lastID = 803;
            {
                oneFrameData.sys_state = DWORD(Message.DATA[0]);
                oneFrameData.gps_num_stats_used = DWORD(Message.DATA[1]);
                oneFrameData.sate_stuts = DWORD(Message.DATA[2]);
                oneFrameData.gps_num_stats2_used = DWORD(Message.DATA[3]);

                int16_t a = 0; int16_t tmp_ = Message.DATA[4];
                a |= tmp_ << 8;
                tmp_ = Message.DATA[5];
                a |= tmp_;
                oneFrameData.gps_age = double(a) * 0.01;

                oneFrameData.gps_num_sats = DWORD(Message.DATA[6]);
                oneFrameData.gps_num_sats2 = DWORD(Message.DATA[7]);
            }
            break;

        case 805:
            if(lastID != 803){
                isBegin = false;
                ROS_ERROR("frame id error 805");
                occurError = true;
            }
            lastID = 805;
            {
                int a = 0; int tmp_ = int(Message.DATA[0]);
                a |= tmp_ << 24;
                tmp_ = int(Message.DATA[1]);
                a |= tmp_ << 16;
                tmp_ = int(Message.DATA[2]);
                a |= tmp_ << 8;
                tmp_ = int(Message.DATA[3]);
                a |= tmp_;

                oneFrameData.altitude = 0.001 * double(a);
            }
            break;

        case 806:
            if(lastID != 805){
                isBegin = false;
                ROS_ERROR("frame id error 806");
                occurError = true;
            }
            lastID = 806;
            {
                uint16_t a = 0; uint16_t tmp_ = uint16_t(Message.DATA[0]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[1]);
                a |= tmp_;
                oneFrameData.pose_E_sigma = double(a) * 0.01;

                a = 0; tmp_ = uint16_t(Message.DATA[2]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[3]);
                a |= tmp_;
                oneFrameData.pose_N_sigma = double(a) * 0.01;

                a = 0; tmp_ = uint16_t(Message.DATA[4]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[5]);
                a |= tmp_;
                oneFrameData.pose_U_sigma = double(a) * 0.01;
            }
            break;

        case 807:
            if(lastID != 806){
                isBegin = false;
                ROS_ERROR("frame id error 807");
                occurError = true;
            }
            lastID = 807;
            {
                int16_t a = 0; int16_t tmp_ = int16_t(Message.DATA[0]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[1]);
                a |= tmp_;
                oneFrameData.vel_E = double(a) * 0.01;

                a = 0; tmp_ = int16_t(Message.DATA[2]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[3]);
                a |= tmp_;
                oneFrameData.vel_N = double(a) * 0.01;

                a = 0; tmp_ = int16_t(Message.DATA[4]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[5]);
                a |= tmp_;
                oneFrameData.vel_U = double(a) * 0.01;

                a = 0; tmp_ = int16_t(Message.DATA[6]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[7]);
                a |= tmp_;
                oneFrameData.vel_body = double(a) * 0.01;
            }
            break;

        case 808:
            if(lastID != 807){
                isBegin = false;
                ROS_ERROR("frame id error 808");
                occurError = true;
            }
            lastID = 808;
            {
                uint16_t a = 0; uint16_t tmp_ = uint16_t(Message.DATA[0]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[1]);
                a |= tmp_;
                oneFrameData.vel_E_sigma = double(a) * 0.001;

                a = 0; tmp_ = uint16_t(Message.DATA[2]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[3]);
                a |= tmp_;
                oneFrameData.vel_N_sigma = double(a) * 0.001;

                a = 0; tmp_ = uint16_t(Message.DATA[4]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[5]);
                a |= tmp_;
                oneFrameData.vel_U_sigma = double(a) * 0.001;

                a = 0; tmp_ = uint16_t(Message.DATA[6]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[7]);
                a |= tmp_;
                oneFrameData.vel_body_sigma = double(a) * 0.001;
            }
            break;

        case 809:
            if(lastID != 808){
                isBegin = false;
                ROS_ERROR("frame id error 809");
                occurError = true;
            }
            lastID = 809;
            {
                int a = 0; u_char tmp_uchar;
                int tmp_ = int(Message.DATA[0]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[1]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[2];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.acc_X = double(a) * 0.0001 * G_gps;

                a = 0; tmp_ = int(Message.DATA[2]);
                a |= (tmp_&0x0f) << 16;
                tmp_ = int(Message.DATA[3]);
                a |= tmp_<<8;
                tmp_ = int(Message.DATA[4]);
                a |= tmp_;
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.acc_Y = double(a) * 0.0001 * G_gps;

                a = 0; tmp_ = int(Message.DATA[5]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[6]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[7];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.acc_Z = double(a) * 0.0001 * G_gps;
            }
            break;

        case 810:
            if(lastID != 809){
                isBegin = false;
                ROS_ERROR("frame id error 810");
                occurError = true;
            }
            lastID = 810;
            {
                int16_t a = 0; int16_t tmp_ = int16_t(Message.DATA[0]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[1]);
                a |= tmp_;
                oneFrameData.angle_heading = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = int16_t(Message.DATA[2]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[3]);
                a |= tmp_;
                oneFrameData.angle_pitch = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = int16_t(Message.DATA[4]);
                a |= tmp_ << 8;
                tmp_ = int16_t(Message.DATA[5]);
                a |= tmp_;
                oneFrameData.angle_roll = double(a) * 0.01 * deg2Rad_factor;

            }
            break;

        case 811:
            if(lastID != 810){
                isBegin = false;
                ROS_ERROR("frame id error 811");
                occurError = true;
            }
            lastID = 811;
            {
                uint16_t a = 0; uint16_t tmp_ = uint16_t(Message.DATA[0]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[1]);
                a |= tmp_;
                oneFrameData.angle_heading_sigma = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = uint16_t(Message.DATA[2]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[3]);
                a |= tmp_;
                oneFrameData.angle_pitch_sigma = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = uint16_t(Message.DATA[4]);
                a |= tmp_<<8;
                tmp_ = uint16_t(Message.DATA[5]);
                a |= tmp_;
                oneFrameData.angle_roll_sigma = double(a) * 0.01 * deg2Rad_factor;

            }
            break;

        case 812:
            if(lastID != 811){
                isBegin = false;
                ROS_ERROR("frame id error 812");
                occurError = true;
            }
            lastID = 812;
            {
                int a = 0; u_char tmp_uchar;
                int tmp_ = int(Message.DATA[0]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[1]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[2];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.ang_rate_X = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = int(Message.DATA[2]);
                a |= (tmp_&0x0f) << 16;
                tmp_ = int(Message.DATA[3]);
                a |= tmp_<<8;
                tmp_ = int(Message.DATA[4]);
                a |= tmp_;
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.ang_rate_Y = double(a) * 0.01 * deg2Rad_factor;

                a = 0; tmp_ = int(Message.DATA[5]);
                a |= tmp_<<12;
                tmp_ = int(Message.DATA[6]);
                a |= tmp_<<4;
                tmp_uchar = Message.DATA[7];
                a |= (tmp_uchar >> 4) &0x0f;
                // tmp_ = 
                a = (a&0x80000)?  -((a&0x7ffff)^0x7ffff) : a ;
                oneFrameData.ang_rate_Z = double(a) * 0.01 * deg2Rad_factor;
            }
            break;
            
        case 813:
            if(lastID != 812){
                isBegin = false;
                ROS_ERROR("frame id error 813");
                occurError = true;
            }
            lastID = 813;
            {
                int64_t a = 0; int64_t tmp_ = int64_t(Message.DATA[0]);
                a |= tmp_ << 56;
                tmp_ = int64_t(Message.DATA[1]);
                a |= tmp_ << 48;
                tmp_ = int64_t(Message.DATA[2]);
                a |= tmp_ << 40;
                tmp_ = int64_t(Message.DATA[3]);
                a |= tmp_ << 32;
                tmp_ = int64_t(Message.DATA[4]);
                a |= tmp_ << 24;
                tmp_ = int64_t(Message.DATA[5]);
                a |= tmp_ << 16;
                tmp_ = int64_t(Message.DATA[6]);
                a |= tmp_ << 8;
                tmp_ = int64_t(Message.DATA[7]);
                a |= tmp_;
                oneFrameData.lontitude = double(a) * 1.0e-8;
                
            }
            break;

        case 814:
            if(lastID != 813){
                isBegin = false;
                ROS_ERROR("frame id error 814");
                occurError = true;
            }
            lastID = 814;
            isBegin = false; // 这里是真的结束了
            {
                int64_t a = 0; int64_t tmp_ = int64_t(Message.DATA[0]);
                a |= tmp_ << 56;
                tmp_ = int64_t(Message.DATA[1]);
                a |= tmp_ << 48;
                tmp_ = int64_t(Message.DATA[2]);
                a |= tmp_ << 40;
                tmp_ = int64_t(Message.DATA[3]);
                a |= tmp_ << 32;
                tmp_ = int64_t(Message.DATA[4]);
                a |= tmp_ << 24;
                tmp_ = int64_t(Message.DATA[5]);
                a |= tmp_ << 16;
                tmp_ = int64_t(Message.DATA[6]);
                a |= tmp_ << 8;
                tmp_ = int64_t(Message.DATA[7]);
                a |= tmp_;
                oneFrameData.latitude = double(a) * 1.0e-8;
            }
            break;
                
        default:
            break;
    }
    if(lastID==814 && !isBegin){
        // ROS_INFO("one frame");
        // std::cout<<oneFrameData<<" \n";
        // std::string gps_date = date2Str(sec2Date(long(oneFrameData.timestamp)));
        // std::cout<<"gps: "<<gps_date<<'\n';
        ros_imu_raw.header.frame_id = gps_frame_id;
        ros_imu_raw.header.stamp = ros::Time().fromSec(oneFrameData.timestamp);
        ros_imu_raw.linear_acceleration.x = oneFrameData.raw_acc_x; // 原始IMU数据
        ros_imu_raw.linear_acceleration.y = oneFrameData.raw_acc_y;
        ros_imu_raw.linear_acceleration.z = oneFrameData.raw_acc_z;
        ros_imu_raw.angular_velocity.x = oneFrameData.raw_gyo_x;
        ros_imu_raw.angular_velocity.y = oneFrameData.raw_gyo_y;
        ros_imu_raw.angular_velocity.z = oneFrameData.raw_gyo_z;

        ros_gps_can.header = ros_imu_raw.header; // 经纬度
        ros_gps_can.latitude = oneFrameData.latitude;
        ros_gps_can.longitude = oneFrameData.lontitude;
        ros_gps_can.altitude = oneFrameData.altitude;

        ros_can_data.header = ros_imu_raw.header; // can数据
        ros_can_data.raw_gyo_x            =   oneFrameData.raw_gyo_x;
        ros_can_data.raw_gyo_y            =   oneFrameData.raw_gyo_y;
        ros_can_data.raw_gyo_z            =   oneFrameData.raw_gyo_z;
        ros_can_data.raw_acc_x            =   oneFrameData.raw_acc_x;
        ros_can_data.raw_acc_y            =   oneFrameData.raw_acc_y;
        ros_can_data.raw_acc_z            =   oneFrameData.raw_acc_z;
        ros_can_data.sys_state            =   oneFrameData.sys_state;
        ros_can_data.gps_num_stats_used   =   oneFrameData.gps_num_stats_used;
        ros_can_data.sate_stuts           =   oneFrameData.sate_stuts;
        ros_can_data.gps_num_stats2_used  =   oneFrameData.gps_num_stats2_used;
        ros_can_data.gps_age              =   oneFrameData.gps_age;
        ros_can_data.gps_num_sats         =   oneFrameData.gps_num_sats;
        ros_can_data.gps_num_sats2        =   oneFrameData.gps_num_sats2;
        ros_can_data.latitude             =   oneFrameData.latitude;
        ros_can_data.lontitude            =   oneFrameData.lontitude;
        ros_can_data.altitude             =   oneFrameData.altitude;
        ros_can_data.pose_E_sigma         =   oneFrameData.pose_E_sigma;
        ros_can_data.pose_N_sigma         =   oneFrameData.pose_N_sigma;
        ros_can_data.pose_U_sigma         =   oneFrameData.pose_U_sigma;
        ros_can_data.vel_E                =   oneFrameData.vel_E;
        ros_can_data.vel_N                =   oneFrameData.vel_N;
        ros_can_data.vel_U                =   oneFrameData.vel_U;
        ros_can_data.vel_body             =   oneFrameData.vel_body;
        ros_can_data.vel_E_sigma          =   oneFrameData.vel_E_sigma;
        ros_can_data.vel_N_sigma          =   oneFrameData.vel_N_sigma;
        ros_can_data.vel_U_sigma          =   oneFrameData.vel_U_sigma;
        ros_can_data.vel_body_sigma       =   oneFrameData.vel_body_sigma;
        ros_can_data.acc_X                =   oneFrameData.acc_X;
        ros_can_data.acc_Y                =   oneFrameData.acc_Y;
        ros_can_data.acc_Z                =   oneFrameData.acc_Z;
        ros_can_data.angle_heading        =   oneFrameData.angle_heading;
        ros_can_data.angle_pitch          =   oneFrameData.angle_pitch;
        ros_can_data.angle_roll           =   oneFrameData.angle_roll;
        ros_can_data.angle_heading_sigma  =   oneFrameData.angle_heading_sigma;
        ros_can_data.angle_pitch_sigma    =   oneFrameData.angle_pitch_sigma;
        ros_can_data.angle_roll_sigma     =   oneFrameData.angle_roll_sigma;
        ros_can_data.ang_rate_X           =   oneFrameData.ang_rate_X;
        ros_can_data.ang_rate_Y           =   oneFrameData.ang_rate_Y;
        ros_can_data.ang_rate_Z           =   oneFrameData.ang_rate_Z;

        pubRawIMU.publish(ros_imu_raw);
        pubGPS_.publish(ros_gps_can);
        pubCanData.publish(ros_can_data);

        if(oneFrameData.timestamp - last_time > 0.025)
            ROS_ERROR_STREAM("can data lost: "<<(oneFrameData.timestamp - last_time));

        last_time = oneFrameData.timestamp;
    }
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "canResolveNode");
    ros::NodeHandle nh;

    CanResolve can_resolver(nh);

    // 获取参数
    std::string pcan_dev;
    nh.param("/gps/pcan_dev", pcan_dev, std::string("pcanusb32"));

    TPCANMsg Message;
    TPCANStatus Status;
	DWORD status;
	unsigned long ulIndex = 0;

    char *endptr;
    unsigned long pcan_device = PCAN_USBBUS1;
    unsigned long tmp = strtoul(pcan_dev.c_str(), &endptr, 0);
    if(*endptr == '\0')
        pcan_device = tmp;

    Status = CAN_Initialize(pcan_device, PCAN_BAUD_500K, 0, 0, 0);

    ROS_INFO("CAN_Initialize(%xh): Status=0x%x\n", pcan_device, (int)Status);

    if(Status)
    {
        ROS_ERROR("fail to initialize the can device!!!");
        return 0;
    }

    while (ros::ok())
    {TicToc t_read_;
        while ((Status=CAN_Read(pcan_device, &Message, NULL)) == PCAN_ERROR_QRCVEMPTY)
			if (usleep(10))
				break;
        // printf("read time: %f ms.\n", t_read_.Toc());
        if (Status != PCAN_ERROR_OK) {
			printf("CAN_Read(%xh) failure 0x%x\n", pcan_device, (int)Status);
			break;
		}

        // 解析
        // printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
		// 	(int)Message.ID, (int)Message.LEN,
		// 	(int)Message.DATA[0], (int)Message.DATA[1],
		// 	(int)Message.DATA[2], (int)Message.DATA[3],
		// 	(int)Message.DATA[4], (int)Message.DATA[5],
		// 	(int)Message.DATA[6], (int)Message.DATA[7]);
        TicToc t_process_one;
        can_resolver.ResolveFrame(Message);
        if(t_process_one.Toc() > 0.1)
            ROS_INFO_STREAM("one can data: "<<t_process_one.Toc()<<" ms.");
        
    }

    CAN_Uninitialize(pcan_device);    

    return 0;
}