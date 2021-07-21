/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//--------------------------------------------------------------------------------
// Xsens device API example for an MTi / MTx / Mtmk4 device using the C++ API
// 修改：增加了同步相关内容
// By：He， 2018
//--------------------------------------------------------------------------------
#include <xsensdeviceapi.h> // The Xsens device API header

#include <iostream>
#include <list>
#include <iomanip>
#include <stdexcept>
#include <fstream>

#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
// #include <xsens/xstime.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include "xsens_imu_driver/synin.h"
#include "xsens_imu_driver/imu_pressure.h"
#include "xsens_imu_driver/imu_magneticField.h"

#include <sensor_msgs/TimeReference.h>

#include <string>
#include <sstream>
#include <serial/serial.h>
// 解析时间--计算imu和gps的时间延迟
#include <ctime>
#include <time.h>
#include <string.h>
#include <stdlib.h> 
#include <thread>

// *****************************
//如果要使用xsens imu的同步功能，则使用如下这个宏定义
//否则，将其注释掉
#define ENABLE_IMU_SYNC_MODE
// *****************************

//是否输出调试信息到文件
//#define ENABLE_OUTPUT_DEBUG_INFO

using namespace std;

using namespace xsens;

using XsMutex = xsens::Mutex;
using XsMutexLocker = xsens::Lock;
using XsUtcTime = XsTimeInfo;

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5) :
		m_maxNumberOfPacketsInBuffer(maxBufferSize),
		m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		XsMutexLocker lock(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		XsMutexLocker lock(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(&m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
		{
			(void)getNextPacket();
		}
		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable XsMutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};



/*//送过来的UTC时间（带时区的），转换为标准Unix时间戳（不带时区的）
double xsUtcTime2UnixTimestamp(const XsUtcTime& utc_time)
{
	struct tm tmp_time;
	char strt[64];
	sprintf(strt, "%04d%02d%02d%02d%02d%02d", utc_time.m_year, utc_time.m_month, utc_time.m_day, 
		utc_time.m_hour, utc_time.m_minute, utc_time.m_second);
	strptime(strt,"%Y%m%d%H%M%S",&tmp_time);
	return (double)mktime(&tmp_time) + utc_time.m_nano * 1e-9;  
}*/
//xs送过来的UTC时间（不带时区的），转换为标准Unix时间戳（不带时区的）
double xsUtcTime2UnixTimestamp(const XsUtcTime& utc_time)
{
	struct tm tmp_time;
	char strt[64];
	sprintf(strt, "%04d%02d%02d%02d%02d%02d", utc_time.m_year, utc_time.m_month, utc_time.m_day, 
		utc_time.m_hour, utc_time.m_minute, utc_time.m_second);
	strptime(strt,"%Y%m%d%H%M%S",&tmp_time);
	setenv("TZ", "Universal", 1);
	tzset();
	return (double)mktime(&tmp_time) + utc_time.m_nano * 1e-9;  
}
// 通过串口数据计算GPS时间与IMU时间偏差量
std::string serial_port;
int baudRate;
std::string qian_zui = "$GPRMC";
bool get_trigger_signal = false;
double trigger_in_time = 0.0;
double t_shift = 0.0; // gps
std::string cache_str;
bool go_on = false;

// 处理数据, 一秒一帧, 非一次完整接收
void readAndProcess(std::string& msg)
{
	static int hour, min, sec, year, mon, day;
	static struct tm p = tm();

	if(msg.size() < 7) return;
	if(go_on || msg.substr(0, 6) == qian_zui)
    {
		if(!go_on &&  msg.substr(0, 6) == qian_zui){
			cache_str.clear(); // 确定是这帧开头
			cache_str = msg;
			if(cache_str.find("\r\n") == -1) go_on = true;
		}
		else if(go_on)
		{ // 或者这帧还未结束
			cache_str += msg;
			// ROS_INFO_STREAM(cache_str);
			{ // 解析数据
				if(cache_str.find("\r\n") == -1)
					return; // 找不到这帧结尾表示还未结束
				else
				{ // 接收到完整一帧
					go_on = false;
					get_trigger_signal = false;
					
					{// 解析数据
						size_t count_ = 0;
						int comma_pose = cache_str.find(",");
						count_ = comma_pose+1; // $GPRMC,
						comma_pose = cache_str.find(",", count_);
						if(!cache_str.substr(count_, comma_pose-count_).empty())
						{// 时分秒
							hour = (cache_str[count_]-'0')*10 + (cache_str[count_+1]-'0');
							min = (cache_str[count_+2]-'0')*10 + (cache_str[count_+3]-'0');
							sec = (cache_str[count_+4]-'0')*10 + (cache_str[count_+5]-'0');
						}
						count_ = comma_pose+1; // 时分秒.00,
						count_ += 2;
						comma_pose = cache_str.find(",", count_);
						count_ = comma_pose+1; // 纬度,
						count_ += 2;
						comma_pose = cache_str.find(",", count_);
						count_ = comma_pose+1; // 经度,
						count_ += 2;
						comma_pose = cache_str.find(",", count_);
						count_ = comma_pose+1; // 地面速率,
						comma_pose = cache_str.find(",", count_);
						count_ = comma_pose+1; // 地面航向,

						comma_pose = cache_str.find(",", count_);
						if(!cache_str.substr(count_, comma_pose-count_).empty())
						{// 年月日
							day = (cache_str[count_]-'0')*10 + (cache_str[count_+1]-'0');
							mon = (cache_str[count_+2]-'0')*10 + (cache_str[count_+3]-'0');
							year = (cache_str[count_+4]-'0')*10 + (cache_str[count_+5]-'0') + 2000;
						}

						// 换算成秒
						p.tm_year = year - 1900;
						p.tm_mon = mon - 1; 
						p.tm_mday = day;
						p.tm_hour = hour+8; // 带上时区
						p.tm_min = min;
						p.tm_sec = sec;
						time_t utc_t = mktime(&p);
						t_shift = double(utc_t) - trigger_in_time;

						// printf("t_shift: %.6f, %.6f.\n", t_shift, trigger_in_time);

						if(fabs(t_shift) > 1.0)
						{
							ROS_WARN("check the system time! t_shift: %.5fs.", t_shift);
						}
					}
				}
				
			}
		}

		// 直接当异常情况扔掉
	}
}

void computeTimeShift()
{
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
        return;
    }
	if(ser.isOpen()) // 确保打开
        ROS_INFO("serial open successfully!");
    else
    {
        return;
    }
	size_t num = 0;
	while(ros::ok())
	{
		if(num = ser.available()){
            std::string msg = ser.read(num);

			if(get_trigger_signal){ // 接收到触发信号之后再处理
            	readAndProcess(msg);
            	// ROS_INFO_STREAM("msg: "<<msg);
			}
        }

		XsTime::msleep(10);
	}

	ser.close();// 关闭串口
}


//--------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "xsens_imu_driver");
	ros::NodeHandle nh;
  	ros::Publisher pub_data = nh.advertise<sensor_msgs::Imu> ("/imu/data", 1);
	ros::Publisher pub_pressure = nh.advertise<xsens_imu_driver::imu_pressure> ("/imu/pressure", 1);
	ros::Publisher pub_mag = nh.advertise<xsens_imu_driver::imu_magneticField> ("/imu/magneticField", 1);
	
#ifdef ENABLE_IMU_SYNC_MODE
	ros::Publisher pub_synout = nh.advertise<std_msgs::Header> ("/imu/synout_msg", 5);
	ros::Publisher pub_synin = nh.advertise<xsens_imu_driver::synin> ("/imu/synin_msg", 5);
	ros::Publisher pub_sync_out = nh.advertise<sensor_msgs::TimeReference>("/imu/trigger_time", 1);
#endif

	
	// Create XsControl object
	XsControl* control = XsControl::construct();
	assert(control != 0);

	try
	{
		// Scan for connected devices
		XsPortInfoArray portInfoArray = XsScanner::scanPorts();

		// Find an MTi / MTx / MTmk4 device
		XsPortInfoArray::const_iterator mtPort = portInfoArray.begin();
		while (mtPort != portInfoArray.end() && !mtPort->deviceId().isMti() && !mtPort->deviceId().isMtig() ) {++mtPort;}
		//while (mtPort != portInfoArray.end() && !mtPort->deviceId().isMtMk4()) {++mtPort;}
		if (mtPort == portInfoArray.end())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort->deviceId().toString().toStdString() << " @ port: " << mtPort->portName().toStdString() << ", baudrate: " << mtPort->baudrate() << std::endl;

		// Open the port with the detected device
		if (!control->openPort(mtPort->portName().toStdString(), mtPort->baudrate()))
		{
			throw std::runtime_error("Could not open port. Aborting.");
		}

		try
		{
			// Get the device object
			XsDevice* device = control->device(mtPort->deviceId());
			assert(device != 0);

			// Print information about detected MTi / MTx / MTmk4 device
			std::cout << "Device: " << device->productCode().toStdString() << " opened." << std::endl;
			
			// Create and attach callback handler to device
			CallbackHandler callback;
			device->addCallbackHandler(&callback);
			
			
			//提前设置好一些变量
			sensor_msgs::Imu imuData;
			imuData.header.frame_id = "/imu";
			
			std_msgs::Header synout_msg;
			uint32_t synout_counter = 0;
			xsens_imu_driver::synin synin_msg;
			uint32_t synin_counter = 0;
			sensor_msgs::TimeReference tigger_time;
			
			xsens_imu_driver::imu_pressure pressure_msg;
			xsens_imu_driver::imu_magneticField magneticField_msg;
			
			double current_imu_time;
			unsigned int synin_marker, synout_marker;			
			double m_pressure;  //Unit: Pa
			double mag_x, mag_y, mag_z;
			
#ifdef ENABLE_IMU_SYNC_MODE
			//============= config ===================================
			if (!device->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("Could not put device into configuration mode. Aborting.");
			}
			
			//设置内部UTC时间为当前系统时间，这样就统一了基准
			//XsUtcTime::currentTime()获取的时间，是不带时区的
			XsUtcTime wall_utc_time = XsUtcTime::currentTime();
			wall_utc_time.m_hour +=8;
			device->setUtcTime(wall_utc_time);
			printf("UTC time: %04d-%02d-%02d %02d:%02d:%02d.%04d    , m_valid:%d\n", wall_utc_time.m_year, wall_utc_time.m_month, wall_utc_time.m_day,
								wall_utc_time.m_hour, wall_utc_time.m_minute, wall_utc_time.m_second, wall_utc_time.m_nano,    wall_utc_time.m_valid);
			
			//设置imu的trigger mode
			XsSyncSetting *psysettings= new XsSyncSetting[3];
			//XsSyncLine line, XsSyncFunction function, XsSyncPolarity polarity, pulseWidth, offset, skipFirst, skipFactor, clockPeriod, triggerOnce
			psysettings[0] = XsSyncSetting(XSL_Bi1Out, XSF_IntervalTransitionMeasurement , XSP_RisingEdge, 500, 0, 30, 15, 0, 0);
			psysettings[1] = XsSyncSetting(XSL_In1, XSF_TriggerIndication, XSP_RisingEdge, 0, 0, 0, 0, 0, 0);  //XSL_Bi1In
			psysettings[2] = XsSyncSetting(XSL_ClockIn, XSF_ClockBiasEstimation, XSP_RisingEdge, 0, 0, 0, 0, 1000, 0);  //XSL_ExtTimepulseIn
			
			XsSyncSettingArray settingList(3, psysettings);
			if(!device->setSyncSettings(settingList))
			{
				throw std::runtime_error("Could not set the Sync Settings. Aborting.");
			}
			//===============================================================
#endif
			
			// Put the device in measurement mode
			if (!device->gotoMeasurement())
			{
				throw std::runtime_error("Could not put device into measurement mode. Aborting.");
			}

			// 获取参数
			nh.param("/imu/sync_SerialPort", serial_port, std::string("/dev/ttyUSB2"));
			nh.param("/imu/sync_BaudRate", baudRate, 9600);// 波特率
			// 开辟线程处理串口数据
			std::thread process_serial(computeTimeShift);
			
			//主循环，获取imu数据，发布
			while (ros::ok())
			{
				if (callback.packetAvailable())
				{
#ifndef ENABLE_IMU_SYNC_MODE
					current_imu_time = ros::Time::now().toSec();  //没开启同步模式，使用系统时间
#endif
					// Retrieve a packet
					XsDataPacket packet = callback.getNextPacket();
					//printf("\n");
					
#ifdef ENABLE_IMU_SYNC_MODE
					//=====================================
					//Get utc time
					if (packet.containsUtcTime())
					{
						XsUtcTime utc_time = packet.utcTime();
						// printf("UTC time: %04d-%02d-%02d %02d:%02d:%02d.%04d    , m_valid:%d\n", utc_time.m_year, utc_time.m_month, utc_time.m_day,
						// 		utc_time.m_hour, utc_time.m_minute, utc_time.m_second, utc_time.m_nano,    utc_time.m_valid);
						
						current_imu_time = xsUtcTime2UnixTimestamp(utc_time);
						// std::cout << std::fixed;
						// std::cout<<"Current Unix Timestamp in IMU: "<<current_imu_time<<std::endl;
					}

					//Get status
					if (packet.containsStatus())
					{
						uint32_t status_words = packet.status();
						synout_marker = (status_words & 0x00400000) >> 22;			
						synin_marker = (status_words & 0x00200000) >> 21;
						
						// printf("status_words: %x\n", status_words);
						if(synin_marker) printf("Syn in !\n");
						// if(synout_marker) printf("Syn out !\n");
						
						
						//发送IMU syn_in消息
						if(synin_marker)
						{
							get_trigger_signal = true;
							trigger_in_time = current_imu_time;
							synin_msg.timeSys = ros::Time::now().toSec();  //当前系统时间
							synin_msg.header.stamp = ros::Time().fromSec(current_imu_time+t_shift);
							pub_synin.publish(synin_msg);
			
#ifdef ENABLE_OUTPUT_DEBUG_INFO			
							std::ofstream myout;
							myout.open("/home/robot/imu-info.txt", std::ios::app);
							myout<<std::fixed<<"synin "<<++synin_counter<<",  sysTime: "<<synin_msg.timeSys<<",  imuTime: "<<current_imu_time<<std::endl;
							myout.close();
#endif
						}
						//发送IMU syn_out消息
						if(synout_marker)
						{
							// 发布trigger时间
							tigger_time.header.stamp = ros::Time().fromSec(current_imu_time+t_shift);
							pub_sync_out.publish(tigger_time);

							synout_msg.stamp = ros::Time().fromSec(current_imu_time+t_shift);
							synout_msg.seq = ++synout_counter;
							pub_synout.publish(synout_msg);
						}
					}
					//=====================================
#endif
					
					//Get Pressure, 气压 Pa
					if (packet.containsPressure())
					{
						XsPressure xsens_pressure = packet.pressure();
						m_pressure = xsens_pressure.m_pressure;  
						//printf("Pressure: %f\n", m_pressure);
						
						pressure_msg.pressure = m_pressure;
						pressure_msg.header.stamp = ros::Time().fromSec(current_imu_time+t_shift);
						pub_pressure.publish(pressure_msg);
					}	

					//Get MagneticField, 地磁
					if (packet.containsCalibratedMagneticField())
					{
						XsVector xsens_magneticfield = packet.calibratedMagneticField();
						mag_x = xsens_magneticfield.at(0);
						mag_y = xsens_magneticfield.at(1);
						mag_z = xsens_magneticfield.at(2);
						//printf("Magnetic Field: %f  %f  %f\n", mag_x, mag_y, mag_z);
						
						magneticField_msg.magX = mag_x;
						magneticField_msg.magY = mag_y;
						magneticField_msg.magZ = mag_z;
						magneticField_msg.header.stamp = ros::Time().fromSec(current_imu_time+t_shift);
						pub_mag.publish(magneticField_msg);
					}


					// Get the orientation data, 角度四元数
					if (packet.containsOrientation())
					{
						XsQuaternion quaternion = packet.orientationQuaternion();

						imuData.orientation.x = quaternion.x(); //四元数
						imuData.orientation.y = quaternion.y();
						imuData.orientation.z = quaternion.z();
						imuData.orientation.w = quaternion.w();
						//printf("Orientation !\n");
					}


					// Get the gyroscope data, 角速度  rad/s
					if (packet.containsCalibratedGyroscopeData())
					{
						XsVector gyroscope = packet.calibratedGyroscopeData();
					
						imuData.angular_velocity.x = gyroscope.at(0);
						imuData.angular_velocity.y = gyroscope.at(1);
						imuData.angular_velocity.z = gyroscope.at(2);
						//printf("Gyroscope !\n");
					}

					// Get the acceleration data, 加速度 m/s^2
					if (packet.containsCalibratedAcceleration()) 
					{
						XsVector acceleration = packet.calibratedAcceleration();

						imuData.linear_acceleration.x = acceleration.at(0);
						imuData.linear_acceleration.y = acceleration.at(1);
						imuData.linear_acceleration.z = acceleration.at(2);
						//printf("Acceleration !\n");
					}

					
					//发送正常IMU消息
					if( std::isnan(imuData.orientation.x) || std::isnan(imuData.orientation.y)  || std::isnan(imuData.orientation.z) || std::isnan(imuData.orientation.w))
					{}
					else
					{
						//imuData.header.stamp = ros::Time::now();
						imuData.header.stamp = ros::Time().fromSec(current_imu_time+t_shift);
						pub_data.publish(imuData);
					}
				}

				XsTime::msleep(1);
			} //end while (ros::ok())

			process_serial.join();
			

#ifdef ENABLE_IMU_SYNC_MODE
			// ========= 结束时，需要将同步的设置清空 ==============
			if (!device->gotoConfig())
			{
				throw std::runtime_error("Could not put device into config mode. Aborting.");
			}
			settingList.clear();  //空的list，意味着清空设置
			if(!device->setSyncSettings(settingList))
			{
				throw std::runtime_error("Could not set the Sync Settings. Aborting.");
			}
			//===========================================
#endif
			
		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}
		
		// Close port
		control->closePort(mtPort->portName().toStdString());
	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}
	
	// Free XsControl object
	control->destruct();
	std::cout << "Successful exit." << std::endl;
	return 0;
}
