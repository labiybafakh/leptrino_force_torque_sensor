#ifndef LEPTRINO_FORCE_TORQUE_HPP
/*
MIT License

Copyright (c) 2023 Muhammad Labiyb Afakh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <array>
#include <mutex>  // Added for thread-safe calibration

#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>
#include <leptrino/pComResInternal.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
// Added new includes for calibration functionality
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;  // Added for service callbacks

class LeptrinoNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  // Added new publisher for calibration status
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_status_pub_;
  
  // Added new subscriber for recalibration
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recalibrate_sub_;
  
  // Added new service for recalibration
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recalibrate_service_;
  
  rclcpp::TimerBase::SharedPtr timer_acquisition_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;
  // Added new timer for auto recalibration
  rclcpp::TimerBase::SharedPtr timer_auto_recalibration_;

  void App_Init();
  void App_Close(rclcpp::Logger logger);
  ULONG SendData(UCHAR *pucInput, USHORT usSize);
  void GetProductInfo(rclcpp::Logger logger);
  void GetLimit(rclcpp::Logger logger);
  void SerialStart(rclcpp::Logger logger);
  void SerialStop(rclcpp::Logger logger);
  
  // Added new methods for calibration
  void RecalibrateService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void RecalibrateTopic(const std_msgs::msg::Empty::SharedPtr msg);
  void AutoRecalibrateCallback();
  bool PerformRecalibration();
  void PublishCalibrationStatus(bool is_calibrating);

  struct ST_SystemInfo
  {
    int com_ok;
  };

  struct FS_data
  {
    std::array<double, 3> force;
    std::array<double, 3> moment;
  };

  UCHAR CommRcvBuff[256];
  UCHAR CommSendBuff[1024];
  UCHAR SendBuff[512];
  double conversion_factor[FN_Num];

  std::string serial_port_;
  int g_rate = 1000;
  
  // Added new variables for calibration
  bool auto_recalibration_enabled_;
  double auto_recalibration_interval_minutes_;
  int calibration_samples_;
  bool is_calibrating_;
  std::mutex calibration_mutex_;

  void SensorAquistionCallback();
  void PublisherCallback();

  rclcpp::Parameter serial_port;
  bool check_nan_=false;
  bool check_zero_=false;

public:
  ST_R_DATA_GET_F *stForce;
  ST_R_DATA_GET_F *grossForce;
  ST_R_GET_INF *stGetInfo;
  ST_R_LEP_GET_LIMIT *stGetLimit;
  ST_SystemInfo gSys;
  FS_data sensor_data;
  FS_data offset;

  LeptrinoNode();
  ~LeptrinoNode();
  void SensorCallback();
  void init(rclcpp::Logger logger);
  void SensorCalibration(rclcpp::Logger logger);
};


#endif