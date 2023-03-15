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

#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>
#include <leptrino/pComResInternal.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class LeptrinoNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_acquisition_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  void App_Init();
  void App_Close(rclcpp::Logger logger);
  ULONG SendData(UCHAR *pucInput, USHORT usSize);
  void GetProductInfo(rclcpp::Logger logger);
  void GetLimit(rclcpp::Logger logger);
  void SerialStart(rclcpp::Logger logger);
  void SerialStop(rclcpp::Logger logger);

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