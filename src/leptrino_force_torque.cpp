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

#include "rclcpp/rclcpp.hpp"
#include "leptrino_force_torque.hpp"

#define PRG_VER "Ver 2.1.0"

// Debug calibration and output sensor
#define DEBUG 0

// to calibrate the offset of sensor
#define Calibration 1

LeptrinoNode::LeptrinoNode() : Node("Leptrino")
{
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("leptrino", 5);

  this->declare_parameter("com_port", "/dev/ttyUSB0");

  serial_port = this->get_parameter("com_port");

  if (!this->get_parameter("com_port", serial_port))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get serial_port parameter");
  }
  else
  {
    serial_port_ = serial_port.as_string();
    RCLCPP_INFO(this->get_logger(), "Using serial port: %s", serial_port_.c_str());
  }

  LeptrinoNode::App_Init();

  if (gSys.com_ok == NG)
  {
    RCLCPP_ERROR(this->get_logger(), "%s open failed\n", serial_port_.c_str());
    exit(0);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Leptrino is connected");
    rclcpp::sleep_for(100ms);
    LeptrinoNode::GetLimit(this->get_logger());
    rclcpp::sleep_for(100ms);
    LeptrinoNode::SerialStart(this->get_logger());
    rclcpp::sleep_for(100ms);
    LeptrinoNode::SensorCalibration(this->get_logger());
    rclcpp::sleep_for(100ms);

    timer_acquisition_ = this->create_wall_timer(1ms, std::bind(&LeptrinoNode::SensorCallback, this));
    timer_publisher_ = this->create_wall_timer(1ms, std::bind(&LeptrinoNode::PublisherCallback, this));
  }
}

LeptrinoNode::~LeptrinoNode()
{
  LeptrinoNode::SerialStop(this->get_logger());
  LeptrinoNode::App_Close(this->get_logger());
  RCLCPP_WARN(this->get_logger(), "Leptrino has been stopped");
}

void LeptrinoNode::init(rclcpp::Logger logger)
{
  LeptrinoNode::SensorCalibration(logger);
}

void LeptrinoNode::PublisherCallback()
{
  if (rclcpp::ok())
  {
    auto wrench_msg = std::make_shared<geometry_msgs::msg::WrenchStamped>();

    wrench_msg->header.stamp = this->now();
    wrench_msg->header.frame_id = "leptrino_sensor";
    wrench_msg->wrench.force.x = sensor_data.force[1];
    wrench_msg->wrench.force.y = sensor_data.force[2];
    wrench_msg->wrench.force.z = sensor_data.force[3];
    wrench_msg->wrench.torque.x = sensor_data.moment[1];
    wrench_msg->wrench.torque.y = sensor_data.moment[2];
    wrench_msg->wrench.torque.z = sensor_data.moment[3];

    wrench_pub_->publish(*wrench_msg);
  }
}

void LeptrinoNode::SensorCallback()
{
  if (rclcpp::ok())
  {
    Comm_Rcv();

    if (Comm_CheckRcv() != 0)
    {
      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));

      auto rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stForce = (ST_R_DATA_GET_F*)CommRcvBuff;

        sensor_data.force[1] = (double)(stForce->ssForce[0] * conversion_factor[0]) + (offset.force[1] * -1);
        sensor_data.force[2] = (double)(stForce->ssForce[1] * conversion_factor[1]) + (offset.force[2] * -1);
        sensor_data.force[3] = (double)(stForce->ssForce[2] * conversion_factor[2]) + (offset.force[3] * -1);
        sensor_data.moment[1] = (double)(stForce->ssForce[3] * conversion_factor[3]) + (offset.moment[1] * -1);
        sensor_data.moment[2] = (double)(stForce->ssForce[4] * conversion_factor[4]) + (offset.moment[2] * -1);
        sensor_data.moment[3] = (double)(stForce->ssForce[5] * conversion_factor[5]) + (offset.moment[3] * -1);

        if (DEBUG)
        {
          RCLCPP_INFO(this->get_logger(), "Output: %f,%f,%f,%f,%f,%f", sensor_data.force[1], sensor_data.force[2],
                      sensor_data.force[3], sensor_data.moment[1], sensor_data.moment[2], sensor_data.moment[3]);
        }
      }
    }
  }
}

void LeptrinoNode::App_Init()
{
  int rt;

  // Commポート初期化
  gSys.com_ok = NG;
  rt = Comm_Open(serial_port_.c_str());
  if (rt == OK)
  {
    Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    gSys.com_ok = OK;
  }
}

void LeptrinoNode::App_Close(rclcpp::Logger logger)
{
  RCLCPP_DEBUG(logger, "Application close\n");

  if (gSys.com_ok == OK)
  {
    Comm_Close();
  }
}

ULONG LeptrinoNode::SendData(UCHAR* pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR* pucWrite = &CommSendBuff[0];
  USHORT usRealSize;

  // データ整形
  *pucWrite = CHR_DLE;  // DLE
  pucWrite++;
  *pucWrite = CHR_STX;  // STX
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++)
  {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE)
    {                       // データが0x10ならば0x10を付加
      *pucWrite = CHR_DLE;  // DLE付加
      pucWrite++;           // 書き込み先
      usRealSize++;         // 実サイズ
      // BCCは計算しない!
    }
    *pucWrite = ucWork;  // データ
    ucBCC ^= ucWork;     // BCC
    pucWrite++;          // 書き込み先
    usRealSize++;        // 実サイズ
  }

  *pucWrite = CHR_DLE;  // DLE
  pucWrite++;
  *pucWrite = CHR_ETX;  // ETX
  ucBCC ^= CHR_ETX;     // BCC計算
  pucWrite++;
  *pucWrite = ucBCC;  // BCC付加
  usRealSize += 3;

  Comm_SendData(&CommSendBuff[0], usRealSize);

  return OK;
}

void LeptrinoNode::GetProductInfo(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor information");
  len = 0x04;                 // データ長
  SendBuff[0] = len;          // レングス
  SendBuff[1] = 0xFF;         // センサNo.
  SendBuff[2] = CMD_GET_INF;  // コマンド種別
  SendBuff[3] = 0;            // 予備

  SendData(SendBuff, len);
}

void LeptrinoNode::GetLimit(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor limit");
  len = 0x04;
  SendBuff[0] = len;            // レングス length
  SendBuff[1] = 0xFF;           // センサNo. Sensor no.
  SendBuff[2] = CMD_GET_LIMIT;  // コマンド種別 Command type
  SendBuff[3] = 0;              // 予備 reserve

  SendData(SendBuff, len);

  while (rclcpp::ok())
  {
    Comm_Rcv();

    if (Comm_CheckRcv() != 0)
    {  // 受信データ有
      CommRcvBuff[0] = 0;

      auto rt = Comm_GetRcvData(CommRcvBuff);

      if (rt > 0)
      {
        stGetLimit = (ST_R_LEP_GET_LIMIT*)CommRcvBuff;
        for (int i = 0; i < FN_Num; i++)
        {
          RCLCPP_INFO(logger, "\tLimit[%d]: %f", i, stGetLimit->fLimit[i]);
          conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
        }
        break;
      }
    }
    rclcpp::sleep_for(1ms);
  }
}

void LeptrinoNode::SerialStart(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Start sensor");
  len = 0x04;                    // データ長
  SendBuff[0] = len;             // レングス
  SendBuff[1] = 0xFF;            // センサNo.
  SendBuff[2] = CMD_DATA_START;  // コマンド種別
  SendBuff[3] = 0;               // 予備

  SendData(SendBuff, len);
}

void LeptrinoNode::SerialStop(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Stop sensor\n");
  len = 0x04;                   // データ長
  SendBuff[0] = len;            // レングス
  SendBuff[1] = 0xFF;           // センサNo.
  SendBuff[2] = CMD_DATA_STOP;  // コマンド種別
  SendBuff[3] = 0;              // 予備

  SendData(SendBuff, len);
}

void LeptrinoNode::SensorCalibration(rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Calibrating sensor data\n");
  int counter = 0;
  const int max_counter = 100;
  double gain = 1.0 / (double)max_counter;

  offset.force = { 0, 0, 0 };
  offset.moment = { 0, 0, 0 };

  double temp_fx[max_counter], temp_fy[max_counter], temp_fz[max_counter];
  double temp_mx[max_counter], temp_my[max_counter], temp_mz[max_counter];

  while (counter < max_counter && rclcpp::ok())
  {
    Comm_Rcv();

    if (Comm_CheckRcv() != 0)
    {
      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));

      auto rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        grossForce = (ST_R_DATA_GET_F*)CommRcvBuff;

        temp_fx[counter] = std::isnan(grossForce->ssForce[0]) ? 0: (double)(grossForce->ssForce[0] * conversion_factor[0]);
        temp_fy[counter] = std::isnan(grossForce->ssForce[1]) ? 0: (double)(grossForce->ssForce[1] * conversion_factor[1]);
        temp_fz[counter] = std::isnan(grossForce->ssForce[2]) ? 0: (double)(grossForce->ssForce[2] * conversion_factor[2]);

        temp_mx[counter] = std::isnan(grossForce->ssForce[3]) ? 0: (double)(grossForce->ssForce[3] * conversion_factor[3]);
        temp_my[counter] = std::isnan(grossForce->ssForce[4]) ? 0: (double)(grossForce->ssForce[4] * conversion_factor[4]);
        temp_mz[counter] = std::isnan(grossForce->ssForce[5]) ? 0: (double)(grossForce->ssForce[5] * conversion_factor[5]);

        if (DEBUG)
        {
          RCLCPP_INFO(logger, "Calibrating %d: %f,%f,%f,%f,%f,%f", counter, temp_fx[counter], temp_fy[counter],
                      temp_fz[counter], temp_mx[counter], temp_my[counter], temp_mz[counter]);
        }

        counter++;
        rclcpp::sleep_for(1ms);
      }
    }
  }

  for (int inc = 0; inc < max_counter; inc++)
  {
    offset.force[1] += temp_fx[inc];
    offset.force[2] += temp_fy[inc];
    offset.force[3] += temp_fz[inc];

    offset.moment[1] += temp_mx[inc];
    offset.moment[2] += temp_my[inc];
    offset.moment[3] += temp_mz[inc];
  }

  offset.force[1] = offset.force[1] / max_counter;
  offset.force[2] = offset.force[2] / max_counter;
  offset.force[3] = offset.force[3] / max_counter;

  offset.moment[1] = offset.moment[1] / max_counter;
  offset.moment[2] = offset.moment[2] / max_counter;
  offset.moment[3] = offset.moment[3] / max_counter;

  RCLCPP_INFO(logger, "Offset-> Fx:%.3f, Fy:%.3f, Fz:%.3f, Mx:%.3f, My:%.3f, Mz:%.3f", offset.force[1], offset.force[2],
              offset.force[3], offset.moment[1], offset.moment[2], offset.moment[3]);

  RCLCPP_INFO(logger, "Calibrating done\n");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeptrinoNode>());
  rclcpp::shutdown();
  return 0;
}
