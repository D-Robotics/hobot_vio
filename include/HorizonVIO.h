// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace HorizonVIO {

struct GNSS_MSG {
  double timestamp;
  double longitude;
  double latitude;
  double altitude;
  double velocity[3]; //  三维速度
  double quat[4]; //  四元数
  int status; //  定位状态
  double vdop, hdop; //  定位精度因子
};

struct WHEEL_MSG {
  double timestamp;
  double wheel_velocity[4];
};

struct IMU_MSG {
  double timestamp;
  double linear_acceleration[3]; //  x, y, z, unit: m/s^2
  double angle_velocity[3];  //  x, y, z, unit: rad/s
};

struct IMG_MSG {
  double timestamp;
  cv::Mat image;
};

enum LocalizationStatus {
  VIO_UNINITIALIZED = 0,
  VIO_TRACKING = 1,
  VIO_LOST
};

enum LocalizationMode {
  VIO_LOCAL_MODE = 0,
  VIO_GLOBAL_MODE
};

struct Localization {
  double timestamp;
  double positizon[3];  //  位置
  double velocity[3];  //  速度
  double quat[4];  //  四元数姿态
  enum LocalizationStatus status;  //  定位状态
  enum LocalizationMode mode; // 定位模式，包括局部定位和全局定位
  //  q, p, v, bg, ba, dt, q p(extrinsics),
  //  fx fy cx cy k1 k2 p1 p2(intrinsics) 30 dims total
  cv::Mat cov;
};

class HorizonVIOSystem {
public:
  HorizonVIOSystem(const std::string &config_path);
  ~HorizonVIOSystem();
  int ReceiveImu(const IMU_MSG &imu_msg);
  int ReceiveWheel(const WHEEL_MSG &wheel_msg);
  int ReceiveGnss(const GNSS_MSG &gnss_msg);
  int ReceiveCamera(const IMG_MSG &img_msg);
  int GetLocalization(Localization &localization);
  int Reset();

private:
  void* private_ptr_ = nullptr;//  私有变量
};

}
