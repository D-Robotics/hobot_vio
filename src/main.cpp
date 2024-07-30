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

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <strstream>
#include <ctime>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "data_collection.h"
#include "HorizonVIO.h"

class ROS2Visualizer : public rclcpp::Node {
  using ROS_Image = sensor_msgs::msg::Image;
  using ROS_Path = nav_msgs::msg::Path;

public:
  static std::shared_ptr<ROS2Visualizer> GetInstance() {
    static auto instance = std::make_shared<ROS2Visualizer>();
    return instance;
  }

  ROS2Visualizer() :
  Node("horizon_vio"),
  is_pose_updated_(false),
  is_img_updated_(false),
  is_running_(true) {
    image_publisher_ = this->create_publisher<ROS_Image>(
            "horizon_vio/horizon_vio_track_image", 5);
    path_publisher_ = this->create_publisher<ROS_Path>(
            "horizon_vio/horizon_vio_path", 5);
    tf_msg_.header.frame_id = "global";

    tf_msg_.child_frame_id = "camera_color_optical_frame";
    image_header_.frame_id = "camera_color_optical_frame";

    draw_thread_ = std::make_shared<std::thread>(std::bind(
            &ROS2Visualizer::DrawTrajectory, this));
  }

  void PublishPose(const Eigen::Matrix<double, 3, 3>&R_GtoCi,
                   const Eigen::Matrix<double, 3, 1>&p_CioinG,
                   double ts,
                   const std::string &child_id) {
    {
      std::lock_guard<std::mutex> lck(state_mutex_);
      tf_msg_.header.stamp.sec = ts;
      tf_msg_.header.stamp.nanosec = (ts - tf_msg_.header.stamp.sec) * 1e9;
      rotationMatrix_ = R_GtoCi;
      translationMatrix_ = p_CioinG;
      is_pose_updated_ = true;
      tf_msg_.child_frame_id = child_id;
    }
  }

  void PublishRenderedImage(cv::Mat &img, double ts, const std::string &frame_id) {
    {
      std::lock_guard<std::mutex> lck(img_mtx_);
      trackHistoryImg_ = img;
      image_header_.stamp.sec = ts;
      image_header_.stamp.nanosec = (ts - image_header_.stamp.sec) * 1e9;
      image_header_.frame_id = frame_id;
      is_img_updated_ = true;
    }
  }

  void ShutDown() {
    if (!is_running_) return;
    is_running_ = false;
    draw_thread_->join();
    //  rclcpp::shutdown();
  }

private:
  std::mutex state_mutex_;
  bool is_pose_updated_;
  Eigen::Matrix<double, 3, 3> rotationMatrix_;
  Eigen::Matrix<double, 3, 1> translationMatrix_;

  nav_msgs::msg::Path path_msg_;
  geometry_msgs::msg::TransformStamped tf_msg_;

  bool is_img_updated_;
  std::mutex img_mtx_;
  std_msgs::msg::Header image_header_;
  cv::Mat trackHistoryImg_, currTrackedFeaturesImg_;

  rclcpp::Publisher<ROS_Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<ROS_Image>::SharedPtr image_publisher_;

  std::shared_ptr<std::thread> draw_thread_;
  std::atomic_bool is_running_;

private:
  void wrap_img(sensor_msgs::msg::Image &img_msg, const cv::Mat &mat) {
    cv_bridge::CvImage img_bridge;
    if (mat.channels() == 1) {
      img_bridge = cv_bridge::CvImage(image_header_, "mono8", mat);
    } else {
      img_bridge = cv_bridge::CvImage(image_header_, "bgr8", mat);
    }
    img_bridge.toImageMsg(img_msg);
  }

  tf2::Transform TransformFromMat () {
    tf2::Matrix3x3 tf_camera_rotation(
            rotationMatrix_(0, 0), rotationMatrix_(0, 1), rotationMatrix_(0, 2),
            rotationMatrix_(1, 0), rotationMatrix_(1, 1), rotationMatrix_(1, 2),
            rotationMatrix_(2, 0), rotationMatrix_(2, 1), rotationMatrix_(2, 2));
    tf2::Vector3 tf_camera_translation (translationMatrix_[0],
                                        translationMatrix_[1],
                                        translationMatrix_[2]);
//    const tf2::Matrix3x3 trans (0, 1, 0,
//                                        0, 0, -1,
//                                        -1, 0, 0);
//    tf_camera_rotation = tf_camera_rotation.transpose() * trans;
   // tf_camera_translation = tf_camera_translation;
    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
  }

  template<typename T>
  bool CheckWhetherAddPose(const geometry_msgs::msg::Pose &position,
          const T& t) {
    auto x = position.position.x - t.x;
    auto y = position.position.y - t.y;
    auto z = position.position.z - t.z;
    return (x * x + y * y + z * z) > 0.001;
  }

  void DrawTrajectory() {
    tf2::Transform tf_transform;
    geometry_msgs::msg::PoseStamped pose_msg;
    tf2_ros::TransformBroadcaster tf_broadcaster(this);

    while (is_running_) {
      //  Pub trajectory
      {
        std::lock_guard<std::mutex> lck(state_mutex_);
        if (is_pose_updated_) {
          tf_transform = TransformFromMat();
          tf_msg_.transform = tf2::toMsg(tf_transform);
          tf_broadcaster.sendTransform(tf_msg_);
          path_msg_.header = tf_msg_.header;
          if (path_msg_.poses.empty() || CheckWhetherAddPose(path_msg_.poses.back().pose,
                  tf_msg_.transform.translation)) {
            pose_msg.header = tf_msg_.header;
            pose_msg.pose.orientation = tf_msg_.transform.rotation;
            pose_msg.pose.position.x = tf_msg_.transform.translation.x;
            pose_msg.pose.position.y = tf_msg_.transform.translation.y;
            pose_msg.pose.position.z = tf_msg_.transform.translation.z;
            path_msg_.poses.push_back(pose_msg);
            path_publisher_->publish(path_msg_);
          }
          is_pose_updated_ = false;
        }
      }

      //  Pub rendered image
      {
        sensor_msgs::msg::Image img_msg;
        std::unique_lock<std::mutex> lck(img_mtx_);
        if (is_img_updated_) {
          wrap_img(img_msg, trackHistoryImg_);
          is_img_updated_ = false;
          lck.unlock();
          image_publisher_->publish(img_msg);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  time_t t = time(nullptr);
  struct tm* now = localtime(&t);
  std::stringstream timestream;
  timestream << "trans_quat_camera_" << std::setw(2) << std::setfill('0')
             << now->tm_hour << '_' << std::setw(2) << std::setfill('0')
             << now->tm_min << '_' << std::setw(2) << std::setfill('0')
             << now->tm_sec << ".txt";
  std::ofstream out_txt_file;
  out_txt_file.open(timestream.str(), std::ios::out | std::ios::trunc);
  out_txt_file << std::fixed;
  ROS2Visualizer::GetInstance();
  std::shared_ptr<Ros2SubNode> ros2SubNode;

  //  Step 1. Loading config file
  std::string tros_distro
      = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  std::string path_config = "/opt/tros/" + tros_distro +
    "/lib/hobot_vio/config/realsenseD435i.yaml";

  ROS2Visualizer::GetInstance()->declare_parameter("path_config", path_config);
  ROS2Visualizer::GetInstance()->get_parameter("path_config", path_config);

  ros2SubNode = std::make_shared<Ros2SubNode>();
  std::thread([&]() {
    rclcpp::spin(ros2SubNode);
    ros2SubNode->ShutDown();
  }).detach();

  auto vio = std::make_shared<HorizonVIO::HorizonVIOSystem>(path_config);

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  uint64 index_img_end = 0;
  cv::Mat image;
  int ret;
  uint64 image_ts;
  uint queue_size;
  while (rclcpp::ok()) {
    std::chrono::steady_clock::time_point tp1 = std::chrono::steady_clock::now();
    ret = ros2SubNode->GetImage(image_ts, image, queue_size);
    std::chrono::steady_clock::time_point tp2 = std::chrono::steady_clock::now();
    if (ret == 0) {
      std::deque<std::pair<uint64, std::shared_ptr<float>>> imu_datas;
      std::chrono::steady_clock::time_point tp3 = std::chrono::steady_clock::now();
      ret = ros2SubNode->GetImusByTimeStamp(image_ts, imu_datas);
      std::chrono::steady_clock::time_point tp4 = std::chrono::steady_clock::now();
      if (ret == 0) {
        for (const auto &imu_data : imu_datas) {
          auto imu_ptr = imu_data.second.get();
          double imu_ts = (double) imu_data.first / 1e9;
          HorizonVIO::IMU_MSG imuMsg{};
          imuMsg.timestamp = imu_ts;
          imuMsg.linear_acceleration[0] = imu_ptr[0];
          imuMsg.linear_acceleration[1] = imu_ptr[1];
          imuMsg.linear_acceleration[2] = imu_ptr[2];
          imuMsg.angle_velocity[0] = imu_ptr[3];
          imuMsg.angle_velocity[1] = imu_ptr[4];
          imuMsg.angle_velocity[2] = imu_ptr[5];
          vio->ReceiveImu(imuMsg);
        }
      }
      HorizonVIO::IMG_MSG img_msg{};
      std::chrono::steady_clock::time_point tp5 = std::chrono::steady_clock::now();
      img_msg.timestamp = (double) image_ts / 1e9;
      img_msg.image = image.clone();
      vio->ReceiveCamera(img_msg);
      std::chrono::steady_clock::time_point tp6 = std::chrono::steady_clock::now();
      ++index_img_end;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    HorizonVIO::Localization localization{};
    vio->GetLocalization(localization);
    if (localization.status == HorizonVIO::LocalizationStatus::VIO_TRACKING) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("horizon_vio_node"),
                         "Localization position[x, y, z]: ["
                                 << localization.positizon[0] << ", "
                                 << localization.positizon[1] << ", "
                                 << localization.positizon[2] << ']');
      RCLCPP_WARN_STREAM(rclcpp::get_logger("horizon_vio_node"),
                         "Image time "
                         << std::fixed << std::setprecision(9) << localization.timestamp);
      out_txt_file << std::setprecision(9) << localization.timestamp << " ";
      out_txt_file << std::setprecision(6)
                   << localization.positizon[0] << " "
                   << localization.positizon[1] << " "
                   << localization.positizon[2] << " "
                   << localization.quat[0] << " "
                   << localization.quat[1] << " "
                   << localization.quat[2] << " "
                   << localization.quat[3]
                   << std::endl;
    }
    Eigen::Quaterniond Q(localization.quat[3],
                         localization.quat[0],
                         localization.quat[1],
                         localization.quat[2]);
    Eigen::Matrix<double, 3, 1> T(localization.positizon[0],
                                  localization.positizon[1],
                                  localization.positizon[2]);
    ROS2Visualizer::GetInstance()->PublishPose(
            Q.toRotationMatrix(), T, localization.timestamp,
            ros2SubNode->GetImageFrame());
    std::chrono::steady_clock::time_point tp8 = std::chrono::steady_clock::now();

//    std::cout << "2-1: " << std::chrono::duration_cast<
//            std::chrono::milliseconds>(tp2 - tp1).count() << std::endl;
//    std::cout << "3-2: " << std::chrono::duration_cast<
//            std::chrono::milliseconds>(tp3 - tp2).count() << std::endl;
//    std::cout << "4-3: " << std::chrono::duration_cast<
//            std::chrono::milliseconds>(tp4 - tp3).count() << std::endl;
//    std::cout << "5-4: " << std::chrono::duration_cast<
//            std::chrono::milliseconds>(tp5 - tp4).count() << std::endl;
//    std::cout << "6-5: " << std::chrono::duration_cast<
//            std::chrono::milliseconds>(tp6 - tp5).count() << std::endl;
  }
  out_txt_file.close();
  rclcpp::shutdown();
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  RCLCPP_WARN_STREAM(rclcpp::get_logger("horizon_vio_node"),
          "Time for the total: " << std::chrono::duration_cast<
                    std::chrono::duration<double> >(t2 - t1).count() << "s" );
  RCLCPP_WARN_STREAM(rclcpp::get_logger("horizon_vio_node"),
          std::setprecision(6) << "avg time : " << std::chrono::duration_cast<
          std::chrono::duration<double> >(t2 - t1).count() / index_img_end);
  ROS2Visualizer::GetInstance()->ShutDown();
  return 0;
}
