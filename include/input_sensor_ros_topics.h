/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_ROS_TOPICS_H
#define INPUT_SENSOR_ROS_TOPICS_H

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.h>

#include <queue>

#include "adi_sensor_frame_info.h"
#include "input_sensor.h"

/**
 * @brief This is input class for sensor as camera
 *
 */
class InputSensorRosTopic : public IInputSensor
{
public:
  void openSensor(
    std::string sensor_name, int input_image_width, int input_image_height, int processing_scale,
    std::string config_file_name);
  void configureSensor(std::string frame_type);
  void getIntrinsics(CameraIntrinsics * camera_intrinsics);
  void getExtrinsics(CameraExtrinsics * camera_extrinsics);
  bool readNextFrame(unsigned short * depth_frame, unsigned short * ir_frame);
  bool getFrameTimestamp(rclcpp::Time * timestamp);
  void closeSensor();
  void syncDepthandIr(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam);
  void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

  void IrCallback(const sensor_msgs::msg::Image::ConstSharedPtr & ir_image);
  void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_image);

  /**
   * @brief Sets ABinvalidation threshold value
   * @return int
   */
  void setABinvalidationThreshold(int /* threshold */)
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }
  /**
   * @brief Sets Confidence threshold value
   * @return int
   */
  void setConfidenceThreshold(int /* threshold */)
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

private:
  std::string in_file_name_;
  int total_frames_ = 0;

  std::queue<ADISensorFrameInfo *> ros_topics_input_frames_queue_;
  // Subscribe to camera-info, depth, ir and point-cloud topics
  std::string camera_info_topic_name;
  std::string depth_image_topic_name;
  std::string ir_image_topic_name;
  std::mutex ros_topics_input_thread_mtx_;
  bool camera_parameters_updated_ = false;
  CameraIntrinsics camera_intrinsics_;
  rclcpp::Time frame_timestamp_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> ir_image_subscriber_;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    sync_policy_depth_ir_image;
  typedef message_filters::Synchronizer<sync_policy_depth_ir_image> depth_ir_image_synchronizer;
  std::shared_ptr<depth_ir_image_synchronizer> depth_ir_image_sync_ptr_;
  rclcpp::Node::SharedPtr private_node_handle_;
};

#endif