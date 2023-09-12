/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_ROS_TOPICS_H
#define INPUT_SENSOR_ROS_TOPICS_H

#include "input_sensor.h"
#include "adi_sensor_frame_info.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/time_synchronizer.h>
#include <queue>

/**
 * @brief This is input class for sensor as camera
 *
 */
class InputSensorRosTopic : public IInputSensor
{
public:
  void openSensor(std::string sensor_name, int input_image_width, int input_image_height, int processing_scale,
                  std::string config_file_name);
  void configureSensor(std::string frame_type);
  void getIntrinsics(CameraIntrinsics* camera_intrinsics);
  void getExtrinsics(CameraExtrinsics* camera_extrinsics);
  bool readNextFrame(unsigned short* depth_frame, unsigned short* ir_frame);
  bool getFrameTimestamp(ros::Time* timestamp);
  void closeSensor();
  void syncDepthandIr(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& ir_image);
  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);

  void IrCallback(const sensor_msgs::ImageConstPtr& ir_image);
  void DepthCallback(const sensor_msgs::ImageConstPtr& depth_image);

  /**
   * @brief Sets ABinvalidation threshold value
   * @return int
   */
  void setABinvalidationThreshold(int threshold)
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }
  /**
   * @brief Sets Confidence threshold value
   * @return int
   */
  void setConfidenceThreshold(int threshold)
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

private:
  std::string in_file_name_;
  int total_frames_ = 0;

  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> ir_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_;
  std::queue<ADISensorFrameInfo*> ros_topics_input_frames_queue_;

  // Subscribe to camera-info, depth, ir and point-cloud topics
  std::string camera_info_topic_name;
  std::string depth_image_topic_name;
  std::string ir_image_topic_name;
  boost::mutex ros_topics_input_thread_mtx_;
  bool camera_parameters_updated_ = false;
  CameraIntrinsics camera_intrinsics_;
  ros::Time frame_timestamp_;
};

#endif