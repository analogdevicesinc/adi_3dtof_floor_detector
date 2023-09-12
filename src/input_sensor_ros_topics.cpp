/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "adi_sensor_frame_info.h"
#include "input_sensor_ros_topics.h"
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

// Sync message for Depth,IR
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_depth_ir;

#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 2

/**
 * @brief subscribes the correct topics to take rostopics as input to algorithm.
 *
 * @param cam_prefix camera prefix name of the input ros topics.
 * @param input_image_width image width
 * @param input_image_height image height
 * @param processing_scale scale factor is not used as the device is not the chosen input mode.
 * @param config_file_name config file name of ToF SDK is not used as the device is not the chosen input mode.
 */
void InputSensorRosTopic::openSensor(std::string cam_prefix, int input_image_width, int input_image_height,
                                     int /*processing_scale*/, std::string /*config_file_name*/)
{
  ros::NodeHandle nh("~");
  frame_counter_ = 0;
  sensor_open_flag_ = false;
  camera_parameters_updated_ = false;

  frame_width_ = input_image_width;
  frame_height_ = input_image_height;
  // Frame Size is directly set to 512x512 using camera info, therefore scale factor is manually set to 1 here.
  input_scale_factor_ = 1;

  // Subscribe to camera-info, depth, ir and point-cloud topics
  camera_info_topic_name = "/" + cam_prefix + "/camera_info";
  depth_image_topic_name = "/" + cam_prefix + "/depth_image";
  ir_image_topic_name = "/" + cam_prefix + "/ir_image";

  camera_info_subscriber_.subscribe(nh, camera_info_topic_name, 5);
  depth_image_subscriber_.subscribe(nh, depth_image_topic_name, 5);
  ir_image_subscriber_.subscribe(nh, ir_image_topic_name, 5);

  camera_info_subscriber_.registerCallback(boost::bind(&InputSensorRosTopic::camInfoCallback, this, _1));

  // We need to sync the depth and ir topics
  static message_filters::Synchronizer<sync_depth_ir> sync_depth_and_ir(sync_depth_ir(MAX_QUEUE_SIZE_FOR_TIME_SYNC));
  sync_depth_and_ir.connectInput(depth_image_subscriber_, ir_image_subscriber_);
  sync_depth_and_ir.registerCallback(boost::bind(&InputSensorRosTopic::syncDepthandIr, this, _1, _2));

  // Update flag.
  sensor_open_flag_ = true;

  // Initialize frame timestamp
  frame_timestamp_ = ros::Time::now();
}

/**
 * @brief camera info call back.
 *
 * @param cam_info camera info pointer
 */
void InputSensorRosTopic::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if (!camera_parameters_updated_)
  {
    // Save
    int frame_width = cam_info->width;
    int frame_height = cam_info->height;
    if ((frame_width != 512) || (frame_height != 512))
    {
      return;
    }
    setFrameWidth(frame_width);
    setFrameHeight(frame_height);

    camera_intrinsics_.camera_matrix[0] = cam_info->K[0];
    camera_intrinsics_.camera_matrix[1] = 0.0f;
    camera_intrinsics_.camera_matrix[2] = cam_info->K[2];
    camera_intrinsics_.camera_matrix[3] = 0.0f;
    camera_intrinsics_.camera_matrix[4] = cam_info->K[4];
    camera_intrinsics_.camera_matrix[5] = cam_info->K[5];
    camera_intrinsics_.camera_matrix[6] = 0.0f;
    camera_intrinsics_.camera_matrix[7] = 0.0f;
    camera_intrinsics_.camera_matrix[8] = 1.0f;

    for (int i = 0; i < 8; i++)
    {
      camera_intrinsics_.distortion_coeffs[i] = cam_info->D[i];
    }
    camera_parameters_updated_ = true;
  }
}

/**
 * @brief synchronizes depth and IR images
 *
 * @param depth_image depth image pointer
 * @param ir_image ir image pointer
 */
void InputSensorRosTopic::syncDepthandIr(const sensor_msgs::ImageConstPtr& depth_image,
                                         const sensor_msgs::ImageConstPtr& ir_image)
{
  // Copy the buffers into the queue
  int image_width = depth_image->width;

  if (!camera_parameters_updated_)
  {
    return;
  }
  int image_height = depth_image->height;
  if ((image_width != frame_width_) || (image_height != frame_height_))
  {
    return;
  }

  int ir_image_width = ir_image->width;
  int ir_image_height = ir_image->height;
  if ((ir_image_width != frame_width_) || (ir_image_height != frame_height_))
  {
    return;
  }

  // Get a new node
  ADISensorFrameInfo* frame_info_node = new ADISensorFrameInfo(image_width, image_height);

  // copy depth image
  memcpy(frame_info_node->depth_image_, &depth_image->data[0], image_width * image_height * 2);
  memcpy(frame_info_node->ir_image_, &ir_image->data[0], image_width * image_height * 2);
  // Extracting timestamp
  frame_info_node->frame_timestamp_ = depth_image->header.stamp;

  // Add new node to the queue
  ros_topics_input_thread_mtx_.lock();
  ros_topics_input_frames_queue_.push(frame_info_node);
  ros_topics_input_thread_mtx_.unlock();
}

/**
 * @brief Configures the sensor
 *
 * @param frame_type frame type, not used in file-io mode
 */
void InputSensorRosTopic::configureSensor(std::string /*frame_type*/)
{
  total_frames_ = 0;
  if (sensor_open_flag_)
  {
    // Wait for the cameraInfo callback and update the camera parameters.
    while (!camera_parameters_updated_)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
      ROS_INFO_STREAM("Waiting for camera Info..");
      ros::spinOnce();
    }
  }

  return;
}

/**
 * @brief Gets the camera intrinsics
 *
 * @param camera_intrinsics camera intrinsics
 */
void InputSensorRosTopic::getIntrinsics(CameraIntrinsics* camera_intrinsics)
{
  *camera_intrinsics = camera_intrinsics_;

  return;
}

/**
 * @brief gets camera extrinsics
 *
 * @param camera_extrinsics camera extrinsics
 */
void InputSensorRosTopic::getExtrinsics(CameraExtrinsics* camera_extrinsics)
{
  camera_extrinsics->rotation_matrix[0] = 1.0f;
  camera_extrinsics->rotation_matrix[1] = 0.0f;
  camera_extrinsics->rotation_matrix[2] = 0.0f;
  camera_extrinsics->rotation_matrix[3] = 0.0f;
  camera_extrinsics->rotation_matrix[4] = 1.0f;
  camera_extrinsics->rotation_matrix[5] = 0.0f;
  camera_extrinsics->rotation_matrix[6] = 0.0f;
  camera_extrinsics->rotation_matrix[7] = 0.0f;
  camera_extrinsics->rotation_matrix[8] = 1.0f;

  camera_extrinsics->translation_matrix[0] = 0.0f;
  camera_extrinsics->translation_matrix[1] = 0.0f;
  camera_extrinsics->translation_matrix[2] = 0.0f;
}

/**
 * @brief reads next frame
 *
 * @param depth_frame pointer to read depth frame
 * @param ir_frame pointer to read ir frame
 * @return true if reading next frame is successful.
 * @return false if reading next frame is failure.
 */
bool InputSensorRosTopic::readNextFrame(unsigned short* depth_frame, unsigned short* ir_frame)
{
  ROS_ASSERT(depth_frame != nullptr);
  ROS_ASSERT(ir_frame != nullptr);

  // Read the next node from queue and copy
  ros_topics_input_thread_mtx_.lock();
  int queue_size = ros_topics_input_frames_queue_.size();
  ros_topics_input_thread_mtx_.unlock();
  // Wait till we have buffers in the queue
  // Do not wait for more than 10 secs.. if it times out assume the end of input
  bool time_out_flag = false;
  ros::Time time_begin = ros::Time::now();
  while ((queue_size <= 0) && (!time_out_flag))
  {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    ros::Time current_time = ros::Time::now();
    ros::Duration wait_duration = current_time - time_begin;
    if (wait_duration.toSec() > 10)
    {
      time_out_flag = true;
    }
    ros_topics_input_thread_mtx_.lock();
    queue_size = ros_topics_input_frames_queue_.size();
    ros_topics_input_thread_mtx_.unlock();
    // ROS_INFO_STREAM("Wait duration : " << wait_duration.toSec());
    ros::spinOnce();
  }
  if (!time_out_flag)
  {
    ros_topics_input_thread_mtx_.lock();
    // Get the node
    ADISensorFrameInfo* new_frame = (ADISensorFrameInfo*)ros_topics_input_frames_queue_.front();
    // Copy contents
    memcpy(depth_frame, new_frame->depth_image_, sizeof(new_frame->depth_image_[0]) * frame_width_ * frame_height_);
    memcpy(ir_frame, new_frame->ir_image_, sizeof(new_frame->ir_image_[0]) * frame_width_ * frame_height_);
    // Extract timestamp
    frame_timestamp_ = new_frame->frame_timestamp_;

    // remove the node from queue
    ros_topics_input_frames_queue_.pop();
    delete new_frame;
    ros_topics_input_thread_mtx_.unlock();
  }

  ++frame_counter_;

  return true;
}

/**
 * @brief gets the frame timestamp from the rostopics
 *
 */
bool InputSensorRosTopic::getFrameTimestamp(ros::Time* timestamp)
{
  *timestamp = frame_timestamp_;
  return true;
}

/**
 * @brief closes the input file
 *
 */
void InputSensorRosTopic::closeSensor()
{
}
