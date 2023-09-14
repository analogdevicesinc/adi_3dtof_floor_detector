/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "adi_3dtof_floor_detector_example.h"
#include <utility>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This function shuts down all the nodes running in a roscore
 *
 */
void ADI3DToFFloorDetectorExample::shutDownAllNodes()
{
  int status = system("rosnode kill -a");
  if (status < 0)
  {
    ROS_INFO_STREAM("Error in \"rosnode kill -a\": " << status);
  }
  ros::shutdown();
}

/**
 * @brief Callback for camera info message
 *
 * @param cam_info - Cam Info Pointer
 */
void ADI3DToFFloorDetectorExample::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  // Save camera info only once
  if (!camera_parameters_updated_)
  {
    // Save
    image_width_ = cam_info->width;
    image_height_ = cam_info->height;
    if ((image_width_ != 512) || (image_height_ != 512))
    {
      ROS_INFO_STREAM("Image size is not set to 512x512");
      return;
    }

    // Check whether original or modified camera intrinsic are sent
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

    image_proc_utils_ = new ImageProcUtils(&camera_intrinsics_, image_width_, image_height_);
    camera_parameters_updated_ = true;
  }
}

/**
 * @brief Callback to depth image
 *
 * @param depth_image Depth image message
 */
void ADI3DToFFloorDetectorExample::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image)
{
  // Depth Image
  if ((depth_image->width != image_width_) || (depth_image->height != image_height_))
  {
    ROS_INFO("Depth image dimension is not matching with camera info.");
    return;
  }
  // Memory Allocation for depth image
  if (depth_image_ == nullptr)
  {
    depth_image_ = new unsigned short[image_width_ * image_height_ * 2];
  }
  // copy
  memcpy(depth_image_, &depth_image->data[0], image_width_ * image_height_ * 2);
  depth_image_recvd_ = true;

  depth_image_topic_ = depth_image;
}

/**
 * @brief Callback to ir image
 *
 * @param ir_image IR image message
 */
void ADI3DToFFloorDetectorExample::irImageCallback(const sensor_msgs::ImageConstPtr& ir_image)
{
  // IR Image
  if ((ir_image->width != image_width_) || (ir_image->height != image_height_))
  {
    ROS_INFO("IR image dimension is not matching with camera info.");
    return;
  }
  // Memory Allocation for ir image
  if (ir_image_ == nullptr)
  {
    ir_image_ = new unsigned short[image_width_ * image_height_ * 2];
  }
  // copy
  memcpy(ir_image_, &ir_image->data[0], image_width_ * image_height_ * 2);
  ir_image_recvd_ = true;
}

/**
 * @brief Callback to floor mask image
 *
 * @param floor_mask_image floor mask image message
 */
void ADI3DToFFloorDetectorExample::floorMaskImageCallback(const sensor_msgs::ImageConstPtr& floor_mask_image)
{
  // Floor mask image
  if ((floor_mask_image->width != image_width_) || (floor_mask_image->height != image_height_))
  {
    ROS_INFO("Floor mask image dimension is not matching with camera info.");
    return;
  }
  // Memory Allocation for floor mask image
  if (floor_mask_image_ == nullptr)
  {
    floor_mask_image_ = new unsigned char[image_width_ * image_height_];
  }
  // copy
  memcpy(floor_mask_image_, &floor_mask_image->data[0], image_width_ * image_height_);
  floor_mask_image_recvd_ = true;
}

/**
 * @brief Callback to compressed floor mask image
 *
 * @param compressed_floor_mask_image Compressed floor mask image message
 */
void ADI3DToFFloorDetectorExample::compressedFloorMaskCallback(
    const sensor_msgs::CompressedImageConstPtr& compressed_floor_mask_image)
{
  if (compressed_floor_mask_image == nullptr)
  {
    ROS_INFO("Compressed floor mask image is empty");
    return;
  }

  int* image_width = (int*)&compressed_floor_mask_image->data[sizeof(compressed_depth_image_transport::ConfigHeader)];
  int* image_height =
      (int*)&compressed_floor_mask_image->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 4];
  int num_pixels = (*image_width) * (*image_width);

  if ((*image_width != image_width_) || (*image_height != image_height_))
  {
    ROS_INFO("Decompressed Depth image dimension is not matching with camera info.");
    return;
  }

  unsigned char* compressed_image_buf =
      (unsigned char*)&compressed_floor_mask_image->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 8];

  // Memory Allocation for round mask image
  if (floor_mask_image_ == nullptr)
  {
    floor_mask_image_ = new unsigned char[image_width_ * image_height_];
  }

  // Decompress
  for (int i = 0; i < num_pixels; i++)
  {
    int val = (compressed_image_buf[i / 8]) & (1 << (i % 8));
    if (val != 0)
    {
      floor_mask_image_[i] = 255;
    }
    else
    {
      floor_mask_image_[i] = 0;
    }
  }

  floor_mask_image_recvd_ = true;
}

/**
 * @brief Generate floor removed point cloud output from depth image
 *
 */
void ADI3DToFFloorDetectorExample::generateFloorRemovedPointCloud()
{
  // Compute point cloud
  if (xyz_image_ == nullptr)
  {
    xyz_image_ = new short[image_width_ * image_height_ * 3];
  }
  image_proc_utils_->computePointCloud(depth_image_, xyz_image_);

  ros::Time stamp;
  std::string frame_id;
  if (depth_image_topic_ != nullptr)
  {
    stamp = depth_image_topic_->header.stamp;
    frame_id = depth_image_topic_->header.frame_id;
  }

  // Transform to map
  // Get the transform wrt to "map"
  sensor_msgs::PointCloud2::Ptr point_cloud = convert2ROSPointCloudMsg(xyz_image_, stamp, frame_id);
  tf_buffer_.canTransform("map", point_cloud->header.frame_id, point_cloud->header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud("map", *point_cloud, transformed_pc_, tf_buffer_);
}

/**
 * @brief Converts point cloud to ROS message format and also removing the floor from point cloud
 *
 * @param xyz_frame point cloud
 * @param stamp time stamp
 * @param frame_id frame ID
 * @return sensor_msgs::PointCloud2::Ptr ROS message of point cloud
 */
sensor_msgs::PointCloud2::Ptr ADI3DToFFloorDetectorExample::convert2ROSPointCloudMsg(short* xyz_frame, ros::Time stamp,
                                                                                     std::string frame_id)
{
  sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);

  pointcloud_msg->header.seq = frame_counter_;
  pointcloud_msg->header.stamp = stamp;
  pointcloud_msg->header.frame_id = std::move(frame_id);
  pointcloud_msg->width = image_width_;
  pointcloud_msg->height = image_height_;
  pointcloud_msg->is_dense = false;
  pointcloud_msg->is_bigendian = false;

  // XYZ data from sensor.
  // This data is in 16 bpp format.
  short* xyz_sensor_buf;
  xyz_sensor_buf = xyz_frame;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  unsigned char* floor_mask_image_runner = &floor_mask_image_[0];

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
  for (int i = 0; i < image_height_; i++)
  {
    for (int j = 0; j < image_width_; j++)
    {
      // Floor Removal
      if (*floor_mask_image_runner == 255)
      {
        *iter_x = 0.0f;
        *iter_y = 0.0f;
        *iter_z = 0.0f;

        xyz_sensor_buf += 3;
      }
      else
      {
        *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
      }
      ++floor_mask_image_runner;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }
  return pointcloud_msg;
}

/**
 * @brief Callback to synchronize depth, ir, floor mask images
 *
 * @param depth_image depth image
 * @param floor_mask_image floor mask image
 */
void ADI3DToFFloorDetectorExample::syncCallback(const sensor_msgs::ImageConstPtr& depth_image,
                                                const sensor_msgs::ImageConstPtr& floor_mask_image)
{
  // Camera Info
  if (!camera_parameters_updated_)
  {
    ROS_INFO("Skipping the frame as camera info is not updated.");
    return;
  }

  depthImageCallback(depth_image);
  floorMaskImageCallback(floor_mask_image);
}

/**
 * @brief Callback to synchronize compressed depth, ir, floor mask images
 *
 * @param compressed_depth_image Compressed depth image
 * @param compressed_floor_mask_image Compressed floor mask image
 */
void ADI3DToFFloorDetectorExample::syncCompressedCallback(
    const sensor_msgs::ImageConstPtr& depth_image,
    const sensor_msgs::CompressedImageConstPtr& compressed_floor_mask_image)
{
  // Camera Info
  if (!camera_parameters_updated_)
  {
    ROS_INFO("Skipping the frame as camera info is not updated.");
    return;
  }

  depthImageCallback(depth_image);
  compressedFloorMaskCallback(compressed_floor_mask_image);
}

/**
 * @brief Gives out floor detection outputs
 *
 * @param depth_image_op_16bit Floor removed depth image
 * @param floor_detection_op_8bit Floor marked depth image
 */
void ADI3DToFFloorDetectorExample::getFloorDetectionOutput(cv::Mat& depth_image_op_16bit,
                                                           cv::Mat& floor_detection_op_8bit)
{
  // Callback Images
  cv::Mat depth_image_16bit = cv::Mat(image_height_, image_width_, CV_16UC1, &depth_image_[0]);
  cv::Mat floor_mask_image_8bit = cv::Mat(cv::Size(image_width_, image_height_), CV_8UC1, &floor_mask_image_[0]);

  // Invert Floor mask
  cv::Mat invert_floor_mask_image_8bit = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
  cv::bitwise_not(floor_mask_image_8bit, invert_floor_mask_image_8bit);

  // convert 8bit floor mask to 16bit
  cv::Mat invert_floor_mask_image_16bit = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_16UC1);
  float floor_scale_factor = 65535.0f / 255;
  invert_floor_mask_image_8bit.convertTo(invert_floor_mask_image_16bit, CV_16UC1, floor_scale_factor, 0);

  // Floor removed 16bit Depth Image
  cv::bitwise_and(depth_image_16bit, invert_floor_mask_image_16bit, depth_image_op_16bit);

  // Floor Detection 8bit RGB Output Image
  // convert 16bit depth image to 8bit
  cv::Mat depth_image_8bit = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
  unsigned short max_element = 8192;
  float scale_factor = 255.0f / max_element;
  depth_image_16bit.convertTo(depth_image_8bit, CV_8UC1, scale_factor, 0);

  // convert 8bit depth image to rgb
  cv::cvtColor(depth_image_8bit, floor_detection_op_8bit, cv::COLOR_GRAY2BGR);

  // Only if floor exists in the depth image
  if (cv::countNonZero(floor_mask_image_8bit) > 0)
  {
    // Get floor marked output
    cv::Mat channels[3];
    cv::split(floor_detection_op_8bit, channels);
    cv::bitwise_or(channels[1], floor_mask_image_8bit, channels[1]);
    cv::merge(channels, 3, floor_detection_op_8bit);
  }
}

/**
 * @brief Generate outputs for the received depth, floor mask images
 *
 * @return true if outputs are generated
 * @return false if outputs are not generated
 */
bool ADI3DToFFloorDetectorExample::generateOutput()
{
  // Make sure we have received frames from the sensor
  bool all_callbacks_recvd = true;
  if ((!depth_image_recvd_) || (!floor_mask_image_recvd_))
  {
    all_callbacks_recvd = false;
  }

  if (all_callbacks_recvd)
  {
    if (enable_pointcloud_output_)
    {
      generateFloorRemovedPointCloud();
      floor_removed_pointcloud_publisher_.publish(transformed_pc_);
    }

    // Generation of Floor Detection Outputs
    cv::Mat depth_image_op_16bit = cv::Mat::zeros(image_height_, image_width_, CV_16UC1);
    cv::Mat floor_detection_op_8bit = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);

    getFloorDetectionOutput(depth_image_op_16bit, floor_detection_op_8bit);

    // Publish
    publishImageAsRosMsg(depth_image_op_16bit, "mono16", "floor_removed_depth_image",
                         floor_removed_depth_image_publisher_);
    publishImageAsRosMsg(floor_detection_op_8bit, enc::BGR8, "floor_marked_depth_image",
                         floor_marked_depth_image_publisher_);

    // Reset flags
    floor_mask_image_recvd_ = false;
    depth_image_recvd_ = false;

    // Update frame count
    frame_counter_++;
  }
  // If all callbacks are not received, just skip the frame and move forward.
  return true;
}

/**
 * @brief This function publishes images as Ros messages.
 *
 * @param img This is input image
 * @param encoding_type number of bits used to represent one pixel of image.
 * @param frame_id frame id of image
 * @param publisher This is ros publisher
 */
void ADI3DToFFloorDetectorExample::publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type,
                                                        std::string frame_id, const ros::Publisher& publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = encoding_type;
  cv_ptr->header.seq = frame_counter_;
  cv_ptr->header.stamp = curr_frame_timestamp_;
  cv_ptr->header.frame_id = std::move(frame_id);
  cv_ptr->image = std::move(img);

  publisher.publish(cv_ptr->toImageMsg());
}

/**
 * @brief Entry to the Example code
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int 0 - Success
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "floor detection output generation");
  ros::NodeHandle nh("~");

  // ROS topic prefix
  std::string cam_prefix;
  nh.param<std::string>("param_rostopic_cam_prefix", cam_prefix, "cam1");

  // 0-Disable, 1 - Generate point cloud from depth image
  int enable_pointcloud_output = 0;
  nh.param<int>("param_enable_pointcloud_output", enable_pointcloud_output, 0);

  try
  {
    // Create an instance of Floor Detector Example
    ADI3DToFFloorDetectorExample floor_detector_example(nh, cam_prefix, enable_pointcloud_output);
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
      ROS_INFO("adi_3dtof_floor_detector output generation::Running loop");
      /*Generate host outputs*/
      if (!floor_detector_example.generateOutput())
      {
        floor_detector_example.shutDownAllNodes();
      }

      loop_rate.sleep();

      ros::spinOnce();
    }

    ros::shutdown();
  }
  catch (const std::exception& e)
  {
    std::cerr << " Exception in constructor of floor_detector_example : " << e.what() << '\n';
  }

  return 0;
}