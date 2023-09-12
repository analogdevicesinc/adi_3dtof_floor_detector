/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_3DTOF_FLOOR_DETECTOR_EXAMPLE_H
#define ADI_3DTOF_FLOOR_DETECTOR_EXAMPLE_H

#include "image_proc_utils.h"
#include <ros/ros.h>
#include <utility>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <compressed_depth_image_transport/compression_common.h>

#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 2

/**
 * @brief This is main class for the Stitch node
 *
 *
 */
class ADI3DToFFloorDetectorExample : public ros::NodeHandle
{
public:
  /**
   * @brief Construct a new ADI3DToFFloorDetectorExample object
   *
   * @param nh ROS node handle
   * @param cam_prefix Camera Prefix
   * @param enable_pointcloud_output Enable floor removed point cloud generation
   */
  ADI3DToFFloorDetectorExample(ros::NodeHandle& nh, std::string cam_prefix, int enable_pointcloud_output) : it_(nh)
  {
    ROS_INFO("ADI3DToF Floor Detector output generation::Inside ADI3DToFFloorDetectorExample()");

    enable_pointcloud_output_ = enable_pointcloud_output;
    depth_image_topic_ = nullptr;

    // Topics : camera-info, depth, ir and floor mask
    camera_info_topic_name_ = "/" + cam_prefix + "/camera_info";
    depth_image_topic_name_ = "/" + cam_prefix + "/depth_image";
    // ir_image_topic_name_ = "/" + cam_prefix + "/ir_image";
    floor_mask_topic_name_ = "/" + cam_prefix + "/floor_mask";
    compressed_floor_mask_topic_name_ = "/" + cam_prefix + "/compressed_floor_mask";

    sub_cam_info_ = this->subscribe<sensor_msgs::CameraInfo>(
        camera_info_topic_name_, 1, boost::bind(&ADI3DToFFloorDetectorExample::camInfoCallback, this, _1));

    depth_image_subscriber_.subscribe(it_, depth_image_topic_name_, 1);
    // ir_image_subscriber_.subscribe(nh, ir_image_topic_name, 1);
    floor_mask_subscriber_.subscribe(nh, floor_mask_topic_name_, 1);
    sync_shared_ptr_.reset(new sync_(sync_policy_depth_floor_(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_,
                                     floor_mask_subscriber_));
    sync_shared_ptr_->registerCallback(boost::bind(&ADI3DToFFloorDetectorExample::syncCallback, this, _1, _2));

    compressed_floor_mask_subscriber_.subscribe(nh, compressed_floor_mask_topic_name_, 1);
    sync_compressed_shared_ptr_.reset(
        new sync_compressed_(sync_policy_compressed_depth_ir_floor_(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
                             depth_image_subscriber_, compressed_floor_mask_subscriber_));
    sync_compressed_shared_ptr_->registerCallback(
        boost::bind(&ADI3DToFFloorDetectorExample::syncCompressedCallback, this, _1, _2));

    // Create TF listerner instance
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    // Flags for received topics
    camera_parameters_updated_ = false;
    depth_image_recvd_ = false;
    ir_image_recvd_ = false;
    floor_mask_image_recvd_ = false;

    // Images
    depth_image_ = nullptr;
    ir_image_ = nullptr;
    floor_mask_image_ = nullptr;
    xyz_image_ = nullptr;

    // init
    frame_counter_ = 0;

    // Create publishers.
    floor_removed_depth_image_publisher_ = this->advertise<sensor_msgs::Image>("floor_removed_depth_image", 10);
    floor_marked_depth_image_publisher_ = this->advertise<sensor_msgs::Image>("floor_marked_depth_image", 10);
    if (enable_pointcloud_output_)
    {
      floor_removed_pointcloud_publisher_ = this->advertise<sensor_msgs::PointCloud2>("floor_removed_point_cloud", 10);
    }
  }

  /**
   * @brief Destroy the ADI3DToFFloorDetectorExample object
   *
   */
  ~ADI3DToFFloorDetectorExample()
  {
    if (depth_image_ != nullptr)
    {
      delete[] depth_image_;
      depth_image_ = nullptr;
    }

    if (ir_image_ != nullptr)
    {
      delete[] ir_image_;
      ir_image_ = nullptr;
    }

    if (floor_mask_image_ != nullptr)
    {
      delete[] floor_mask_image_;
      floor_mask_image_ = nullptr;
    }

    if (xyz_image_ != nullptr)
    {
      delete[] xyz_image_;
      xyz_image_ = nullptr;
    }

    delete tf_listener_;
    delete image_proc_utils_;
  }

  void shutDownAllNodes();
  bool generateOutput();

private:
  int image_width_ = 512;
  int image_height_ = 512;
  int frame_counter_;
  int enable_pointcloud_output_;
  bool camera_parameters_updated_;
  CameraIntrinsics camera_intrinsics_;
  ImageProcUtils* image_proc_utils_ = nullptr;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  sensor_msgs::PointCloud2 transformed_pc_;

  std::string camera_info_topic_name_;
  std::string depth_image_topic_name_;
  std::string ir_image_topic_name_;
  std::string floor_mask_topic_name_;
  std::string compressed_floor_mask_topic_name_;

  ros::Subscriber sub_cam_info_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter depth_image_subscriber_;
  image_transport::SubscriberFilter ir_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> floor_mask_subscriber_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> compressed_floor_mask_subscriber_;

  // sync policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
      sync_policy_depth_floor_;
  // synchronizer
  typedef message_filters::Synchronizer<sync_policy_depth_floor_> sync_;
  boost::shared_ptr<sync_> sync_shared_ptr_;

  // sync policy for compressed images
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CompressedImage>
      sync_policy_compressed_depth_ir_floor_;
  // synchronizer for compressed images
  typedef message_filters::Synchronizer<sync_policy_compressed_depth_ir_floor_> sync_compressed_;
  boost::shared_ptr<sync_compressed_> sync_compressed_shared_ptr_;

  bool depth_image_recvd_;
  bool ir_image_recvd_;
  bool floor_mask_image_recvd_;

  sensor_msgs::ImageConstPtr depth_image_topic_;

  unsigned short* depth_image_;
  unsigned short* ir_image_;
  unsigned char* floor_mask_image_;
  short* xyz_image_;

  ros::Publisher floor_removed_depth_image_publisher_;
  ros::Publisher floor_marked_depth_image_publisher_;
  ros::Publisher floor_removed_pointcloud_publisher_;

  ros::Time curr_frame_timestamp_ = ros::Time::now();

  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);
  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image);
  void irImageCallback(const sensor_msgs::ImageConstPtr& ir_image);
  void floorMaskImageCallback(const sensor_msgs::ImageConstPtr& floor_mask_image);
  void compressedFloorMaskCallback(const sensor_msgs::CompressedImageConstPtr& compressed_floor_mask_image);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

  void syncCallback(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& floor_mask_image);

  void syncCompressedCallback(const sensor_msgs::ImageConstPtr& depth_image,
                              const sensor_msgs::CompressedImageConstPtr& compressed_floor_mask_image);

  void generateFloorRemovedPointCloud();
  sensor_msgs::PointCloud2::Ptr convert2ROSPointCloudMsg(short* xyz_frame, ros::Time stamp, std::string frame_id);

  void getFloorDetectionOutput(cv::Mat& depth_image_op_16bit, cv::Mat& floor_detection_op_8bit);

  void publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                            const ros::Publisher& publisher);
};

#endif