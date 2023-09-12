/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "adi_3dtof_floor_detector_node.h"
#include <compressed_depth_image_transport/compression_common.h>

/**
 * @brief This image fills and publishes the camera information
 *
 * @param frame_id  frame_id of camera_info
 * @param publisher This is Ros publisher
 */
void ADI3DToFFloorDetector::fillAndPublishCameraInfo(const std::string& frame_id, const ros::Publisher& publisher)
{
  cam_info_msg_.header.seq = input_sensor_->getFrameCounter();
  cam_info_msg_.header.stamp = curr_frame_timestamp_;
  cam_info_msg_.header.frame_id = frame_id;

  cam_info_msg_.width = image_width_;
  cam_info_msg_.height = image_height_;

  cam_info_msg_.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  cam_info_msg_.K.fill(0.0f);
  cam_info_msg_.K[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.K[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.K[4] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.K[5] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.K[8] = 1.0f;

  cam_info_msg_.P.fill(0.0);
  cam_info_msg_.P[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.P[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.P[3] = depth_extrinsics_.translation_matrix[0];
  cam_info_msg_.P[5] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.P[6] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.P[7] = depth_extrinsics_.translation_matrix[1];
  cam_info_msg_.P[10] = 1.0f;
  cam_info_msg_.P[11] = depth_extrinsics_.translation_matrix[2];

  cam_info_msg_.D.resize(0);
  for (float distortion_coeff : depth_intrinsics_.distortion_coeffs)
  {
    cam_info_msg_.D.push_back(distortion_coeff);
  }

  cam_info_msg_.R.fill(0.0f);
  for (int i = 0; i < 9; i++)
  {
    cam_info_msg_.R[i] = depth_extrinsics_.rotation_matrix[i];
  }

  cam_info_msg_.binning_x = 0;
  cam_info_msg_.binning_y = 0;
  cam_info_msg_.roi.do_rectify = false;
  cam_info_msg_.roi.height = 0;
  cam_info_msg_.roi.width = 0;
  cam_info_msg_.roi.x_offset = 0;
  cam_info_msg_.roi.y_offset = 0;

  publisher.publish(cam_info_msg_);
}

/**
 * @brief This function publishes a image(of cv::Mat() type) as Ros message.
 *
 * @param img Input image
 * @param encoding_type number of bits used to represent one pixel of image.
 * @param frame_id frame id of image
 * @param publisher ROS publisher handle
 * @param enable_image_compression Image compression Flag
 */
void ADI3DToFFloorDetector::publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                                                 const ros::Publisher& publisher, bool enable_image_compression)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = encoding_type;
  cv_ptr->header.seq = input_sensor_->getFrameCounter();
  cv_ptr->header.stamp = curr_frame_timestamp_;
  cv_ptr->header.frame_id = std::move(frame_id);
  cv_ptr->image = std::move(img);

  if (enable_image_compression)
  {
    // We have to send compressed image.
    publisher.publish(cv_ptr->toCompressedImageMsg());
  }
  else
  {
    publisher.publish(cv_ptr->toImageMsg());
  }
}

/**
 * @brief This function publishes the point cloud
 *
 * @param xyz_frame Buffer containing the xyz values in interleaved format
 *
 * Note: Assumes that cam_info_msg_ is already populated
 */
void ADI3DToFFloorDetector::publishPointCloud(short* xyz_frame)
{
  sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);

  pointcloud_msg->header.seq = input_sensor_->getFrameCounter();
  pointcloud_msg->header.stamp = curr_frame_timestamp_;
  pointcloud_msg->header.frame_id = optical_camera_link_;
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

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
  for (int i = 0; i < image_height_; i++)
  {
    for (int j = 0; j < image_width_; j++)
    {
      *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  // Publisher
  xyz_image_publisher_.publish(pointcloud_msg);
}

/**
 * @brief This function publishes depth image , ir image, point-cloud and camera info.
 *
 * @param depth_frame - Pointer to the depth frame buffer
 * @param ir_frame - Pointer to the ir frame buffer
 * @param floor_mask_frame - Pointer to the Floor Mask buffer
 * @param xyz_frame - Pointer to the xyz frame buffer
 */
void ADI3DToFFloorDetector::publishImageAndCameraInfo(unsigned short* depth_frame, unsigned short* ir_frame,
                                                      unsigned char* floor_mask_frame, short* xyz_frame)
{
  // Publish image as Ros message
  cv::Mat m_disp_image_depth, m_disp_image_ir, m_disp_image_floor_mask;

  // convert to 16 bit depth and IR image of CV format.
  m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame);
  m_disp_image_ir = cv::Mat(image_height_, image_width_, CV_16UC1, ir_frame);
  m_disp_image_floor_mask = cv::Mat(image_height_, image_width_, CV_8UC1, floor_mask_frame);

  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);
  // encoding type should not be mono as nodelet expects it to be in enc format
  // frame id should be the frame name not topic name
  publishImageAsRosMsg(m_disp_image_depth, "mono16", optical_camera_link_, depth_image_publisher_, false);
  // temporarary : displaying normaized depth in ir publisher.
  publishImageAsRosMsg(m_disp_image_ir, "mono16", optical_camera_link_, ir_image_publisher_, false);
  publishImageAsRosMsg(m_disp_image_floor_mask, "mono8", optical_camera_link_, floor_mask_publisher_, false);

  if (enable_pointcloud_publisher_)
  {
    publishPointCloud(xyz_frame);
  }
}

/**
 * @brief This function publishes camera info and compressed version of depth, ir, floor mask images
 *
 * @param compressed_depth_frame Compressed depth image
 * @param compressed_depth_frame_size Compressed depth image size
 * @param compressed_ir_frame Compressed ir image
 * @param compressed_ir_frame_size Compressed ir image size
 * @param compressed_floor_mask_frame Compressed floor mask image
 * @param compressed_floor_mask_frame_size Compressed floor mask image size
 * @param xyz_frame point cloud buffer
 */
void ADI3DToFFloorDetector::publishImageAndCameraInfo(unsigned char* compressed_depth_frame,
                                                      int compressed_depth_frame_size,
                                                      unsigned char* compressed_ir_frame, int compressed_ir_frame_size,
                                                      unsigned char* compressed_floor_mask_frame,
                                                      int compressed_floor_mask_frame_size, short* xyz_frame)
{
  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);

  publishCompressedImageAsRosMsg(compressed_depth_frame, compressed_depth_frame_size, "mono16", optical_camera_link_,
                                 depth_image_publisher_);

  publishCompressedImageAsRosMsg(compressed_ir_frame, compressed_ir_frame_size, "mono16", optical_camera_link_,
                                 ir_image_publisher_);

  publishCompressedImageAsRosMsg(compressed_floor_mask_frame, compressed_floor_mask_frame_size, "mono8",
                                 optical_camera_link_, floor_mask_publisher_);

  if (enable_pointcloud_publisher_)
  {
    publishPointCloud(xyz_frame);
  }
}

/**
 * @brief Gives output frame for floor detection video
 *
 * @param depth_frame_with_floor Original depth frame
 * @param ir_frame Original IR frame
 * @param floor_mask_8bit Floor Mask
 * @return cv::Mat Final frame for Output Video
 */
cv::Mat ADI3DToFFloorDetector::getFloorDetectionOutput(unsigned short* depth_frame_with_floor, unsigned short* ir_frame,
                                                       unsigned char* floor_mask_8bit)
{
  // Gamma Correction for 16bit IR image
  for (int i = 0; i < (image_width_ * image_height_); i++)
  {
    float read = (float)ir_frame[i];
    float out_val = (float)(256.0f * log(read)) / log(2048.0f);
    ir_frame[i] = (uint16_t)out_val;
  }

  // Get 8bit IR image
  int max = 0, min = 65535;
  for (int i = 0; i < (image_width_ * image_height_); i++)
  {
    if (ir_frame[i] > max)
    {
      max = ir_frame[i];
    }
    if (ir_frame[i] < min)
    {
      min = ir_frame[i];
    }
  }
  cv::Mat ir_image_8bit = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);
  for (int i = 0; i < (image_width_ * image_height_); i++)
  {
    ir_image_8bit.data[i] = (ir_frame[i] - min) / (float)(max - min) * 255;
  }

  // Get rgb 8 bit ir image
  cv::Mat ir_image_8bit_rgb = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
  cv::cvtColor(ir_image_8bit, ir_image_8bit_rgb, cv::COLOR_GRAY2BGR);

  // convert 16bit depth image with floor to 8bit
  cv::Mat depth_image_16bit = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame_with_floor);
  cv::Mat depth_image_8bit = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
  unsigned short max_element = 8192;
  float scale_factor = 255.0f / max_element;
  depth_image_16bit.convertTo(depth_image_8bit, CV_8UC1, scale_factor, 0);

  // convert 8bit depth image to rgb
  cv::Mat depth_image_8bit_rgb = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
  cv::cvtColor(depth_image_8bit, depth_image_8bit_rgb, cv::COLOR_GRAY2BGR);

  // Concatenation of input depth and IR images
  cv::Mat ir_depth_op_image = cv::Mat::zeros(cv::Size(image_width_ * 2, image_height_), CV_8UC3);
  cv::hconcat(ir_image_8bit_rgb, depth_image_8bit_rgb, ir_depth_op_image);

  // Get floor marked output
  cv::Mat floor_mask_8bit_image = cv::Mat(image_height_, image_width_, CV_8UC1, floor_mask_8bit);

  // Only if floor exists in the depth image
  int non_zero_pixels = cv::countNonZero(floor_mask_8bit_image);
  if (non_zero_pixels > 0)
  {
    cv::Mat channels[3];
    cv::split(depth_image_8bit_rgb, channels);
    cv::bitwise_or(channels[1], floor_mask_8bit_image, channels[1]);
    cv::merge(channels, 3, depth_image_8bit_rgb);
  }

  // Concatenate floor detection output
  cv::Mat final_out_image = cv::Mat::zeros(cv::Size(image_width_ * 3, image_height_), CV_8UC3);
  cv::hconcat(ir_depth_op_image, depth_image_8bit_rgb, final_out_image);

  return final_out_image;
}

/**
 *@brief This function shuts down all the active nodes
 *
 */
void ADI3DToFFloorDetector::shutDownAllNodes()
{
  int status = system("rosnode kill -a");
  if (status < 0)
  {
    ROS_INFO_STREAM("Error in \"rosnode kill -a\": " << status);
  }
  ros::shutdown();
}

/**
 * @brief Publishes the compressed image as ROS message
 *
 * @param compressed_img input image
 * @param compressed_img_size input image size
 * @param encoding_type encodding type
 * @param frame_id frame id
 * @param publisher output image publisher
 */
void ADI3DToFFloorDetector::publishCompressedImageAsRosMsg(unsigned char* compressed_img, int compressed_img_size,
                                                           const std::string& encoding_type, std::string frame_id,
                                                           const ros::Publisher& publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  sensor_msgs::CompressedImage::Ptr compressed_payload_ptr(new sensor_msgs::CompressedImage());

  compressed_payload_ptr->format = encoding_type + ";compressedDepth rvl";
  compressed_payload_ptr->header.seq = input_sensor_->getFrameCounter();
  compressed_payload_ptr->header.stamp = curr_frame_timestamp_;
  compressed_payload_ptr->header.frame_id = std::move(frame_id);
  compressed_payload_ptr->data.resize(compressed_img_size + 8 + sizeof(compressed_depth_image_transport::ConfigHeader));

  // Adding image transport publisher header, so that rviz/image view tools can directly disply compressed images.
  compressed_depth_image_transport::ConfigHeader compression_configuration{};
  compression_configuration.format = compressed_depth_image_transport::INV_DEPTH;

  float depth_quantization = 0;
  float maximum_depth = 1;

  float depth_quantization_a = depth_quantization * (depth_quantization + 1.0f);
  float depth_quantization_b = 1.0f - depth_quantization_a / maximum_depth;

  compression_configuration.depthParam[0] = depth_quantization_a;
  compression_configuration.depthParam[1] = depth_quantization_b;

  memcpy(&compressed_payload_ptr->data[0], &compression_configuration,
         sizeof(compressed_depth_image_transport::ConfigHeader));
  memcpy(&compressed_payload_ptr->data[0] + sizeof(compressed_depth_image_transport::ConfigHeader), &image_width_,
         sizeof(int));
  memcpy(&compressed_payload_ptr->data[4] + sizeof(compressed_depth_image_transport::ConfigHeader), &image_height_,
         sizeof(int));

  memcpy(&compressed_payload_ptr->data[8] + sizeof(compressed_depth_image_transport::ConfigHeader), compressed_img,
         compressed_img_size);

  publisher.publish(compressed_payload_ptr);
}

/**
 * @brief new values from dynamic reconfigure are copied to a struture variable here, actual update to individual
 * parameters happens in updateDynamicReconfigureVariablesInputThread and updateDynamicReconfigureVariablesProcessThread
 * functions.
 *
 * @param config Config parameters present in GUI
 * @param level
 */
void ADI3DToFFloorDetector::dynamicallyReconfigureVariables(adi_3dtof_floor_detector::FloorDetectorParamsConfig& config,
                                                            uint32_t /*level*/)
{
  // update all the values in dynamic reconfigure.
  dynamic_reconfigure_config_ = config;
}

/**
 * @brief Overwriting the parameters read from Dynamic Reconfigure only in init time
 * This is done to make sure that launch file has higher priority than dynamic reconfigure
 * in init time.
 *
 */
void ADI3DToFFloorDetector::initSettingsForDynamicReconfigure()
{
  dynamic_reconfigure_config_.enable_ransac_floor_detection = enable_ransac_floor_detection_;
  dynamic_reconfigure_config_.ransac_max_iterations = ransac_max_iterations_;
  dynamic_reconfigure_config_.ransac_distance_threshold_mtr = ransac_distance_threshold_mtr_;
  dynamic_reconfigure_config_.enable_fallback_floor_detection = enable_fallback_floor_detection_;
  dynamic_reconfigure_config_.fallback_floor_height_offset_mtr = fallback_floor_height_offset_mtr_;
  dynamic_reconfigure_config_.ab_threshold = ab_threshold_;
  dynamic_reconfigure_config_.confidence_threshold = confidence_threshold_;
}
