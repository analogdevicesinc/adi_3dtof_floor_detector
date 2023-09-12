/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "module_profile.h"
#include "adtf31xx_sensor_frame_info.h"
#include "adi_3dtof_floor_detector_node.h"

/**
 * @brief Updates the parameters of algorithm based on dynamic reconfigure.
 *
 */

void ADI3DToFFloorDetector::updateDynamicReconfigureVariablesProcessThread()
{
  if (enable_ransac_floor_detection_ != dynamic_reconfigure_config_.enable_ransac_floor_detection)
  {
    enable_ransac_floor_detection_ = dynamic_reconfigure_config_.enable_ransac_floor_detection;
    ROS_INFO("Dynamic Reconfigure: Modified the 'enable_ransac_floor_detection' parameter %d",
             enable_ransac_floor_detection_);
  }

  if (ransac_max_iterations_ != dynamic_reconfigure_config_.ransac_max_iterations)
  {
    ransac_max_iterations_ = dynamic_reconfigure_config_.ransac_max_iterations;
    floor_plane_detection_->setMaxIterations(ransac_max_iterations_);
    ROS_INFO("Dynamic Reconfigure: Modified the 'ransac_max_iterations' parameter %d", ransac_max_iterations_);
  }

  if (ransac_distance_threshold_mtr_ != (float)(dynamic_reconfigure_config_.ransac_distance_threshold_mtr))
  {
    ransac_distance_threshold_mtr_ = (float)(dynamic_reconfigure_config_.ransac_distance_threshold_mtr);
    floor_plane_detection_->setRANSACDistanceThreshold(ransac_distance_threshold_mtr_);
    ROS_INFO("Dynamic Reconfigure: Modified the 'ransac_distance_threshold_mtr' parameter %f",
             ransac_distance_threshold_mtr_);
  }

  if (enable_fallback_floor_detection_ != dynamic_reconfigure_config_.enable_fallback_floor_detection)
  {
    enable_fallback_floor_detection_ = dynamic_reconfigure_config_.enable_fallback_floor_detection;
    ROS_INFO("Dynamic Reconfigure: Modified the 'enable_fallback_floor_detection' parameter %d",
             enable_fallback_floor_detection_);
  }

  if (fallback_floor_height_offset_mtr_ != (float)dynamic_reconfigure_config_.fallback_floor_height_offset_mtr)
  {
    ROS_INFO("Before: Modified the 'fallback_floor_height_offset_mtr' parameter %f", fallback_floor_height_offset_mtr_);
    fallback_floor_height_offset_mtr_ = (float)dynamic_reconfigure_config_.fallback_floor_height_offset_mtr;
    ROS_INFO("Dynamic Reconfigure: Modified the 'fallback_floor_height_offset_mtr' parameter %f",
             dynamic_reconfigure_config_.fallback_floor_height_offset_mtr);
  }
}

/**
 *
 * @brief This function is the entry point to the adi 3d tof floor detector algorithm
 *
 *
 */
bool ADI3DToFFloorDetector::runFloorDetection()
{
  PROFILE_FUNCTION_START(FLOOR_DETECTOR_0)

  PROFILE_FUNCTION_START(FLOOR_DETECTOR_1_PREPROCESS)
  // nullptr checks
  if ((camera_map_tf_listener_ == nullptr) || (optical_map_tf_listener_ == nullptr))
  {
    return false;
  }

  // Update dynamic reconfigure values.
  updateDynamicReconfigureVariablesProcessThread();

  // Get frame from sensor.
  ADTF31xxSensorFrameInfo* inframe;
  try
  {
    inframe = floorDetectorIOThreadGetNextFrame();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  if (inframe == nullptr)
  {
    return false;
  }

  depth_frame_ = inframe->getDepthFrame();
  ir_frame_ = inframe->getIRFrame();
  xyz_frame_ = inframe->getXYZFrame();
  compressed_depth_frame_ = inframe->getCompressedDepthFrame();
  compressed_depth_frame_size_ = inframe->getCompressedDepthFrameSize();

  if ((depth_frame_ == nullptr) || (ir_frame_ == nullptr) || (xyz_frame_ == nullptr) ||
      ((enable_compression_op_image_topics_) && (compressed_depth_frame_ == nullptr)))
  {
    return false;
  }

  // Rotation of point cloud
  if (camera_tilted_)
  {
    // With Rotation
    input_xyz_frame_ = inframe->getRotatedXYZFrame();
  }
  else
  {
    // Without Rotation
    input_xyz_frame_ = xyz_frame_;
  }
  // xyz_frame is getting published (Rviz will handle rotation for this point cloud for visualization)
  output_xyz_frame_ = xyz_frame_;

  PROFILE_FUNCTION_END(FLOOR_DETECTOR_1_PREPROCESS)

  PROFILE_FUNCTION_START(FLOOR_DETECTOR_6_RUN)
  // Set global timestamp
  curr_frame_timestamp_ = inframe->getFrameTimestamp();

  ADI3DToFFloorDetectorOutputInfo* new_output_frame = new ADI3DToFFloorDetectorOutputInfo(image_width_, image_height_);
  if (new_output_frame == nullptr)
  {
    return false;
  }

  // nullptr checks
  if ((new_output_frame->depth_frame_ == nullptr) || (new_output_frame->ir_frame_ == nullptr) ||
      (new_output_frame->xyz_frame_ == nullptr))
  {
    return false;
  }

  // Get camera link and virtual camera link TF
  getCameraLinksTF();

  // Set default values for floor mask
  memset(floor_mask_, 0, (image_width_ * image_height_ * sizeof(floor_mask_[0])));
  if (enable_compression_op_image_topics_)
  {
    memset(compressed_floor_mask_, 0, (compressed_floor_pixels_ * sizeof(compressed_floor_mask_[0])));
  }

  // Set filter flag
  FilterFlag filter_flag = DoNotRemoveFloor;

  if (enable_ransac_floor_detection_)
  {
    PROFILE_FUNCTION_START(FLOOR_DETECTOR_7_RANSAC_RUN)
    // int64 e1 = cv::getTickCount();
    // Floor Removal using modified PCL RANSAC
    // Default values for Enhanced RANSAC debug outputs
    ransac_iterations_ = 0;
    noise_count_ = 0;
    ransac_time_ms_ = 0;

    // Enhanced RANSAC approach
    ransac_floor_detection_status_ = floor_plane_detection_->detectFloorUsingEnhancedRANSAC(
        &depth_frame_[0], &output_xyz_frame_[0], filter_flag, &floor_mask_[0], &compressed_floor_mask_[0],
        input_xyz_frame_, inframe->getModifiedXYZFrame(), inframe->getModifiedXYZFrameSize(),
        inframe->getNearbyObjectsFoundFlag());

#if 0
    //Debug outputs   
    if (ransac_floor_detection_status_)
    {
      ransac_iterations_ = floor_plane_detection_->getRANSACIterations();
      noise_count_ = floor_plane_detection_->getNoiseCount();
    }
#endif
    // int64 e2 = cv::getTickCount();
    // ransac_time_ms_ = ((e2 - e1) * 1000.0f) / cv::getTickFrequency();

    PROFILE_FUNCTION_END(FLOOR_DETECTOR_7_RANSAC_RUN)
  }

  if ((!ransac_floor_detection_status_) && (enable_fallback_floor_detection_) || (!enable_ransac_floor_detection_))
  {
    PROFILE_FUNCTION_START(FLOOR_DETECTOR_8_FALLBACK_RUN)
    if (!ransac_floor_detection_status_)
    {
      std::cout << "Running traditional floor detection algorithm.\n" << std::endl;
    }
    // 3D Y Threshold based approach
    image_proc_utils_->detectFloorFromPointCloud(input_xyz_frame_, floor_distance_threshold_mm_, image_width_,
                                                 image_height_, depth_frame_, output_xyz_frame_, filter_flag,
                                                 &floor_mask_[0], &compressed_floor_mask_[0]);
    PROFILE_FUNCTION_END(FLOOR_DETECTOR_8_FALLBACK_RUN)
  }

  // The generate visualization and publish topics are run in a different thread,
  // so, copy the buffers needed by these functions to a queue
  if (new_output_frame != nullptr)
  {
    new_output_frame->frame_number_ = frame_number_;
    new_output_frame->ransac_floor_detection_status_ = ransac_floor_detection_status_;
    new_output_frame->ransac_iterations_ = ransac_iterations_;
    new_output_frame->noise_count_ = noise_count_;
    new_output_frame->ransac_time_ms_ = ransac_time_ms_;

    memcpy(new_output_frame->depth_frame_, depth_frame_, image_width_ * image_height_ * sizeof(depth_frame_[0]));
    memcpy(new_output_frame->ir_frame_, ir_frame_, image_width_ * image_height_ * sizeof(ir_frame_[0]));
    memcpy(new_output_frame->xyz_frame_, input_xyz_frame_, 3 * image_width_ * image_height_ * sizeof(xyz_frame_[0]));
    memcpy(new_output_frame->floor_mask_8bit_, floor_mask_, image_width_ * image_height_ * sizeof(floor_mask_[0]));

    if (enable_compression_op_image_topics_)
    {
      new_output_frame->compressed_depth_frame_size_ = compressed_depth_frame_size_;
      new_output_frame->compressed_floor_mask_size_ = compressed_floor_pixels_;
      memcpy(new_output_frame->compressed_depth_frame_, compressed_depth_frame_,
             2 * image_width_ * image_height_ * sizeof(compressed_depth_frame_[0]));
      memcpy(new_output_frame->compressed_floor_mask_, compressed_floor_mask_,
             compressed_floor_pixels_ * sizeof(compressed_floor_mask_[0]));
    }

    // Push
    floorDetectorIOThreadPushOutputNode(new_output_frame);
  }

  frame_number_++;

  // dispose the frame
  delete inframe;
  PROFILE_FUNCTION_END(FLOOR_DETECTOR_6_RUN)

  PROFILE_FUNCTION_END(FLOOR_DETECTOR_0)
  return true;
}

/**
 * @brief This function gives TF between different camera frames
 *
 */
void ADI3DToFFloorDetector::getCameraLinksTF()
{
  // Get RPY
  geometry_msgs::TransformStamped camera_map_transform;
  try
  {
    camera_map_transform =
        camera_map_tf_buffer_.lookupTransform("map", camera_link_, ros::Time(0.0f), ros::Duration(1.0f));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Camera to Map TF2 Error: %s\n", ex.what());
    return;
  }

  // Camera Tilt
  // Ref : https://en.wikipedia.org/wiki/Rotation_matrix
  tf2::Quaternion camera_map_qt(camera_map_transform.transform.rotation.x, camera_map_transform.transform.rotation.y,
                                camera_map_transform.transform.rotation.z, camera_map_transform.transform.rotation.w);
  tf2::Matrix3x3 camera_map_rotation_matrix(camera_map_qt);
  double roll, pitch, yaw;
  /*
  Input Pitch should be always in (-1.57, 1.57).
  For the |pitch| > 1.57, please consider taking (3.14 +/- pitch) as output here.
  For input pitch |1.57|, output will be  wrong because cosβ = 0
  */
  camera_map_rotation_matrix.getRPY(roll, pitch, yaw);
  camera_roll_rad_ = roll;
  camera_pitch_rad_ = pitch;
  camera_yaw_rad_ = yaw;

  camera_tilted_ = false;

  // Yaw is neglected as it does not affect the algorithm
  if ((std::fabs(camera_roll_rad_) >= std::numeric_limits<float>::epsilon()) ||
      (std::fabs(camera_pitch_rad_) >= std::numeric_limits<float>::epsilon()))
  {
    camera_tilted_ = true;
  }

  // Get reverse rotation matrix
  geometry_msgs::TransformStamped optical_map_transform;
  try
  {
    optical_map_transform =
        optical_map_tf_buffer_.lookupTransform("map", optical_camera_link_, ros::Time(0.0f), ros::Duration(1.0f));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Optical to Map TF2 Error: %s\n", ex.what());
    return;
  }

  tf2::Quaternion optical_map_qt(optical_map_transform.transform.rotation.x, optical_map_transform.transform.rotation.y,
                                 optical_map_transform.transform.rotation.z,
                                 optical_map_transform.transform.rotation.w);
  tf2::Matrix3x3 cam_rotation_matrix(optical_map_qt);

  depth_extrinsics_external_.translation_matrix[0] = optical_map_transform.transform.translation.x;
  depth_extrinsics_external_.translation_matrix[1] = optical_map_transform.transform.translation.y;
  depth_extrinsics_external_.translation_matrix[2] = optical_map_transform.transform.translation.z;

  // Rotation Matrix
  float optical_map_rotation_matrix[9];
  int j = 0;
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      optical_map_rotation_matrix[j++] = cam_rotation_matrix.getRow(i)[k];
    }
  }

  // Point Cloud reference : forward -> +Z, Right -> +X, Down -> +Y
  // ROS reference : forward -> +X, Left -> +Y, Up -> +Z
  // Multiplying with constant matrix which rotates the point cloud in Z -90 and X -90 degrees
  // Final Rotation Matrix = reference_reverse_rotation_matrix * optical_map_rotation_matrix
  float reference_reverse_rotation_matrix[9] = { 0, -1, 0, 0, 0, -1, 1, 0, 0 };
  image_proc_utils_->matrixMultiplication(&reference_reverse_rotation_matrix[0], 3, 3, &optical_map_rotation_matrix[0],
                                          3, 3, &depth_extrinsics_external_.rotation_matrix[0]);

  /*Camera height from floor*/
  camera_height_mtr_ = optical_map_transform.transform.translation.z;
  floor_distance_threshold_mm_ = (camera_height_mtr_ - fallback_floor_height_offset_mtr_) * 1000.0f;
}
