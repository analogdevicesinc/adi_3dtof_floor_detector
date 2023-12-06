/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "input_sensor_adtf31xx.h"

#include <fstream>

#include "aditof/camera.h"
#include "aditof/frame.h"
#include "aditof/system.h"

using aditof::Status;

/**
 * @brief Opens the camera
 *
 * @param sensor_name This parameter is not used in this derived member function.
 * @param input_image_width width of the image
 * @param input_image_height height of the image
 * @param processing_scale scale factor for image and camera intrinsics
 * @param config_file_name path of configuration json file for ToF SDK.
 */
void InputSensorADTF31XX::openSensor(
  std::string /*sensor_name*/, int input_image_width, int input_image_height, int processing_scale,
  std::string config_file_name)
{
  sensor_open_flag_ = false;

  // realtime mode
  Status status = Status::OK;
  aditof::System system;

  std::vector<std::shared_ptr<aditof::Camera>> cameras;
  system.getCameraList(cameras);
  if (cameras.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No cameras found");
    return;
  }

  camera_ = cameras.front();

  // user can pass any config.json stored anywhere in HW
  status = camera_->setControl("initialization_config", config_file_name);
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not set the initialization config file!");
    return;
  }

  status = camera_->initialize();
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialize camera!");
    return;
  }

  sensor_open_flag_ = true;

  frame_width_ = input_image_width;
  frame_height_ = input_image_height;
  input_scale_factor_ = processing_scale;

  // Clear camera parameters.
  memset(&camera_intrinsics_, 0, sizeof(camera_intrinsics_));

  return;
}

/**
 * @brief Configures the camera
 *
 * @param frame_type frame type
 */
void InputSensorADTF31XX::configureSensor(std::string frame_type)
{
  Status status = Status::OK;
  aditof::System system;

  std::vector<std::string> frame_types;
  camera_->getAvailableFrameTypes(frame_types);
  if (frame_types.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "no frame type avaialble!");
    return;
  }

  aditof::CameraDetails camera_details;
  camera_->getDetails(camera_details);

  std::cout << "Cx, Cy : " << camera_details.intrinsics.cx << ", " << camera_details.intrinsics.cy
            << std::endl;
  std::cout << "Fx, Fy : " << camera_details.intrinsics.fx << ", " << camera_details.intrinsics.fy
            << std::endl;
  std::cout << "K1, K2 : " << camera_details.intrinsics.k1 << ", " << camera_details.intrinsics.k2
            << std::endl;
  std::cout << "K3, K4 : " << camera_details.intrinsics.k3 << ", " << camera_details.intrinsics.k4
            << std::endl;
  std::cout << "K5, K6 : " << camera_details.intrinsics.k5 << ", " << camera_details.intrinsics.k6
            << std::endl;
  std::cout << "P1, P2 : " << camera_details.intrinsics.p1 << ", " << camera_details.intrinsics.p2
            << std::endl;

  // Camera matrix = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  camera_intrinsics_.camera_matrix[0] = camera_details.intrinsics.fx;
  camera_intrinsics_.camera_matrix[1] = 0;
  camera_intrinsics_.camera_matrix[2] = camera_details.intrinsics.cx;
  camera_intrinsics_.camera_matrix[3] = 0;
  camera_intrinsics_.camera_matrix[4] = camera_details.intrinsics.fy;
  camera_intrinsics_.camera_matrix[5] = camera_details.intrinsics.cy;
  camera_intrinsics_.camera_matrix[6] = 0;
  camera_intrinsics_.camera_matrix[7] = 0;
  camera_intrinsics_.camera_matrix[8] = 1;

  // Distortion vector = [k1, k2, p1, p2, k3, k4, k5, k6]
  camera_intrinsics_.distortion_coeffs[0] = camera_details.intrinsics.k1;
  camera_intrinsics_.distortion_coeffs[1] = camera_details.intrinsics.k2;

  camera_intrinsics_.distortion_coeffs[2] = camera_details.intrinsics.p1;
  camera_intrinsics_.distortion_coeffs[3] = camera_details.intrinsics.p2;

  camera_intrinsics_.distortion_coeffs[4] = camera_details.intrinsics.k3;
  camera_intrinsics_.distortion_coeffs[5] = camera_details.intrinsics.k4;
  camera_intrinsics_.distortion_coeffs[6] = camera_details.intrinsics.k5;
  camera_intrinsics_.distortion_coeffs[7] = camera_details.intrinsics.k6;

  status = camera_->setFrameType(frame_type);
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not set camera frame type!");
    return;
  }

  status = camera_->start();
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not start the camera!");
    return;
  }

  return;
}

/**
 * @brief gets the camera intrinsics from the ToF SDK.
 *
 * @param camera_intrinsics camera intrinsics of ToF module.
 */
void InputSensorADTF31XX::getIntrinsics(CameraIntrinsics * camera_intrinsics)
{
  *camera_intrinsics = camera_intrinsics_;

  // Scale
  camera_intrinsics->camera_matrix[0] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[2] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[4] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[5] /= input_scale_factor_;

  return;
}

/**
 * @brief reads frame from ToF SDK
 *
 * @param depth_frame pointer to get depth frame
 * @param ir_frame pointer to get ir frame
 * @return true if read is successful
 * @return false if read is not successful
 */

bool InputSensorADTF31XX::readNextFrame(unsigned short * depth_frame, unsigned short * ir_frame)
{
  aditof::Frame frame;

  Status status = camera_->requestFrame(&frame);
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not request frame!");
    return false;
  }

  // Depth
  uint16_t * depth_frame_src;
  status = frame.getData("depth", &depth_frame_src);
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not get depth data!");
    return false;
  }
  int frame_width = frame_width_ / input_scale_factor_;
  int frame_height = frame_height_ / input_scale_factor_;

  // Copy Depth
  memcpy(depth_frame, depth_frame_src, frame_width * frame_height * bytes_per_pixel_);

  // IR
  unsigned short * ir_frame_src;
  status = frame.getData("ir", &ir_frame_src);
  if (status != Status::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not get ir data!");
    return false;
  }
  memcpy(ir_frame, ir_frame_src, frame_width * frame_height * bytes_per_pixel_);

  // Confidence image
  // unsigned char* confidence_img = (unsigned char*)&ir_frame_src[frame_width * frame_height];
  // memcpy(conf_frame, confidence_img, frame_width * frame_height);

  return true;
}

/**
 * @brief gets the frame timestamp from ToF SDK
 *
 */

bool InputSensorADTF31XX::getFrameTimestamp(rclcpp::Time * timestamp)
{
  *timestamp = rclcpp::Clock{}.now();
  return true;
}

/**
 * @brief Gets camera extrinsics
 *
 * @param camera_extrinsics Camera extrinsics
 */

void InputSensorADTF31XX::getExtrinsics(CameraExtrinsics * camera_extrinsics)
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

  return;
}

/**
 * @brief closes the sensor
 *
 */
void InputSensorADTF31XX::closeSensor()
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping the Camera");
  camera_->stop();
  return;
}

/**
 * @brief calls set ab invalidation threshold function from ToF SDK
 *
 * @param threshold abinvalidation threshold
 */
void InputSensorADTF31XX::setABinvalidationThreshold(int threshold)
{
  camera_->adsd3500SetABinvalidationThreshold(threshold);
  return;
}

/**
 * @brief calls set confidence threshold function from ToF SDK
 *
 * @param threshold confidence threshold
 */
void InputSensorADTF31XX::setConfidenceThreshold(int threshold)
{
  camera_->adsd3500SetConfidenceThreshold(threshold);
  return;
}
