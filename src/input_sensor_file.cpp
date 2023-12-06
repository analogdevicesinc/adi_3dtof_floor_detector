/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "input_sensor_file.h"

#include <assert.h>

#include <fstream>
#include <thread>

/**
 * @brief opens the files in old .bin format which are collected via datacollection.exe
 *
 * @param sensor_name name of the input file
 * @param input_image_width width of the image
 * @param input_image_height height of the image
 * @param processing_scale scales the image dimensions and camera intrinsics.
 * @param config_file_name config file name of ToF SDK is not used as this is file io mode.
 */
void InputSensorFile::openSensor(
  std::string sensor_name, int input_image_width, int input_image_height, int processing_scale,
  std::string /*config_file_name*/)
{
  in_file_name_ = sensor_name;
  frame_counter_ = 0;
  sensor_open_flag_ = false;

  // Open file for streaming.
  in_file_.open(sensor_name, std::ifstream::binary);
  if (in_file_.is_open()) {
    // Update flag.
    sensor_open_flag_ = true;
  }

  frame_width_ = input_image_width;
  frame_height_ = input_image_height;
  input_scale_factor_ = processing_scale;
}

/**
 * @brief Configures the sensor
 *
 * @param frame_type frame type, not used in file-io mode
 */
void InputSensorFile::configureSensor(std::string /*frame_type*/)
{
  total_frames_ = 0;
  if (in_file_.is_open()) {
    // find total frames in the file.
    int num_bytes_in_combo_frame =
      (frame_width_ * frame_height_ * bytes_per_pixel_) * 5;  // *2 for depth, ir and x y z
    in_file_.seekg(0, std::ios::end);
    std::fstream::pos_type file_length = in_file_.tellg();
    in_file_.seekg(0, std::ios::beg);
    total_frames_ = file_length / num_bytes_in_combo_frame;
    setProcessingScale(2);
  }

  return;
}

/**
 * @brief Gets the camera intrinsics
 *
 * @param camera_intrinsics camera intrinsics
 */
void InputSensorFile::getIntrinsics(CameraIntrinsics * camera_intrinsics)
{
  camera_intrinsics->camera_matrix[0] = 781.291565f;
  camera_intrinsics->camera_matrix[1] = 0.0f;
  camera_intrinsics->camera_matrix[2] = 520.714905f;
  camera_intrinsics->camera_matrix[3] = 0.0f;
  camera_intrinsics->camera_matrix[4] = 781.87738f;
  camera_intrinsics->camera_matrix[5] = 513.001709f;
  camera_intrinsics->camera_matrix[6] = 0.0f;
  camera_intrinsics->camera_matrix[7] = 0.0f;
  camera_intrinsics->camera_matrix[8] = 1.0f;

  camera_intrinsics->distortion_coeffs[0] = -0.0693829656f;
  camera_intrinsics->distortion_coeffs[1] = 0.115561306f;
  camera_intrinsics->distortion_coeffs[2] = 0.000196631983f;
  camera_intrinsics->distortion_coeffs[3] = -0.00011414945f;
  camera_intrinsics->distortion_coeffs[4] = 0.0944529548f;
  camera_intrinsics->distortion_coeffs[5] = 0.269195855f;
  camera_intrinsics->distortion_coeffs[6] = -0.00811018609f;
  camera_intrinsics->distortion_coeffs[7] = 0.190516844f;

  // Scale
  camera_intrinsics->camera_matrix[0] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[2] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[4] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[5] /= input_scale_factor_;

  return;
}

/**
 * @brief gets camera extrinsics
 *
 * @param camera_extrinsics camera extrinsics
 */
void InputSensorFile::getExtrinsics(CameraExtrinsics * camera_extrinsics)
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
 * @brief reads next frame from the file
 *
 * @param scaled_depth_frame pointer to read depth frame
 * @param scaled_ir_frame pointer to read ir frame
 * @return true if reading next frame is successful.
 * @return false if reading next frame is failure.
 */
bool InputSensorFile::readNextFrame(
  unsigned short * scaled_depth_frame, unsigned short * scaled_ir_frame)
{
  // Read next frame
  int num_samples_in_frame = (frame_width_ * frame_height_);
  int num_bytes_in_combo_frame =
    (num_samples_in_frame * bytes_per_pixel_) * 5;  // 5 for depth, ir and x y z
  char * frame_buffer = new char[num_bytes_in_combo_frame];
  bool error_reading_frame = false;

  // Allocate memory
  unsigned short * depth_frame = new unsigned short[num_samples_in_frame];
  unsigned short * ir_frame = new unsigned short[num_samples_in_frame];

  assert(scaled_depth_frame != nullptr);
  assert(scaled_ir_frame != nullptr);

  if (in_file_.is_open() && (frame_counter_ < total_frames_)) {
    in_file_.read(frame_buffer, num_bytes_in_combo_frame);
    if (in_file_.gcount() != num_bytes_in_combo_frame) {
      // Error in reading frame.
      error_reading_frame = true;
    }

    ++frame_counter_;
  } else {
    // EOF is reached or file is not openned
    error_reading_frame = true;
  }

  unsigned short * temp_frame_buffer = reinterpret_cast<unsigned short *>(frame_buffer);
  if (!error_reading_frame) {
    // Copy depth data and ir data
    // First comes depth frame,
    memcpy(depth_frame, temp_frame_buffer, num_samples_in_frame * bytes_per_pixel_);
    temp_frame_buffer += num_samples_in_frame;

    // followed by ir frame.
    memcpy(ir_frame, temp_frame_buffer, num_samples_in_frame * bytes_per_pixel_);
    temp_frame_buffer += num_samples_in_frame;

    // followed by x y z image.
    // Not reading Point cloud.
    // memcpy(xyz_frame, temp_frame_buffer, num_samples_in_frame * bytes_per_pixel_ * 3);
    temp_frame_buffer += num_samples_in_frame * 3;
  }

  if (error_reading_frame) {
    // close file
    closeSensor();
    // openSensor(in_file_name_);
    // delete memory
    delete[] frame_buffer;
    delete[] depth_frame;
    delete[] ir_frame;
    return false;
  }

  unsigned short * temp_depth_frame = depth_frame;
  unsigned short * temp_ir_frame = ir_frame;

  for (int i = 0; i < frame_height_; i += input_scale_factor_) {
    for (int j = 0; j < frame_width_; j += input_scale_factor_) {
      *scaled_depth_frame++ = temp_depth_frame[j];
      *scaled_ir_frame++ = temp_ir_frame[j];
    }
    // Skip rows.
    temp_depth_frame += (frame_width_ * input_scale_factor_);
    temp_ir_frame += (frame_width_ * input_scale_factor_);
  }

  delete[] frame_buffer;
  delete[] depth_frame;
  delete[] ir_frame;

  // Sleep.. Emulate 10FPS behaviour. This also ensures that we do not overwrite the frames in read thread.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return true;
}

/**
 * @brief gets frame time stamp
 *
 */
bool InputSensorFile::getFrameTimestamp(rclcpp::Time * timestamp)
{
  *timestamp = rclcpp::Clock{}.now();
  return true;
}

/**
 * @brief closes the input file
 *
 */
void InputSensorFile::closeSensor()
{
  if (in_file_.is_open()) {
    in_file_.close();
  }
}
