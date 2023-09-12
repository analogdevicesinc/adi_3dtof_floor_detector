/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_H
#define INPUT_SENSOR_H

#include <ros/ros.h>
#include <stdint.h>
#include "adi_camera.h"

/**
 * @brief This is input base class
 *
 */
class IInputSensor
{
protected:
  bool sensor_open_flag_;
  int frame_width_ = 1024;
  int frame_height_ = 1024;
  int bytes_per_pixel_ = 2;
  int input_scale_factor_ = 2;
  unsigned int frame_counter_ = 0;

public:
  virtual void openSensor(std::string sensor_name, int input_image_width, int input_image_height, int processing_scale,
                          std::string config_file_name) = 0;
  virtual void configureSensor(std::string frame_type) = 0;
  virtual void getIntrinsics(CameraIntrinsics* camera_intrinsics) = 0;
  virtual void getExtrinsics(CameraExtrinsics* camera_extrinsics) = 0;
  virtual bool readNextFrame(unsigned short* depth_frame, unsigned short* ir_frame) = 0;
  virtual bool getFrameTimestamp(ros::Time* timestamp) = 0;
  virtual void closeSensor() = 0;
  virtual void setConfidenceThreshold(int threshold) = 0;
  virtual void setABinvalidationThreshold(int threshold) = 0;

  /**
   * @brief function to check whether input is opened
   *
   * @return true if the input sensor opened
   * @return false if the input sensor is not opened
   */
  bool isOpened()
  {
    return sensor_open_flag_;
  }
  /**
   * @brief Get the Frame Width object
   * Only integer scaling is supported
   *
   * @return int
   */
  int getFrameWidth()
  {
    return frame_width_ / input_scale_factor_;
  }
  /**
   * @brief Get the Frame Height object
   * Only integer scaling is supported
   * @return int
   */
  int getFrameHeight()
  {
    return frame_height_ / input_scale_factor_;
  }
  /**
   * @brief Set the Frame Width object
   * Only integer scaling is supported
   *
   * @return int
   */
  void setFrameWidth(int frm_width)
  {
    frame_width_ = frm_width;
  }
  /**
   * @brief Sett the Frame Height object
   * Only integer scaling is supported
   * @return int
   */
  void setFrameHeight(int frm_height)
  {
    frame_height_ = frm_height;
  }
  /**
   * @brief Set the Processing Scale object
   *
   * @param scale_factor
   * @return int
   */
  void setProcessingScale(int scale_factor)
  {
    input_scale_factor_ = scale_factor;
  }
  /**
   * @brief Get the current Frame count value
   * @return int
   */
  int getFrameCounter()
  {
    return frame_counter_;
  }
};

#endif
