/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FILE_H
#define INPUT_SENSOR_FILE_H

#include "input_sensor.h"
#include <stdint.h>
#include <fstream>

/**
 * @brief This is input class for sensor as camera
 *
 */
class InputSensorFile : public IInputSensor
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
  std::ifstream in_file_;
  int total_frames_ = 0;
};

#endif
