/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_ADTF31XX_H
#define INPUT_SENSOR_ADTF31XX_H

#include "input_sensor.h"
#include "aditof/camera.h"
#include "aditof/frame.h"
#include "aditof/system.h"
#include <stdint.h>

/**
 * @brief This is input class for sensor as camera
 *
 */
class InputSensorADTF31xx : public IInputSensor
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
  void setABinvalidationThreshold(int threshold);
  void setConfidenceThreshold(int threshold);

private:
  std::shared_ptr<aditof::Camera> camera_;
  CameraIntrinsics camera_intrinsics_;
};

#endif
