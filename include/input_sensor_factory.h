/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FACTORY_H
#define INPUT_SENSOR_FACTORY_H

#include "input_sensor.h"
#include "input_sensor_adtf31xx.h"
#include "input_sensor_file.h"
#include "input_sensor_file_rosbagbin.h"
#include "input_sensor_ros_topics.h"

/**
 * @brief This class is input sensor factory
 *
 */
class InputSensorFactory
{
public:
  /**
   * @brief Get the Input Sensor object
   *
   * @param input_sensor_type
   * @return IInputSensor*
   */
  static IInputSensor * getInputSensor(int input_sensor_type)
  {
    IInputSensor * input_sensor;
    switch (input_sensor_type) {
      case 0:
// Camera
#ifdef ENABLE_ADI_3DTOF_ADTF31XX_SENSOR
        input_sensor = new InputSensorADTF31XX;
#else
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Either the ADI TOF Sensor is not connected to Host machine or the compile option "
          "-DSENSOR_TYPE_TOF=TRUE "
          "is not specified while building the code");
        input_sensor = nullptr;
#endif
        break;
      case 1:
        // File
        input_sensor = new InputSensorFile;
        break;
      case 2:
        // ROS Bag Bin
        input_sensor = new InputSensorFileRosbagBin;
        break;
      case 3:
        // ROS Topics
#ifndef ENABLE_ADI_3DTOF_ADTF31XX_SENSOR
        input_sensor = new InputSensorRosTopic;
#else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ROS Topic support is given only to Host.");
        input_sensor = nullptr;
#endif
        break;
      default:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Not a valid senor type.");
        input_sensor = nullptr;
        break;
    }
    return input_sensor;
  }
};
#endif
