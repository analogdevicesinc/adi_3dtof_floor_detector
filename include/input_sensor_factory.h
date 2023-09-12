/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FACTORY_H
#define INPUT_SENSOR_FACTORY_H

#include "input_sensor_adtf31xx.h"
#include "input_sensor_file.h"
#include "input_sensor_file_rosbagbin.h"
#include "input_sensor.h"
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
  static IInputSensor* getInputSensor(int input_sensor_type)
  {
    IInputSensor* input_sensor;
    switch (input_sensor_type)
    {
      case 0:
// Camera
#ifdef ENABLE_ADI_3DTOF_ADTF31XX_SENSOR
        input_sensor = new InputSensorADTF31xx;
#else
        ROS_ERROR(
            "Since the ROS node is now executing on the host, the value of arg_input_sensor_mode = 0 is not supported."
            "Please check for argument arg_input_sensor_mode in related launch files.");
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
        input_sensor = new InputSensorRosTopic;
        break;
      default:
        ROS_INFO_STREAM("Not a valid senor type.");
        input_sensor = nullptr;
        break;
    }
    return input_sensor;
  }
};
#endif
