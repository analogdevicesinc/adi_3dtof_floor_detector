/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef OUTPUT_SENSOR_FACTORY_H
#define OUTPUT_SENSOR_FACTORY_H

#include "output_sensor_file.h"
#include "output_sensor.h"

/**
 * @brief This is output sensor factory class
 *
 */
class OutputSensorFactory
{
public:
  /**
   * @brief Get the Output Sensor object
   *
   * @param output_sensor_type
   * @return OOutputSensor*
   */
  static OOutputSensor* getOutputSensor(int output_sensor_type)
  {
    OOutputSensor* output_sensor;
    switch (output_sensor_type)
    {
      case 0:
        // No output
        output_sensor = nullptr;
        break;
      case 1:
        // Outputs stored as files
        output_sensor = new OutputSensorFile;
        break;
      case 2:
        // ROS Bag
        ROS_INFO_STREAM("ROSBAG support TBD.");
        output_sensor = nullptr;
        break;
      default:
        ROS_INFO_STREAM("Not a valid senor type.");
        output_sensor = nullptr;
        break;
    }
    return output_sensor;
  }
};
#endif