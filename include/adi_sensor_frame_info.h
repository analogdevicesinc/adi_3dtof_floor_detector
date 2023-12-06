/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_SENSOR_FRAME_INFO_H
#define ADI_SENSOR_FRAME_INFO_H

#include <stdint.h>

#include <rclcpp/rclcpp.hpp>

/**
 * @brief This is input class for sensor as camera
 *
 */
class ADISensorFrameInfo
{
public:
  /**
   * @brief Construct a new ADISensorFrameInfo object
   *
   * @param image_width
   * @param image_height
   */
  ADISensorFrameInfo(int image_width, int image_height)
  {
    depth_image_ = new short[image_width * image_height];
    ir_image_ = new short[image_width * image_height];
    frame_timestamp_ = rclcpp::Clock{}.now();
  }

  /**
   * @brief Destroy the ADISensorFrameInfo object
   *
   */
  ~ADISensorFrameInfo()
  {
    delete[] depth_image_;
    delete[] ir_image_;
  }
  short * depth_image_;
  short * ir_image_;
  rclcpp::Time frame_timestamp_;
};

#endif
