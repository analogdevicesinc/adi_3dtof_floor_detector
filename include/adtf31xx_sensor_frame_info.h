/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADTF31XX_SENSOR_FRAME_INFO_H
#define ADTF31XX_SENSOR_FRAME_INFO_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>
/**
 * @brief This is the class for adtf31xx sensor frame
 *
 */
class ADTF31xxSensorFrameInfo
{
public:
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   */
  ADTF31xxSensorFrameInfo(int image_width, int image_height)
  {
    // Create the node.
    depth_frame_ = nullptr;
    ir_frame_ = nullptr;
    xyz_frame_ = nullptr;
    rotated_xyz_frame_ = nullptr;
    image_width_ = image_width;
    image_height_ = image_height;

    depth_frame_ = new unsigned short[image_width * image_height];
    ir_frame_ = new unsigned short[image_width * image_height];
    xyz_frame_ = new short[image_width * image_height * 3];
    rotated_xyz_frame_ = new short[image_width * image_height * 3];

    // Worst case, RVL compression can take ~1.5x the input data.
    compressed_depth_frame_ = new unsigned char[2 * image_width * image_height];
    compressed_depth_frame_size_ = 0;
    frame_timestamp_ = ros::Time::now();
    modified_xyz_frame_ = new float[image_width * image_height * 3];
    modified_xyz_frame_size_ = 0;
    nearby_objects_found_flag_ = false;
  }

  /**
   * @brief Destructor
   */
  ~ADTF31xxSensorFrameInfo()
  {
    if (depth_frame_ != nullptr)
    {
      delete[] depth_frame_;
    }
    if (ir_frame_ != nullptr)
    {
      delete[] ir_frame_;
    }
    if (xyz_frame_ != nullptr)
    {
      delete[] xyz_frame_;
    }
    if (rotated_xyz_frame_ != nullptr)
    {
      delete[] rotated_xyz_frame_;
    }
    if (compressed_depth_frame_ != nullptr)
    {
      delete[] compressed_depth_frame_;
    }
    if (modified_xyz_frame_ != nullptr)
    {
      delete[] modified_xyz_frame_;
    }
  }

  /**
   * @brief Get the depth image frame
   *
   * @return unsigned short* depth image pointer
   */
  unsigned short* getDepthFrame() const
  {
    return depth_frame_;
  }

  /**
   * @brief Get the IR image frame
   *
   * @return unsigned short* IR image pointer
   */

  unsigned short* getIRFrame() const
  {
    return ir_frame_;
  }

  /**
   * @brief Get point cloud frame
   *
   * @return short* point cloud pointer
   */

  short* getXYZFrame() const
  {
    return xyz_frame_;
  }

  /**
   * @brief Get the Rotated point cloud frame
   *
   * @return short* point cloud pointer
   */
  short* getRotatedXYZFrame() const
  {
    return rotated_xyz_frame_;
  }

  /**
   * @brief Get Compressed depth image frame
   *
   * @return unsigned char* compressed depth image pointer
   */
  unsigned char* getCompressedDepthFrame() const
  {
    return compressed_depth_frame_;
  }

  /**
   * @brief Get Frame Timestamp Pointer
   *
   * @return ros::Time* Frame Timnestamp pointer
   */
  ros::Time* getFrameTimestampPtr()
  {
    return &frame_timestamp_;
  }

  /**
   * @brief Get Frame Timestamp
   *
   * @return ros::Time Frame Timestamp
   */
  ros::Time getFrameTimestamp() const
  {
    return frame_timestamp_;
  }

  /**
   * @brief Gives compressed depth image size
   *
   * @return int size of compressed depth image
   */
  int getCompressedDepthFrameSize() const
  {
    return compressed_depth_frame_size_;
  }

  /**
   * @brief Set the Compressed depth image size
   *
   * @param compressed_depth_frame_size size of compressed depth image
   */
  void setCompressedDepthFrameSize(int compressed_depth_frame_size)
  {
    compressed_depth_frame_size_ = compressed_depth_frame_size;
  }

  void setModifiedXYZFrameSize(int frame_size)
  {
    modified_xyz_frame_size_ = frame_size;
  }

  int getModifiedXYZFrameSize()
  {
    return modified_xyz_frame_size_;
  }

  float* getModifiedXYZFrame() const
  {
    return modified_xyz_frame_;
  }

  bool* getNearbyObjectsFoundFlagPtr()
  {
    return &nearby_objects_found_flag_;
  }

  bool getNearbyObjectsFoundFlag()
  {
    return nearby_objects_found_flag_;
  }

  // Assignment operator
  ADTF31xxSensorFrameInfo& operator=(const ADTF31xxSensorFrameInfo& rhs)
  {
    image_width_ = rhs.image_width_;
    image_height_ = rhs.image_height_;
    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_[0]) * image_width_ * image_height_);
    memcpy(ir_frame_, rhs.ir_frame_, sizeof(ir_frame_[0]) * image_width_ * image_height_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_[0]) * image_width_ * image_height_ * 3);
    memcpy(rotated_xyz_frame_, rhs.rotated_xyz_frame_,
           sizeof(rotated_xyz_frame_[0]) * image_width_ * image_height_ * 3);
    compressed_depth_frame_size_ = rhs.compressed_depth_frame_size_;
    memcpy(compressed_depth_frame_, rhs.compressed_depth_frame_,
           sizeof(compressed_depth_frame_[0]) * compressed_depth_frame_size_);
    frame_timestamp_ = rhs.frame_timestamp_;

    memcpy(modified_xyz_frame_, rhs.modified_xyz_frame_, (image_width_ * image_height_) * 3 * sizeof(float));
    nearby_objects_found_flag_ = rhs.nearby_objects_found_flag_;
    modified_xyz_frame_size_ = rhs.modified_xyz_frame_size_;
    return *this;
  }

private:
  /**
   * @brief Image width
   */
  int image_width_;

  /**
   * @brief Image height
   */
  int image_height_;

  /**
   * @brief Depth image
   */
  unsigned short* depth_frame_;

  /**
   * @brief IR image
   */
  unsigned short* ir_frame_;
  /**
   * @brief xyz frame
   *
   */
  short* xyz_frame_;
  /**
   * @brief rotated xyz frame
   *
   */
  short* rotated_xyz_frame_;
  /**
   * @brief size of compressed depth frame
   *
   */
  int compressed_depth_frame_size_;

  /**
   * @brief compressed depth frame
   *
   */
  unsigned char* compressed_depth_frame_;

  /**
   * @brief Frame Timestamp
   */
  ros::Time frame_timestamp_;

  /**
   * @brief Modified/Filtered xyz buffer
   */
  float* modified_xyz_frame_;
  /**
   * @brief Modified/Filtered xyz buffer size
   */
  int modified_xyz_frame_size_;
  /**
   * @brief Flag to indicate the presence of a nearby object
   */
  bool nearby_objects_found_flag_;
};

#endif
