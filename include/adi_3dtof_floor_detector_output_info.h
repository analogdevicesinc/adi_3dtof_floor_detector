/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_3DTOF_FLOOR_DETECTOR_OUTPUT_INFO_H
#define ADI_3DTOF_FLOOR_DETECTOR_OUTPUT_INFO_H

#include <cstring>
/**
 * @brief This is the class for adi 3d tof floor detector output info
 *
 */
class ADI3DToFFloorDetectorOutputInfo
{
public:
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   */
  ADI3DToFFloorDetectorOutputInfo(int image_width, int image_height)
  {
    // Create the node.
    frame_number_ = -1;
    ransac_floor_detection_status_ = false;
    ransac_iterations_ = -1;
    ransac_time_ms_ = -1;
    noise_count_ = -1;
    compressed_depth_frame_size_ = 0;
    compressed_ir_frame_size_ = 0;
    compressed_floor_mask_size_ = 0;
    image_width_ = image_width;
    image_height_ = image_height;
    depth_frame_ = nullptr;
    ir_frame_ = nullptr;
    xyz_frame_ = nullptr;
    floor_mask_8bit_ = nullptr;
    compressed_ir_frame_ = nullptr;
    compressed_depth_frame_ = nullptr;
    compressed_floor_mask_ = nullptr;

    depth_frame_ = new unsigned short[image_width * image_height];
    ir_frame_ = new unsigned short[image_width * image_height];
    compressed_depth_frame_ = new unsigned char[2 * image_width * image_height];
    compressed_ir_frame_ = new unsigned char[2 * image_width * image_height];
    floor_mask_8bit_ = new unsigned char[image_width * image_height];
    compressed_floor_mask_ = new unsigned char[(image_width * image_height) / 8];
    xyz_frame_ = new short[3 * image_width * image_height];
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFFloorDetectorOutputInfo()
  {
    if (depth_frame_ != nullptr) {
      delete[] depth_frame_;
    }
    if (ir_frame_ != nullptr) {
      delete[] ir_frame_;
    }
    if (floor_mask_8bit_ != nullptr) {
      delete[] floor_mask_8bit_;
    }
    if (compressed_depth_frame_ != nullptr) {
      delete[] compressed_depth_frame_;
    }
    if (compressed_ir_frame_ != nullptr) {
      delete[] compressed_ir_frame_;
    }
    if (compressed_floor_mask_ != nullptr) {
      delete[] compressed_floor_mask_;
    }
    if (xyz_frame_ != nullptr) {
      delete[] xyz_frame_;
    }
  }

  // Assignment operator
  ADI3DToFFloorDetectorOutputInfo & operator=(const ADI3DToFFloorDetectorOutputInfo & rhs)
  {
    image_width_ = rhs.image_width_;
    image_height_ = rhs.image_height_;
    frame_number_ = rhs.frame_number_;
    ransac_floor_detection_status_ = rhs.ransac_floor_detection_status_;
    ransac_iterations_ = rhs.ransac_iterations_;
    noise_count_ = rhs.noise_count_;
    ransac_time_ms_ = rhs.ransac_time_ms_;
    compressed_depth_frame_size_ = rhs.compressed_depth_frame_size_;
    compressed_ir_frame_size_ = rhs.compressed_ir_frame_size_;
    compressed_floor_mask_size_ = rhs.compressed_floor_mask_size_;

    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_) * image_width_ * image_height_);
    memcpy(ir_frame_, rhs.ir_frame_, sizeof(ir_frame_) * image_width_ * image_height_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_) * image_width_ * image_height_ * 3);
    memcpy(
      floor_mask_8bit_, rhs.floor_mask_8bit_,
      sizeof(floor_mask_8bit_[0]) * image_width_ * image_height_);
    memcpy(
      compressed_floor_mask_, rhs.compressed_floor_mask_,
      sizeof(compressed_floor_mask_[0]) * compressed_floor_mask_size_);
    memcpy(
      compressed_depth_frame_, rhs.compressed_depth_frame_,
      sizeof(compressed_depth_frame_) * compressed_depth_frame_size_);
    memcpy(
      compressed_ir_frame_, rhs.compressed_ir_frame_,
      sizeof(compressed_ir_frame_) * compressed_ir_frame_size_);
    return *this;
  }

  int image_width_;
  int image_height_;
  int frame_number_;
  bool ransac_floor_detection_status_;
  int ransac_iterations_;
  int noise_count_;
  float ransac_time_ms_;
  int compressed_depth_frame_size_;
  int compressed_ir_frame_size_;
  int compressed_floor_mask_size_;

  unsigned short * depth_frame_;
  unsigned short * ir_frame_;
  short * xyz_frame_;
  unsigned char * floor_mask_8bit_;
  unsigned char * compressed_floor_mask_;
  unsigned char * compressed_depth_frame_;
  unsigned char * compressed_ir_frame_;
};

#endif
