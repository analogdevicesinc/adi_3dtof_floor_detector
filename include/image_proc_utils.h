/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef IMAGE_PROC_UTILS_H
#define IMAGE_PROC_UTILS_H

#include "adi_camera.h"
#include "floor_plane_detection.h"
#include <cv_bridge/cv_bridge.h>

#define QFORMAT_FOR_POINTCLOUD_LUT 14
#define QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE (1 << QFORMAT_FOR_POINTCLOUD_LUT)

/**
 * @brief This class has image processing utilities
 *
 */
class ImageProcUtils
{
public:
  // default constructor with no argument
  ImageProcUtils() = default;

  /**
   * @brief Construct a new Image Proc Utils object
   *
   * @param camera_intrinsics pointer to camera intrinsics
   * @param image_width width of the image
   * @param image_height height of the image
   */
  ImageProcUtils(CameraIntrinsics* camera_intrinsics, int image_width, int image_height)
  {
    image_width_ = image_width;
    image_height_ = image_height;

    // Generate LUT for range to depth correction
    range_to_xyz_lut_fixed_point_ = new short[image_width_ * image_height_ * 3];
    memset(range_to_xyz_lut_fixed_point_, 0, sizeof(short) * image_width_ * image_height_ * 3);

    generateRangeTo3DLUT(camera_intrinsics);
  }

  /**
   * @brief Destroy the Image Proc Utils object
   *
   */
  ~ImageProcUtils()
  {
    // Generate LUT for range to depth correction
    delete[] range_to_xyz_lut_fixed_point_;
  }

  void generateRangeTo3DLUT(CameraIntrinsics* camera_intrinsics);

  void computePointCloud(unsigned short* range_image, short* xyz_frame);

  static void matrixMultiplication(float* input_matrix1, int rows1, int columns1, float* input_matrix2, int rows2,
                                   int columns2, float* output_matrix);

  static void matrixMultiplication3x3And3x1(float* input_matrix1, short* input_matrix2, short* output_matrix);

  static void rotatePointCloud(short* input_point_cloud, short* rotated_point_cloud,
                               CameraExtrinsics* camera_extrinsics, int image_width, int image_height);

  static void detectFloorFromPointCloud(short* xyz_frame, float floor_distance_threshold_mm, int image_width,
                                        int image_height, unsigned short* output_depth_frame, short* output_xyz_frame,
                                        FilterFlag filter_flag, unsigned char* floor_mask,
                                        unsigned char* compressed_floor_mask);

private:
  int image_width_;
  int image_height_;
  short* range_to_xyz_lut_fixed_point_;
};

#endif
