/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "image_proc_utils.h"

#include <opencv2/calib3d.hpp>

/**
 * @brief Generates a Range to 3D projection Look up table, which can be
 * used to compute point-cloud from the depth image.
 *
 * @param camera_intrinsics camera intrinsics
 */
void ImageProcUtils::generateRangeTo3DLUT(CameraIntrinsics * camera_intrinsics)
{
  /* Generate Camera Intrinsics calibration arrays. */
  double k_raw_array[9] = {camera_intrinsics->camera_matrix[0], camera_intrinsics->camera_matrix[1],
                           camera_intrinsics->camera_matrix[2], camera_intrinsics->camera_matrix[3],
                           camera_intrinsics->camera_matrix[4], camera_intrinsics->camera_matrix[5],
                           camera_intrinsics->camera_matrix[6], camera_intrinsics->camera_matrix[7],
                           camera_intrinsics->camera_matrix[8]};
  double d_raw_array[8] = {
    camera_intrinsics->distortion_coeffs[0], camera_intrinsics->distortion_coeffs[1],
    camera_intrinsics->distortion_coeffs[2], camera_intrinsics->distortion_coeffs[3],
    camera_intrinsics->distortion_coeffs[4], camera_intrinsics->distortion_coeffs[5],
    camera_intrinsics->distortion_coeffs[6], camera_intrinsics->distortion_coeffs[7]};

  cv::Mat k_raw = cv::Mat(3, 3, CV_64F, k_raw_array);
  cv::Mat d_raw = cv::Mat(1, 8, CV_64F, d_raw_array);
  cv::Size img_size(image_width_, image_height_);
  cv::Mat k_rect = cv::getOptimalNewCameraMatrix(k_raw, d_raw, img_size, 0, img_size, nullptr);

  // Prepare the rectification maps
  cv::Mat r = cv::Mat::eye(3, 3, CV_32F);

  short * range_to_3d_lut = range_to_xyz_lut_fixed_point_;

  for (int y = 0; y < image_height_; y++) {
    for (int x = 0; x < image_width_; x++) {
      cv::Mat distorted_pt(1, 1, CV_32FC2, cv::Scalar(x, y));
      cv::Mat undistorted_pt(1, 1, CV_32FC2);

      cv::undistortPoints(distorted_pt, undistorted_pt, k_raw, d_raw);

      float ux = undistorted_pt.at<float>(0);
      float uy = undistorted_pt.at<float>(1);

      float scale_factor = 1.0f / sqrtf(1.0f + ux * ux + uy * uy);

      *range_to_3d_lut++ = short((ux * scale_factor) * QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE);
      *range_to_3d_lut++ = short((uy * scale_factor) * QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE);
      *range_to_3d_lut++ = short(scale_factor * QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE);
    }
  }
}

/**
 * @brief Computes point cloud using range_to_xyz_lut look up table and range image
 *
 * @param range_image Range image
 * @param xyz_frame Output frame for point cloud
 */

void ImageProcUtils::computePointCloud(unsigned short * range_image, short * xyz_frame)
{
  short * range_to_xyz_lut_fixed_point = range_to_xyz_lut_fixed_point_;
  for (int i = 0; i < image_height_; i++) {
    for (int j = 0; j < image_width_; j++) {
      *xyz_frame++ =
        ((*range_to_xyz_lut_fixed_point++) * (*range_image)) >> QFORMAT_FOR_POINTCLOUD_LUT;
      *xyz_frame++ =
        ((*range_to_xyz_lut_fixed_point++) * (*range_image)) >> QFORMAT_FOR_POINTCLOUD_LUT;
      *xyz_frame++ =
        ((*range_to_xyz_lut_fixed_point++) * (*range_image)) >> QFORMAT_FOR_POINTCLOUD_LUT;
      range_image++;
    }
  }
}

/**
 * @brief This function does the matrix multiplication
 *
 * @param input_matrix1 address of first matrix array
 * @param rows1 Number of rows in first matrix
 * @param columns1 Number of columns in first matrix
 * @param input_matrix2 address of second matrix array
 * @param rows2 Number of rows in second matrix
 * @param columns2 Number of columns in second matrix
 * @param output_matrix address of output matrix array
 */
void ImageProcUtils::matrixMultiplication(
  float * input_matrix1, int rows1, int columns1, float * input_matrix2, int rows2, int columns2,
  float * output_matrix)
{
  /*Assumption output_matrix has minimum size = rows1 x columns2*/
  /*Rule: No. of columns of matrix1 should be equal to no.of rows of matrix2*/
  if (
    input_matrix1 == nullptr || input_matrix2 == nullptr || output_matrix == nullptr ||
    columns1 != rows2) {
    return;
  }

  for (int r1 = 0; r1 < rows1; r1++) {
    for (int c2 = 0; c2 < columns2; c2++) {
      *((output_matrix + r1 * columns2) + c2) = 0;
      for (int i = 0; i < rows2; i++) {
        *((output_matrix + r1 * columns2) + c2) +=
          *((input_matrix1 + r1 * columns1) + i) * *((input_matrix2 + i * columns2) + c2);
      }
    }
  }
}

/**
 * @brief This function is the optimized version of multiplying 3x3 and 3x1 matrices.
 *
 * @param input_matrix1 address of first matrix array
 * @param input_matrix2 address of second matrix array
 * @param output_matrix address of output matrix array
 */
void ImageProcUtils::matrixMultiplication3x3And3x1(
  float * input_matrix1, short * input_matrix2, short * output_matrix)
{
  if (input_matrix1 == nullptr || input_matrix2 == nullptr || output_matrix == nullptr) {
    return;
  }

  output_matrix[0] = input_matrix1[0] * input_matrix2[0] + input_matrix1[1] * input_matrix2[1] +
                     input_matrix1[2] * input_matrix2[2];
  input_matrix1 += 3;
  output_matrix[1] = input_matrix1[0] * input_matrix2[0] + input_matrix1[1] * input_matrix2[1] +
                     input_matrix1[2] * input_matrix2[2];
  input_matrix1 += 3;
  output_matrix[2] = input_matrix1[0] * input_matrix2[0] + input_matrix1[1] * input_matrix2[1] +
                     input_matrix1[2] * input_matrix2[2];
  input_matrix1 += 3;
}

/**
 * @brief Rotates the given input pointcloud using rotation matrix provided
 *
 * @param input_point_cloud Input point cloud
 * @param rotated_point_cloud Output rotated point cloud
 * @param camera_extrinsics Camera extrinsics which provides rotation matrix
 * @param image_width Image width
 * @param image_height Image height
 */
void ImageProcUtils::rotatePointCloud(
  short * input_point_cloud, short * rotated_point_cloud, CameraExtrinsics * camera_extrinsics,
  int image_width, int image_height)
{
  float * rotation_matrix = camera_extrinsics->rotation_matrix;
  for (int i = 0; i < (image_width * image_height); i++) {
    if (input_point_cloud[2] == 0) {
      *rotated_point_cloud++ = 0;
      *rotated_point_cloud++ = 0;
      *rotated_point_cloud++ = 0;
      input_point_cloud += 3;
    } else {
      matrixMultiplication3x3And3x1(&rotation_matrix[0], input_point_cloud, rotated_point_cloud);
      input_point_cloud += 3;
      rotated_point_cloud += 3;
    }
  }
}

/**
 * @brief Detects the floor points from the point cloud using 3D Y threshold based approach
 *
 * @param xyz_frame input point cloud
 * @param floor_distance_threshold_mm Floor Distance Threshold Offset
 * @param image_width Image Width
 * @param image_height Image Height
 * @param output_depth_frame Depth image
 * @param output_xyz_frame Output Point cloud
 * @param filter_flag
 * 0: Do not remove any points from depth image and point cloud
 * 1: Remove floor pixels from depth image,
 * 2: Remove floor points from point cloud,
 * 3: Remove floor points from both depth image and point cloud
 * @param floor_mask Floor Mask Image
 * @param compressed_floor_mask Compressed Floor Mask
 */
void ImageProcUtils::detectFloorFromPointCloud(
  short * xyz_frame, float floor_distance_threshold_mm, int image_width, int image_height,
  unsigned short * output_depth_frame, short * output_xyz_frame, FilterFlag filter_flag,
  unsigned char * floor_mask, unsigned char * compressed_floor_mask)
{
  bool floor_mask_flag = false;
  bool floor_mask_compression_flag = false;
  if (floor_mask != nullptr) {
    floor_mask_flag = true;
  }
  if (compressed_floor_mask != nullptr) {
    floor_mask_compression_flag = true;
  }

  bool filter_depth_image_flag = false;
  bool filter_pointcloud_flag = false;
  if (filter_flag == RemoveFloor) {
    filter_depth_image_flag = true;
    filter_pointcloud_flag = true;
  } else if (filter_flag == RemoveFloorFromDepthImage) {
    filter_depth_image_flag = true;
  } else if (filter_flag == RemoveFloorFromPointCloud) {
    filter_pointcloud_flag = true;
  }

  if ((xyz_frame != nullptr) && (output_depth_frame != nullptr) && (output_xyz_frame != nullptr)) {
    // 3D Y value Thresholding
    for (int i = 0; i < (image_width * image_height); i++) {
      int point_cloud_index = 3 * i;
      if (*(xyz_frame + 1) >= floor_distance_threshold_mm) {
        // Update depth image
        if (filter_depth_image_flag) {
          output_depth_frame[i] = 0;
        }

        // Update point cloud
        if (filter_pointcloud_flag) {
          output_xyz_frame[point_cloud_index] = 0;
          output_xyz_frame[point_cloud_index + 1] = 0;
          output_xyz_frame[point_cloud_index + 2] = 0;
        }

        // Update floor mask
        if (floor_mask_flag) {
          floor_mask[i] = 255;
        }

        // Update compressed floor mask
        if (floor_mask_compression_flag) {
          compressed_floor_mask[i / 8] |= (1 << (i % 8));
        }
      }
      xyz_frame += 3;
    }
  }
}