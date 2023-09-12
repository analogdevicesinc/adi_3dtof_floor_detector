/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef FLOOR_PLANE_DETECTION_H
#define FLOOR_PLANE_DETECTION_H

#include "custom_sac_segmentation.h"

enum FilterFlag
{
  // 0: Do not remove any points from depth image and point cloud
  DoNotRemoveFloor,
  // 1: Remove floor pixels from depth image,
  RemoveFloorFromDepthImage,
  // 2: Remove floor points from point cloud,
  RemoveFloorFromPointCloud,
  // 3: Remove floor points from both depth image and point cloud
  RemoveFloor
};

/**
 * @brief Class for Floor Plane Detection
 *
 */
class FloorPlaneDetection
{
public:
  /**
   * @brief Construct a new Floor Plane Detection object
   *
   * @param image_width Original image width should be 512
   * @param image_height Original image height should be 512
   * @param ransac_distance_threshold_mtr Distance which determines how close the point must be to the RANSAC plane in
   * order to be selected as inlier.
   * @param max_iterations Maximum number of RANSAC iterations which is allowed
   * @param discard_distance_threshold_mtr Threshold to filter the point cloud based on depth (Z)
   * @param camera_height_mtr Threshold to filter the point cloud based on height (Y), to disable this pass 0
   */
  FloorPlaneDetection(int image_width = 512, int image_height = 512, float ransac_distance_threshold_mtr = 0.025f,
                      int max_iterations = 10, float discard_distance_threshold_mtr = 1.5f,
                      float camera_height_mtr = 0.17f)
    : preprocessed_pointcloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    image_width_ = image_width;
    image_height_ = image_height;
    ransac_distance_threshold_mm_ = ransac_distance_threshold_mtr * 1000;
    ransac_max_iterations_ = max_iterations;
    discard_distance_threshold_mm_ = discard_distance_threshold_mtr * 1000;
    camera_height_mtr_ = camera_height_mtr;

    // Default image size is 512x512
    scaled_image_width_ = image_width / ransac_scale_factor_;
    scaled_image_height_ = image_height / ransac_scale_factor_;

    modified_point_cloud_ = new float[scaled_image_width_ * scaled_image_height_ * 3];

    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setOptimizeCoefficients(true);
    seg_.setNumberOfThreads(-1);
    seg_.setMinmimumPointCloudSize(minimum_pointcloud_size_);
  }

  /**
   * @brief Destroy the Floor Plane Detection object
   *
   */
  ~FloorPlaneDetection()
  {
    if (modified_point_cloud_ != NULL)
    {
      delete[] modified_point_cloud_;
    }
  }

  float getRANSACDistanceThreshold();
  void setRANSACDistanceThreshold(float ransac_distance_threshold_mtr);
  int getMaxIterations();
  void setMaxIterations(int max_iterations);
  float getDiscardDistanceThreshold();
  void setDiscardDistanceThreshold(float discard_distance_threshold_mtr);
  float getCameraHeight();
  void setCameraHeight(float camera_height_mtr);

  int preProcessPointCloud(short* input_point_cloud, float* modified_point_cloud, bool* nearby_objects_flag);
  bool detectFloorUsingEnhancedRANSAC(unsigned short* output_depth_image, short* output_point_cloud,
                                      FilterFlag filter_flag, unsigned char* floor_mask,
                                      unsigned char* compressed_floor_mask, short* input_point_cloud,
                                      float* modified_point_cloud, int modified_point_cloud_size,
                                      bool nearby_objects_flag);

  int getRANSACIterations();
  int getNoiseCount();

private:
  int image_width_;
  int image_height_;
  int scaled_image_width_;
  int scaled_image_height_;
  int ransac_max_iterations_;
  float camera_height_mtr_;
  float ransac_distance_threshold_mm_;
  float discard_distance_threshold_mm_;
  float* modified_point_cloud_;

  int ransac_scale_factor_ = 2;
  short nearby_object_threshold_distance_mm_ = 500;
  int nearby_oblect_pixels_count_threshold_ = (int)(2000 / std::pow(ransac_scale_factor_, 2));
  int minimum_pointcloud_size_ = (int)(512 / ransac_scale_factor_);
  bool nearby_objects_flag_ = false;
  float point_cloud_height_threshold_delta_mtr_ = 0.1f;
  FilterFlag filter_flag_ = DoNotRemoveFloor;
  short* input_point_cloud_ = nullptr;
  int noise_count_ = 0;
  int iterations_ = 0;

  pcl::CustomSACSegmentation<pcl::PointXYZ> seg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_pointcloud_ptr_;

  bool detectFloor(unsigned short* output_depth_image, short* output_point_cloud, FilterFlag filter_flag,
                   unsigned char* floor_mask, unsigned char* compressed_floor_mask);

  void postProcessPointCloud(pcl::ModelCoefficients& model_coefficients, unsigned short* output_depth_image,
                             short* output_point_cloud, unsigned char* floor_mask,
                             unsigned char* compressed_floor_mask);
};

#endif
