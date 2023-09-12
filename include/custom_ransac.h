/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/*
 * This file has been modifed by Analog Devices, Inc. on 5th September, 2022.
 * Portions Copyright © <2022-2023> Analog Devices, Inc.
 */

#ifndef CUSTOM_RANSAC_H
#define CUSTOM_RANSAC_H

#pragma once

#include "custom_sac.h"
#include "custom_sac_model.h"

namespace pcl
{
/** \brief @b CustomRandomSampleConsensus represents an implementation of the RANSAC (RANdom SAmple Consensus)
 * algorithm, as described in: "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image
 * Analysis and Automated Cartography", Martin A. Fischler and Robert C. Bolles, Comm. Of the ACM 24: 381–395, June
 * 1981. A parallel variant is available, enable with setNumberOfThreads. Default is non-parallel. \author Radu B. Rusu
 * \ingroup sample_consensus
 */
template <typename PointT>
class CustomRandomSampleConsensus : public CustomSampleConsensus<PointT>
{
  using CustomSampleConsensusModelPtr = typename CustomSampleConsensusModel<PointT>::Ptr;

public:
  using Ptr = shared_ptr<CustomRandomSampleConsensus<PointT> >;
  using ConstPtr = shared_ptr<const CustomRandomSampleConsensus<PointT> >;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  using CustomSampleConsensus<PointT>::max_iterations_;
  using CustomSampleConsensus<PointT>::threshold_;
  using CustomSampleConsensus<PointT>::iterations_;
  using CustomSampleConsensus<PointT>::sac_model_;
  using CustomSampleConsensus<PointT>::model_;
  using CustomSampleConsensus<PointT>::model_coefficients_;
  using CustomSampleConsensus<PointT>::inliers_;
  using CustomSampleConsensus<PointT>::probability_;
  using CustomSampleConsensus<PointT>::threads_;
  using CustomSampleConsensus<PointT>::minimum_pointcloud_size_;
  using CustomSampleConsensus<PointT>::ptcld_buffer_;
  using CustomSampleConsensus<PointT>::noise_count;
  using CustomSampleConsensus<PointT>::ransac_iterations;

  /*Angle offset*/
  float delta_ = 20.0f;
  /* Minimum angle between the camera plane and RANSAC plane*/
  float min_threshold_for_angle_bw_planes_ = 0;
  /* Maximum angle between the camera plane and RANSAC plane*/
  float max_threshold_for_angle_bw_planes_ = 0;
  /* Best angle between the camera plane and RANSAC plane*/
  float best_angle_bw_planes_ = 0;

  /** \brief RANSAC (RAndom SAmple Consensus) main constructor
   * \param[in] model a Sample Consensus model
   */
  CustomRandomSampleConsensus(const CustomSampleConsensusModelPtr& model) : CustomSampleConsensus<PointT>(model)
  {
    // Maximum number of trials before we give up.
    max_iterations_ = 10000;
  }

  /** \brief RANSAC (RAndom SAmple Consensus) main constructor
   * \param[in] model a Sample Consensus model
   * \param[in] threshold distance to model threshold
   */
  CustomRandomSampleConsensus(const CustomSampleConsensusModelPtr& model, double threshold)
    : CustomSampleConsensus<PointT>(model, threshold)
  {
    // Maximum number of trials before we give up.
    max_iterations_ = 10000;
  }

  /** \brief Compute the actual model and find the inliers
   * \param[in] debug_verbosity_level enable/disable on-screen debug information and set the verbosity level
   */
  bool computeModel(int debug_verbosity_level = 0) override;

private:
  float getAngleBwCameraAndFloorPlanes(Eigen::VectorXf model_coefficients);
  bool validateFloorDetection(Indices& inliers, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr);
  bool validateInliers(Indices& inliers, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr);
  bool GenerateFilteredPointCloud(Indices& inliers, PointCloudConstPtr original_cloud_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr);
};
}  // namespace pcl

#endif
