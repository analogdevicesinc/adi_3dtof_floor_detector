/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef CUSTOM_SAC_MODEL_PLANE_H
#define CUSTOM_SAC_MODEL_PLANE_H

#pragma once

#include "custom_sac_model.h"
#include <pcl/sample_consensus/model_types.h>

namespace pcl
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief CustomSampleConsensusModelPlane defines a model for 3D plane segmentation.
 * The model coefficients are defined as:
 *   - \b a : the X coordinate of the plane's normal (normalized)
 *   - \b b : the Y coordinate of the plane's normal (normalized)
 *   - \b c : the Z coordinate of the plane's normal (normalized)
 *   - \b d : the fourth <a href="http://mathworld.wolfram.com/HessianNormalForm.html">Hessian component</a> of the
 * plane's equation
 *
 * \author Radu B. Rusu
 * \ingroup sample_consensus
 */
template <typename PointT>
class CustomSampleConsensusModelPlane : public CustomSampleConsensusModel<PointT>
{
public:
  using CustomSampleConsensusModel<PointT>::model_name_;
  using CustomSampleConsensusModel<PointT>::input_;
  using CustomSampleConsensusModel<PointT>::indices_;
  using CustomSampleConsensusModel<PointT>::error_sqr_dists_;
  using CustomSampleConsensusModel<PointT>::isModelValid;

  using PointCloud = typename CustomSampleConsensusModel<PointT>::PointCloud;
  using PointCloudPtr = typename CustomSampleConsensusModel<PointT>::PointCloudPtr;
  using PointCloudConstPtr = typename CustomSampleConsensusModel<PointT>::PointCloudConstPtr;

  using Ptr = shared_ptr<CustomSampleConsensusModelPlane<PointT>>;
  using ConstPtr = shared_ptr<const CustomSampleConsensusModelPlane<PointT>>;

  /** \brief Constructor for base CustomSampleConsensusModelPlane.
   * \param[in] cloud the input point cloud dataset
   * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
   */
  CustomSampleConsensusModelPlane(const PointCloudConstPtr& cloud, bool random = false)
    : CustomSampleConsensusModel<PointT>(cloud, random)
  {
    model_name_ = "CustomSampleConsensusModelPlane";
    sample_size_ = 3;
    model_size_ = 4;
  }

  /** \brief Constructor for base CustomSampleConsensusModelPlane.
   * \param[in] cloud the input point cloud dataset
   * \param[in] indices a vector of point indices to be used from \a cloud
   * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
   */
  CustomSampleConsensusModelPlane(const PointCloudConstPtr& cloud, const std::vector<int>& indices, bool random = false)
    : CustomSampleConsensusModel<PointT>(cloud, indices, random)
  {
    model_name_ = "CustomSampleConsensusModelPlane";
    sample_size_ = 3;
    model_size_ = 4;
  }

  /** \brief Empty destructor */
  ~CustomSampleConsensusModelPlane()
  {
  }

  /** \brief Check whether the given index samples can form a valid plane model, compute the model coefficients from
   * these samples and store them internally in model_coefficients_. The plane coefficients are:
   * a, b, c, d (ax+by+cz+d=0)
   * \param[in] samples the point indices found as possible good candidates for creating a valid model
   * \param[out] model_coefficients the resultant model coefficients
   */
  bool computeModelCoefficients(const std::vector<int>& samples, Eigen::VectorXf& model_coefficients) const override;

  /** \brief Compute all distances from the cloud data to a given plane model.
   * \param[in] model_coefficients the coefficients of a plane model that we need to compute distances to
   * \param[out] distances the resultant estimated distances
   */
  void getDistancesToModel(const Eigen::VectorXf& model_coefficients, std::vector<double>& distances) const override;

  /** \brief Select all the points which respect the given model coefficients as inliers.
   * \param[in] model_coefficients the coefficients of a plane model that we need to compute distances to
   * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
   * \param[out] inliers the resultant model inliers
   */
  void selectWithinDistance(const Eigen::VectorXf& model_coefficients, const double threshold,
                            std::vector<int>& inliers) override;

  /** \brief Select all the points which respect the given model coefficients as inliers.
   * \param[in] model_coefficients the coefficients of a plane model that we need to compute distances to
   * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
   * \param[in] ptcld_buffer input point cloud buffer
   * \param[out] inliers the resultant model inliers
   */
  void selectWithinDistance(const Eigen::VectorXf& model_coefficients, const double threshold,
                            std::vector<int>& inliers, float* ptcld_buffer) override;

  /** \brief Count all the points which respect the given model coefficients as inliers.
   *
   * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
   * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
   * \return the resultant number of inliers
   */
  std::size_t countWithinDistance(const Eigen::VectorXf& model_coefficients, const double threshold) const override;

  /** \brief Count all the points which respect the given model coefficients as inliers.
   *
   * \param[in] model_coefficients the coefficients of a model that we need to compute distances to
   * \param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
   * \param[in] ptcld_buffer input point cloud buffer
   * \return the resultant number of inliers
   */
  std::size_t countWithinDistance(const Eigen::VectorXf& model_coefficients, const double threshold,
                                  float* ptcld_buffer) const override;

  /** \brief Recompute the plane coefficients using the given inlier set and return them to the user.
   * @note: these are the coefficients of the plane model after refinement (e.g. after SVD)
   * \param[in] inliers the data inliers found as supporting the model
   * \param[in] model_coefficients the initial guess for the model coefficients
   * \param[out] optimized_coefficients the resultant recomputed coefficients after non-linear optimization
   */
  void optimizeModelCoefficients(const std::vector<int>& inliers, const Eigen::VectorXf& model_coefficients,
                                 Eigen::VectorXf& optimized_coefficients) const override;

  /** \brief Create a new point cloud with inliers projected onto the plane model.
   * \param[in] inliers the data inliers that we want to project on the plane model
   * \param[in] model_coefficients the *normalized* coefficients of a plane model
   * \param[out] projected_points the resultant projected points
   * \param[in] copy_data_fields set to true if we need to copy the other data fields
   */
  void projectPoints(const std::vector<int>& inliers, const Eigen::VectorXf& model_coefficients,
                     PointCloud& projected_points, bool copy_data_fields = true) const override;

  /** \brief Verify whether a subset of indices verifies the given plane model coefficients.
   * \param[in] indices the data indices that need to be tested against the plane model
   * \param[in] model_coefficients the plane model coefficients
   * \param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
   */
  bool doSamplesVerifyModel(const std::set<int>& indices, const Eigen::VectorXf& model_coefficients,
                            const double threshold) const override;

  /** \brief Return a unique id for this model (SACMODEL_PLANE). */
  inline pcl::SacModel getModelType() const override
  {
    return (SACMODEL_PLANE);
  }

protected:
  using CustomSampleConsensusModel<PointT>::sample_size_;
  using CustomSampleConsensusModel<PointT>::model_size_;

private:
  /** \brief Check if a sample of indices results in a good sample of points
   * indices.
   * \param[in] samples the resultant index samples
   */
  bool isSampleGood(const std::vector<int>& samples) const override;
};
}  // namespace pcl

#endif