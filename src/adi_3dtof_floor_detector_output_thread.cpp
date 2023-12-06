/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <chrono>
#include <thread>

#include "adi_3dtof_floor_detector_node.h"
#include "module_profile.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.h"

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This function sets the abort flag for the output thread,
 * the function is normally called by the main function to abort the output thread.
 *
 */
void ADI3DToFFloorDetector::processOutputAbort() { process_output_thread_abort_ = true; }

/**
 * @brief This function pushes the output node to the output queue.
 * If the queue is full, then the last item in the queue gets replaced
 * with the latest node.
 *
 * @param new_output_node : Pointer to the output node to be published
 */
void ADI3DToFFloorDetector::floorDetectorIOThreadPushOutputNode(
  ADI3DToFFloorDetectorOutputInfo * new_output_node)
{
  output_thread_mtx_.lock();
  int queue_size = output_node_queue_.size();
  output_thread_mtx_.unlock();

  if (queue_size <= (max_output_queue_length_ - 1)) {
    // Push this one
    output_thread_mtx_.lock();
    output_node_queue_.push(new_output_node);
    output_thread_mtx_.unlock();
  } else {
    __attribute__((unused)) ADI3DToFFloorDetectorOutputInfo * last_node = nullptr;
    // Replace the last item with the current one.
    output_thread_mtx_.lock();
    last_node = (ADI3DToFFloorDetectorOutputInfo *)output_node_queue_.back();
    output_thread_mtx_.unlock();

    // Copy the contents of new node into the old one and then delete the new node.
    last_node = new_output_node;

    // Delete this one
    delete new_output_node;
  }
}

/**
 * @brief The output process function, this is running a loop
 * which reads the frame from the putput queue, generates the visualization output
 * and publishes the output as ROS messages.
 *
 */
void ADI3DToFFloorDetector::processOutput()
{
  int output_queue_size = output_node_queue_.size();

  while ((!process_output_thread_abort_) || (output_queue_size > 0)) {
    // std::cout << "Inside processOutput" << std::endl;
    output_thread_mtx_.lock();
    output_queue_size = output_node_queue_.size();
    output_thread_mtx_.unlock();
    if (output_queue_size > 0) {
      PROFILE_FUNCTION_START(FLOOR_DETECTOR_9_POSTPROCESS)
      output_thread_mtx_.lock();
      ADI3DToFFloorDetectorOutputInfo * new_frame =
        (ADI3DToFFloorDetectorOutputInfo *)output_node_queue_.front();
      output_node_queue_.pop();
      output_thread_mtx_.unlock();

      // Publish
      if (enable_compression_op_image_topics_) {
        PROFILE_FUNCTION_START(FLOOR_DETECTOR_4_COMPRESS_DEPTH_IR)
        // IR compression
        compressed_depth_image_transport::RvlCodec rvl;
        new_frame->compressed_ir_frame_size_ = rvl.CompressRVL(
          &new_frame->ir_frame_[0], &new_frame->compressed_ir_frame_[0],
          image_width_ * image_height_);
        PROFILE_FUNCTION_END(FLOOR_DETECTOR_4_COMPRESS_DEPTH_IR)

        publishImageAndCameraInfo(
          new_frame->compressed_depth_frame_, new_frame->compressed_depth_frame_size_,
          new_frame->compressed_ir_frame_, new_frame->compressed_ir_frame_size_,
          new_frame->compressed_floor_mask_, new_frame->compressed_floor_mask_size_,
          new_frame->xyz_frame_);
      } else {
        publishImageAndCameraInfo(
          new_frame->depth_frame_, new_frame->ir_frame_, new_frame->floor_mask_8bit_,
          new_frame->xyz_frame_);
      }

      // Write outputs
      if (output_sensor_ != nullptr) {
        // floor detection output (depth, ir, floor marked depth image)
        cv::Mat floor_detection_op_image = getFloorDetectionOutput(
          new_frame->depth_frame_, new_frame->ir_frame_, new_frame->floor_mask_8bit_);
        output_sensor_->write(
          new_frame->frame_number_, floor_detection_op_image,
          new_frame->ransac_floor_detection_status_, new_frame->ransac_iterations_,
          new_frame->noise_count_, new_frame->ransac_time_ms_);
      }

      delete new_frame;
      PROFILE_FUNCTION_END(FLOOR_DETECTOR_9_POSTPROCESS)
    }

    // Sleep
    std::this_thread::sleep_for(2ms);
  }
}
