/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "output_sensor_file.h"

#include "image_proc_utils.h"

/**
 * @brief Opens output files (video and/or csv)
 *
 * @param input_file_name Input file name
 * @param image_width Image Width
 * @param image_height Image Height
 * @param output_flag Output flag to save outputs
 */
void OutputSensorFile::open(
  std::string input_file_name, int image_width, int image_height, OutputFlag output_flag)
{
  if (output_flag == EnableCSVOutputOnly) {
    csv_enabled_ = true;
  } else if (output_flag == EnableVideoOutputOnly) {
    video_enabled_ = true;
  } else if (output_flag == EnableAllOutputs) {
    csv_enabled_ = true;
    video_enabled_ = true;
  }

  if (csv_enabled_) {
    // Open the output csv file
    openOutputCsvFile(input_file_name);
  }

  if (video_enabled_) {
    // Open output video file
    // Concatenate Depth, IR, Floor marked Depth image
    openOutputVideoFile(input_file_name, image_width * 3, image_height);
  }
}

/**
 * @brief Writes output files (video and/or csv)
 *
 * @param frame_number Frame Identifier
 * @param m_out_visualization_image Output frame for video write
 * @param floor_detection_status RANSAC status
 * @param ransac_iterations RANSAC iterations
 * @param noise_count Noise count in Pointcloud
 * @param ransac_time_out time taken for RANSAC process
 */
void OutputSensorFile::write(
  int frame_number, const cv::Mat & m_out_visualization_image, bool floor_detection_status,
  int ransac_iterations, int noise_count, float ransac_time_out)
{
  if (csv_enabled_) {
    writeOutputCsvFile(
      frame_number, floor_detection_status, ransac_iterations, noise_count, ransac_time_out);
  }

  if (video_enabled_) {
    writeOutputVideoFile(m_out_visualization_image);
  }
}

/**
 * @brief Closes all opened output files
 *
 */
void OutputSensorFile::close()
{
  if (csv_enabled_) {
    closeOutputCsvFile();
  }

  if (video_enabled_) {
    closeOutputVideoFile();
  }
}

/**
 * @brief Opens output csv file
 *
 * @param input_file_name Input file name
 */
void OutputSensorFile::openOutputCsvFile(const std::string & input_file_name)
{
  output_csv_file_name_ = input_file_name.substr(0, input_file_name.find_last_of('.')) + ".csv";

  // Open file for streaming.
  output_csv_file_.open(output_csv_file_name_, std::ios::out);
  if (output_csv_file_.is_open()) {
    // Update flag.
    output_csv_file_ << "frame_number"
                     << ","
                     << "ransac_floor_detection_status"
                     << ","
                     << "ransac_iterations"
                     << ","
                     << "noise_count"
                     << ","
                     << "ransac_time_in_ms" << std::endl;
  } else {
    std::cout << "Could not open output csv file for the input " << input_file_name << std::endl;
  }
}

/**
 * @brief Opens output video file
 *
 * @param input_file_name Input file name
 * @param image_width Image Width
 * @param image_height Image Height
 */
void OutputSensorFile::openOutputVideoFile(
  const std::string & input_file_name, int image_width, int image_height)
{
  output_video_file_name_ = input_file_name.substr(0, input_file_name.find_last_of('.')) + ".avi";
  output_video_writer_ = new cv::VideoWriter(
    output_video_file_name_, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'), 10,
    cv::Size(image_width, image_height), true);
  if (!output_video_writer_->isOpened()) {
    std::cout << "Could not open output video file for the input " << input_file_name << std::endl;
  }
}

/**
 * @brief Write outputs into csv file
 *
 * @param frame_number Frame number
 * @param floor_detection_status RANSAC detection status
 * @param ransac_iterations RANSAC iterations needed to generate output
 * @param noise_count Noisy pixels count below RANSAC plane
 * @param ransac_time_ms_out Time took by RANSAC to generate floor plane
 */
void OutputSensorFile::writeOutputCsvFile(
  int frame_number, bool floor_detection_status, int ransac_iterations, int noise_count,
  float ransac_time_ms_out)
{
  if (output_csv_file_.is_open()) {
    output_csv_file_ << frame_number << "," << floor_detection_status << "," << ransac_iterations
                     << "," << noise_count << "," << ransac_time_ms_out << std::endl;
  }
}

/**
 * @brief Writes output video file
 *
 * @param image Output image
 */
void OutputSensorFile::writeOutputVideoFile(const cv::Mat & image)
{
  if (output_video_writer_->isOpened()) {
    output_video_writer_->write(image);
  }
}

/**
 * @brief closes the file
 *
 */
void OutputSensorFile::closeOutputCsvFile()
{
  if (output_csv_file_.is_open()) {
    output_csv_file_.close();
  }
}

/**
 * @brief Closes output video file
 *
 */
void OutputSensorFile::closeOutputVideoFile()
{
  if (output_video_writer_->isOpened()) {
    output_video_writer_->release();
    output_video_writer_ = nullptr;
  }
}