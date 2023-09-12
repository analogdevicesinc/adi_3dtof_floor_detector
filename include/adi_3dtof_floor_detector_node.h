/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_3DTOF_FLOOR_DETECTOR_NODE_H
#define ADI_3DTOF_FLOOR_DETECTOR_NODE_H

#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "image_proc_utils.h"
#include "output_sensor.h"
#include "output_sensor_factory.h"
#include "floor_plane_detection.h"
#include "adtf31xx_sensor_frame_info.h"
#include "adi_3dtof_floor_detector_output_info.h"

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <utility>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <dynamic_reconfigure/server.h>
#include <adi_3dtof_floor_detector/FloorDetectorParamsConfig.h>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFFloorDetector : public ros::NodeHandle
{
public:
  /**
   * @brief Construct a new ADI3DToFFloorDetector object
   */
  ADI3DToFFloorDetector()
  {
    dynamic_reconfigure_callbacktype_ =
        boost::bind(&ADI3DToFFloorDetector::dynamicallyReconfigureVariables, this, _1, _2);

    server_.setCallback(dynamic_reconfigure_callbacktype_);

    ros::NodeHandle nh("~");

    // Get Parameters
    // ToF camera link
    std::string camera_link;
    nh.param<std::string>("param_camera_link", camera_link, "adi_camera_link");

    std::string optical_camera_link;
    nh.param<std::string>("param_optical_camera_link", optical_camera_link, "optical_camera_link");

    // Input sensor Mode: 0 - Camera/ 2 - ROSBag bin File/ 3 - RosTopic
    int input_sensor_mode;
    nh.param<int>("param_input_sensor_mode", input_sensor_mode, 0);

    // Output sensor Mode: 0 - No output/ 1 - Write output (AVI, CSV)
    int output_sensor_mode;
    nh.param<int>("param_output_sensor_mode", output_sensor_mode, 0);

    std::string input_file_name_or_ros_topic_prefix_name;
    nh.param<std::string>("param_input_file_name_or_ros_topic_prefix_name", input_file_name_or_ros_topic_prefix_name,
                          "no name");

    std::string frame_type;
    nh.param<std::string>("param_frame_type", frame_type, "no name");

    int ab_threshold;
    nh.param<int>("param_ab_threshold", ab_threshold, 10);

    int confidence_threshold;
    nh.param<int>("param_confidence_threshold", confidence_threshold, 10);

    std::string config_file_name_of_tof_sdk;
    nh.param<std::string>("param_config_file_name_of_tof_sdk", config_file_name_of_tof_sdk, "no name");

    int enable_ransac_floor_detection;
    nh.param<int>("param_enable_ransac_floor_detection", enable_ransac_floor_detection, 1);

    int enable_fallback_floor_detection;
    nh.param<int>("param_enable_fallback_floor_detection", enable_fallback_floor_detection, 1);

    float ransac_distance_threshold_mtr;
    nh.param<float>("param_ransac_distance_threshold_mtr", ransac_distance_threshold_mtr, 0.025f);

    int ransac_max_iterations;
    nh.param<int>("param_ransac_max_iterations", ransac_max_iterations, 10);

    float discard_distance_threshold_mtr;
    nh.param<float>("param_discard_distance_threshold_mtr", discard_distance_threshold_mtr, 1.5f);

    float fallback_floor_height_offset_mtr;
    nh.param<float>("param_fallback_floor_height_offset_mtr", fallback_floor_height_offset_mtr, 0.1f);

    int enable_compression_op_image_topics;
    nh.param<int>("param_enable_compression_op_image_topics", enable_compression_op_image_topics, 0);

    // Do not change the default values of the below 3 parameters
    int input_image_width;
    nh.param<int>("param_input_image_width", input_image_width, 1024);

    int input_image_height;
    nh.param<int>("param_input_image_height", input_image_height, 1024);

    int processing_scale;
    nh.param<int>("param_processing_scale", processing_scale, 2);

    camera_link_ = std::move(camera_link);
    optical_camera_link_ = std::move(optical_camera_link);
    input_sensor_mode_ = input_sensor_mode;
    output_sensor_mode_ = output_sensor_mode;
    input_file_name_or_ros_topic_prefix_name_ = std::move(input_file_name_or_ros_topic_prefix_name);
    fallback_floor_height_offset_mtr_ = fallback_floor_height_offset_mtr;
    frame_number_ = 0;
    enable_ransac_floor_detection_ = (enable_ransac_floor_detection == 1) ? true : false;
    enable_fallback_floor_detection_ = (enable_fallback_floor_detection == 1) ? true : false;
    enable_compression_op_image_topics_ = (enable_compression_op_image_topics == 1) ? true : false;
    ransac_distance_threshold_mtr_ = ransac_distance_threshold_mtr;
    discard_distance_threshold_mtr_ = discard_distance_threshold_mtr;
    ransac_max_iterations_ = ransac_max_iterations;
    ab_threshold_ = ab_threshold;
    confidence_threshold_ = confidence_threshold;
    processing_scale_ = processing_scale;

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    input_sensor_->openSensor(input_file_name_or_ros_topic_prefix_name_, input_image_width, input_image_height,
                              processing_scale_, config_file_name_of_tof_sdk);
    if (!input_sensor_->isOpened())
    {
      ROS_ERROR("Could not open the sensor %s", input_file_name_or_ros_topic_prefix_name_.c_str());
      shutDownAllNodes();
    }

    // Configure the sensor
    input_sensor_->configureSensor(frame_type);
    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();

    // Buffer allocation
    floor_mask_ = new unsigned char[image_width_ * image_height_];
    compressed_floor_pixels_ = (image_width_ * image_height_) / 8;
    if (enable_compression_op_image_topics_)
    {
      compressed_floor_mask_ = new unsigned char[compressed_floor_pixels_];
    }

    // Get output sensor module
    output_sensor_ = OutputSensorFactory::getOutputSensor(output_sensor_mode_);
    if (output_sensor_ != nullptr)
    {
      output_sensor_->open(input_file_name_or_ros_topic_prefix_name_, image_width_, image_height_, EnableAllOutputs);
    }

    // Create TF listerner instance
    optical_map_tf_listener_ = new tf2_ros::TransformListener(optical_map_tf_buffer_);
    camera_map_tf_listener_ = new tf2_ros::TransformListener(camera_map_tf_buffer_);

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    // Get camera link TFs
    getCameraLinksTF();

    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);
    floor_plane_detection_ =
        new FloorPlaneDetection(image_width_, image_height_, ransac_distance_threshold_mtr_, ransac_max_iterations_,
                                discard_distance_threshold_mtr_, camera_height_mtr_);

    // Publishers
    if (enable_compression_op_image_topics_ == true)
    {
      depth_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("depth_image/compressedDepth", 10);
      ir_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("ir_image/compressedDepth", 10);
      floor_mask_publisher_ = this->advertise<sensor_msgs::CompressedImage>("compressed_floor_mask", 10);
    }
    else
    {
      depth_image_publisher_ = this->advertise<sensor_msgs::Image>("depth_image", 10);
      ir_image_publisher_ = this->advertise<sensor_msgs::Image>("ir_image", 10);
      floor_mask_publisher_ = this->advertise<sensor_msgs::Image>("floor_mask", 10);
    }
    depth_info_publisher_ = this->advertise<sensor_msgs::CameraInfo>("camera_info", 10);
    if (enable_pointcloud_publisher_)
    {
      xyz_image_publisher_ = this->advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    }

    // For File-io, we do not want to miss any frame, so increasing the queue size.
    if (input_sensor_mode_ != 0)
    {
      max_input_queue_length_ = 100;
      max_output_queue_length_ = 100;
    }

    // setting ab thresold to pulsatrix
    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    // setting confidence threshold to pulsatrix
    input_sensor_->setConfidenceThreshold(confidence_threshold_);

    // Initially setting dynamic reconfigure values to same as launch file
    initSettingsForDynamicReconfigure();
  }

  void shutDownAllNodes();

  bool runFloorDetection();

  void processOutput();

  void processOutputAbort();

  void readInputAbort();

  void readInput();

  ADTF31xxSensorFrameInfo* floorDetectorIOThreadGetNextFrame(void);

  ADI3DToFFloorDetectorOutputInfo* floorDetectorIOThreadGetNext(void);

  void dynamicallyReconfigureVariables(adi_3dtof_floor_detector::FloorDetectorParamsConfig& config, uint32_t level);

  void updateDynamicReconfigureVariablesInputThread();

  void updateDynamicReconfigureVariablesProcessThread();

  /**
   * @brief Destroy the ADI3DToFFloorDetector object
   *
   */
  ~ADI3DToFFloorDetector()
  {
    // Close the input sensor
    input_sensor_->closeSensor();

    delete optical_map_tf_listener_;
    delete camera_map_tf_listener_;
    delete[] floor_mask_;
    if (enable_compression_op_image_topics_)
    {
      delete[] compressed_floor_mask_;
    }

    delete image_proc_utils_;
    delete floor_plane_detection_;

    // Close outputs
    if (output_sensor_ != nullptr)
    {
      output_sensor_->close();
    }
  }

private:
  IInputSensor* input_sensor_;
  OOutputSensor* output_sensor_;
  ImageProcUtils* image_proc_utils_;
  FloorPlaneDetection* floor_plane_detection_;
  int image_width_;
  int image_height_;
  int frame_number_;
  int input_sensor_mode_;
  int output_sensor_mode_;
  std::string camera_link_;
  std::string optical_camera_link_;
  bool enable_compression_op_image_topics_ = false;
  bool enable_pointcloud_publisher_ = false;
  int ab_threshold_ = 10;
  int confidence_threshold_ = 10;
  std::string input_file_name_or_ros_topic_prefix_name_;
  std::string output_file_name_;
  tf2_ros::Buffer optical_map_tf_buffer_;
  tf2_ros::Buffer camera_map_tf_buffer_;
  tf2_ros::TransformListener* optical_map_tf_listener_;
  tf2_ros::TransformListener* camera_map_tf_listener_;
  sensor_msgs::CameraInfo cam_info_msg_;
  unsigned short* depth_frame_ = nullptr;
  unsigned short* ir_frame_ = nullptr;
  short* xyz_frame_ = nullptr;
  short* input_xyz_frame_ = nullptr;
  short* output_xyz_frame_ = nullptr;
  int compressed_depth_frame_size_ = 0;
  int compressed_ir_frame_size_ = 0;
  unsigned char* compressed_depth_frame_ = nullptr;
  unsigned char* compressed_ir_frame_ = nullptr;
  ros::Publisher depth_image_publisher_;
  ros::Publisher ir_image_publisher_;
  ros::Publisher xyz_image_publisher_;
  ros::Publisher depth_info_publisher_;
  ros::Publisher floor_mask_publisher_;
  CameraIntrinsics depth_intrinsics_;
  CameraExtrinsics depth_extrinsics_;
  CameraExtrinsics depth_extrinsics_external_;

  bool process_output_thread_abort_ = false;
  bool read_input_thread_abort_ = false;

  boost::mutex output_thread_mtx_;
  boost::mutex input_thread_mtx_;

  /*Floor Detection Variables*/
  /**
   * @brief Variable to choose which floor detection algorithm to run
   * 1. RANSAC based (default)
   * 2. Threshold Y based
   *
   */
  bool enable_ransac_floor_detection_;

  /**
   * @brief Enable Fallback floor detection algorithm(Threshold Y based) if RANSAC fails
   *
   */
  bool enable_fallback_floor_detection_;

  /**
   * @brief Result status of RANSAC floor detection
   * If the status is true then RANSAC result can be considered, otherwise, run the Threshold Y based floor detection.
   *
   */
  bool ransac_floor_detection_status_ = false;

  /**
   * @brief Rotation around X axis in radian
   *
   */
  float camera_roll_rad_ = 0.0f;

  /**
   * @brief Rotation around Y axis in radian
   *
   */
  float camera_pitch_rad_ = 0.0f;

  /**
   * @brief Rotation around Z axis in radian
   *
   */
  float camera_yaw_rad_ = 0.0f;

  /**
   * @brief Flag for camera rotation
   *
   */
  bool camera_tilted_ = false;

  /**
   * @brief distance (25mm) which determines how close the point must be to the RANSAC plane in order to be selected as
   * inlier.
   *
   */
  float ransac_distance_threshold_mtr_ = 0.025f;

  /**
   * @brief Maximum number of RANSAC iterations (10) which is allowed
   *
   */
  int ransac_max_iterations_ = 10;

  /**
   * @brief Camera height, default is 0.1524m (6 inches)
   *
   */
  float camera_height_mtr_ = 0.1524f;

  /**
   * @brief Threshold to filter the point cloud based on depth (Z)
   * Select the points with depth value lesser than 1.5m from the sensor
   *
   */
  float discard_distance_threshold_mtr_ = 1.5f;

  /**
   * @brief Points which are lower than floor
   *
   */
  int noise_count_ = 0;

  /**
   * @brief Number of iterations RANSAC has taken for floor detection
   *
   */
  int ransac_iterations_ = 0;

  /**
   * @brief Time (ms) RANSAC has taken for floor detection
   *
   */
  float ransac_time_ms_ = 0.0f;

  /**
   * @brief Floor height with offset for Threshold Y based floor detection.
   *
   */
  float fallback_floor_height_offset_mtr_;

  /**
   * @brief Floor distnace threhsold wrt sensor
   *
   */
  float floor_distance_threshold_mm_;

  /**
   * @brief Floor mask
   *
   */
  unsigned char* floor_mask_ = nullptr;

  /**
   * @brief Compressed floor mask
   *
   */
  unsigned char* compressed_floor_mask_ = nullptr;

  /**
   * @brief Compressed floor pixels count
   *
   */
  int compressed_floor_pixels_ = 0;

  /**
   * @brief    This is the scale factor to scale the input image.
               The processing will happen on the scaled down image
               The topics and the output coordinates will correspond
               to the scaled image size.
   *
   */
  int processing_scale_ = 2;

  ros::Time curr_frame_timestamp_ = ros::Time::now();

  int max_input_queue_length_ = 1;

  std::queue<ADTF31xxSensorFrameInfo*> input_frames_queue_;

  int max_output_queue_length_ = 5;

  std::queue<ADI3DToFFloorDetectorOutputInfo*> output_node_queue_;

  bool error_in_frame_read_ = false;

  dynamic_reconfigure::Server<adi_3dtof_floor_detector::FloorDetectorParamsConfig> server_;

  dynamic_reconfigure::Server<adi_3dtof_floor_detector::FloorDetectorParamsConfig>::CallbackType
      dynamic_reconfigure_callbacktype_;

  adi_3dtof_floor_detector::FloorDetectorParamsConfig dynamic_reconfigure_config_;

  void initSettingsForDynamicReconfigure();

  void getCameraLinksTF();

  void fillAndPublishCameraInfo(const std::string& frame_id, const ros::Publisher& publisher);

  void publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                            const ros::Publisher& publisher, bool enable_image_compression);

  void publishPointCloud(short* xyz_frame);

  void publishImageAndCameraInfo(unsigned short* depth_frame, unsigned short* ir_frame, unsigned char* floor_mask_frame,
                                 short* xyz_frame);

  void publishImageAndCameraInfo(unsigned char* compressed_depth_frame, int compressed_depth_frame_size,
                                 unsigned char* compressed_ir_frame, int compressed_ir_frame_size,
                                 unsigned char* compressed_floor_mask_frame, int compressed_floor_mask_frame_size,
                                 short* xyz_frame);

  bool processOutputDone();

  void processOutputStart();

  void floorDetectorIOThreadPushOutputNode(ADI3DToFFloorDetectorOutputInfo* new_output_node);

  void publishCompressedImageAsRosMsg(unsigned char* compressed_img, int compressed_img_size,
                                      const std::string& encoding_type, std::string frame_id,
                                      const ros::Publisher& publisher);

  void rotatePointCloud(short* input_point_cloud, short* rotated_point_cloud, CameraExtrinsics* camera_extrinsics,
                        int image_width, int image_height);

  cv::Mat getFloorDetectionOutput(unsigned short* depth_frame_with_floor, unsigned short* ir_frame,
                                  unsigned char* floor_mask_8bit);
};

#endif
