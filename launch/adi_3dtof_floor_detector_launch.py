import os
import json
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

#configuration and ini file names
config_json_file_names = ['config_crosby_old_modes.json', 'config_crosby_adsd3500_new_modes.json']
config_ini_file_names = ['RawToDepthAdsd3500_qmp.ini', 'RawToDepthAdsd3500_lr-qnative.ini']

package_dir = get_package_share_directory('adi_3dtof_floor_detector') + "/../../../../src/adi_3dtof_floor_detector/"

def modify_ini_path_in_json_file(config_file_name_of_tof_sdk):
    with open(config_file_name_of_tof_sdk, 'r+') as file:
        data = json.load(file)

    for i in range(len(config_json_file_names)):
        if (config_file_name_of_tof_sdk.rsplit("/", 1)[1] == config_json_file_names[i]):
            modified_file_path = package_dir + "config/" + config_ini_file_names[i] 
    
    data["DEPTH_INI"] = modified_file_path

    with open(config_file_name_of_tof_sdk, 'w') as file:
        json.dump(data, file, indent = 4) 

def generate_launch_description(): 
    #Arguments

    #0:ADTF3175D ToF sensor
    #2:Rosbag bin file format
    #3:ROS Topics format
    arg_input_mode_desc = DeclareLaunchArgument('arg_input_mode', default_value='0', description='Input Modes')

    #0:Disable all outputs
    #1:Enable both AVI and CSV outputs
    arg_output_mode_desc = DeclareLaunchArgument('arg_output_mode', default_value='0', description='Output Modes')

    #If 'arg_input_mode_desc' is 1 or 2 then 'arg_input_file_name_or_ros_topic_prefix_name_desc' represents the file name.
    #If 'arg_input_mode' is 3 then 'arg_input_file_name_or_ros_topic_prefix_name_desc' represents the prefix of ros topics, Default value: 'cam1'.
    #Relative path to the launch file which gets executed.
    arg_input_file_name_or_ros_topic_prefix_name_desc = DeclareLaunchArgument('arg_input_file_name_or_ros_topic_prefix_name', default_value= package_dir + "../adi_3dtof_input_video_files/adi_3dtof_height_170mm_yaw_135degrees_cam1.bin")

    
    ###### ADTF31XX Sensor related arguments ######

    #'qmp' - Sensor serial number starting with CR/DV
    #'lr-qnative' - Sensor serial number starting with AM
    arg_frame_type_desc = DeclareLaunchArgument('arg_frame_type', default_value='qmp', description='Frame Type')

    arg_ab_threshold_desc = DeclareLaunchArgument('arg_ab_threshold', default_value='10', description='AB threshold value')

    #For Sensor serial number starting with CR and DV: 10
    #For sensor serial number starting with AM : 25
    arg_confidence_threshold_desc = DeclareLaunchArgument('arg_confidence_threshold', default_value='10', description='Confidence threshold value')

    # Configuration fie name of ToF SDK
    #    "config_crosby_old_modes.json" - Sensor serial number starting with CR/DV - config_json_file_names[0]
    #    "config_crosby_adsd3500_new_modes.json" - Sensor serial number starting with AM - config_json_file_names[1]
    arg_config_file_name_of_tof_sdk_desc = DeclareLaunchArgument('arg_config_file_name_of_tof_sdk', default_value= package_dir + "config/" + config_json_file_names[0])

    # Modifying the path to ini file in json file
    if arg_input_mode_desc.default_value[0].text == '0':
        modify_ini_path_in_json_file(arg_config_file_name_of_tof_sdk_desc.default_value[0].text) 


    ###### Floor Detector related arguments ######

    #True: Activates RANSAC Floor Detector 
    #False: Activates Fallback Floor Detector 
    arg_enable_ransac_floor_detection_desc = DeclareLaunchArgument('arg_enable_ransac_floor_detection', default_value='True', description='RANSAC Floor Detector enabler')

    arg_ransac_distance_threshold_mtr_desc = DeclareLaunchArgument('arg_ransac_distance_threshold_mtr', default_value='0.025', description='Distance which determines how close the point \
                                                                   must be to the RANSAC plane in order to be selected as inlier')
    
    arg_ransac_max_iterations_desc = DeclareLaunchArgument('arg_ransac_max_iterations', default_value='10', description='Maximum number of RANSAC iterations which is allowed')

    arg_discard_distance_threshold_mtr_desc = DeclareLaunchArgument('arg_discard_distance_threshold_mtr', default_value='1.5', description='Points with depth value lesser than the given threshold are considered')


    ###### Fallback Floor Detector related arguments ######

    arg_camera_height_from_floor_in_mtr_desc = DeclareLaunchArgument('arg_camera_height_from_floor_in_mtr', default_value='0.17', description='Camera height in mtr')

    arg_enable_fallback_floor_detection = DeclareLaunchArgument('arg_enable_fallback_floor_detection', default_value='True', description='Enable Fallback Floor Detector if RANSAC Floor Detector fails \
                                                                 when "arg_enable_ransac_floor_detection_desc" is set to True')

    arg_fallback_floor_height_offset_mtr_desc = DeclareLaunchArgument('arg_fallback_floor_height_offset_mtr', default_value='0.1', description='Floor height offset for 3D Yw based floor detection')


    ###### Output topics related arguments ######

    #Currenlty, point cloud compression is not supported
    arg_enable_compression_op_image_topics_desc = DeclareLaunchArgument('arg_enable_compression_op_image_topics', default_value='False', description='Compression for Depth, IR, Floor Mask Output Topics')


    ###### TF related arguments ######

    """
        map
        ^
        |
    camera_device
        ^
        |
    camera_optical
    """

    #Prefix for the camera
    # If 'arg_input_sensor_mode' default value is 3, then value of 'arg_input_file_name_or_ros_topic_prefix_name' and 'var_ns_prefix_cam1' should not be same.
    var_ns_prefix_cam1 = "cam1"

    #Camera frame name (forward is +ve X axis)
    var_cam1_base_frame = f"{var_ns_prefix_cam1}_adtf31xx"
    
    #Camera optical frame name (forward is +ve Z axis)
    var_cam1_base_frame_optical = f"{var_ns_prefix_cam1}_adtf31xx_optical"

    #Camera's parent frame name
    var_cam1_parent_frame = "map"

    #Camera's child frame name
    var_cam1_child_frame = var_cam1_base_frame_optical

    #Optical to Device rotation, this are standard values, would never change 
    #Fixed conversion - Optical Frame: X-Right,Y-Down,Z-Front to Base Frame : X-Front,Y-Left,Z-Up conversion
    var_cam_optical_to_base_roll = "-1.57"
    var_cam_optical_to_base_pitch = "0"
    var_cam_optical_to_base_yaw = "-1.57"

    # Camera position wrt map
    #Camera position X
    var_cam1_pos_x = "0.0"

    #Camera position Y
    var_cam1_pos_y = "0.0"

    #Camera position Z
    var_cam1_pos_z = LaunchConfiguration('arg_camera_height_from_floor_in_mtr')

    #Side Tilt:
    #Left side down:-ve  Right side down: +ve
    var_cam1_roll = "0.0"
    
    #Vertical Tilt, down +ve radian, up -ve radian
    #Restriction on Pitch, Accepted Range (-1.5708, 1.5708)
    #If |Pitch| > 1.57, output from TF transform should be taken as cam_pitch = 3.14 - extracted pitch_angle
    var_cam1_pitch = "0.0"

    #Horizontal Tilt: Left: +ve, Right: -ve
    var_cam1_yaw = "0.0"

    #Nodes

    adi_3dtof_floor_detector_node_desc = Node(
                                        package='adi_3dtof_floor_detector',
                                        namespace=var_ns_prefix_cam1,
                                        executable='adi_3dtof_floor_detector_node',
                                        name='adi_3dtof_floor_detector_node',
                                        output="screen",
                                        parameters=[{
                                            'param_camera_link': var_cam1_base_frame,
                                            'param_optical_camera_link': var_cam1_base_frame_optical,
                                            'param_input_sensor_mode': LaunchConfiguration('arg_input_mode'),
                                            'param_output_sensor_mode': LaunchConfiguration('arg_output_mode'),
                                            'param_input_file_name_or_ros_topic_prefix_name': LaunchConfiguration('arg_input_file_name_or_ros_topic_prefix_name'),
                                            
                                            'param_frame_type': LaunchConfiguration('arg_frame_type'),
                                            'param_ab_threshold': LaunchConfiguration('arg_ab_threshold'),
                                            'param_confidence_threshold': LaunchConfiguration('arg_confidence_threshold'),
                                            'param_config_file_name_of_tof_sdk': LaunchConfiguration('arg_config_file_name_of_tof_sdk'),

                                            'param_enable_ransac_floor_detection': LaunchConfiguration('arg_enable_ransac_floor_detection'),
                                            'param_ransac_distance_threshold_mtr': LaunchConfiguration('arg_ransac_distance_threshold_mtr'),
                                            'param_ransac_max_iterations': LaunchConfiguration('arg_ransac_max_iterations'),
                                            'param_discard_distance_threshold_mtr': LaunchConfiguration('arg_discard_distance_threshold_mtr'),

                                            'param_enable_fallback_floor_detection': LaunchConfiguration('arg_enable_fallback_floor_detection'),
                                            'param_fallback_floor_height_offset_mtr': LaunchConfiguration('arg_fallback_floor_height_offset_mtr'),
                                            
                                            'param_enable_compression_op_image_topics': LaunchConfiguration('arg_enable_compression_op_image_topics'),
                                        }],
                                        #prefix=['valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind.rpt'],
                                        #prefix=['valgrind --tool=memcheck --track-origins=yes --log-file=valgrind.rpt'],
                                        on_exit=launch.actions.Shutdown()
                                    )
    
    cam1_base_to_optical_tf_node_desc = Node(
                                       package='tf2_ros',
                                       namespace=var_ns_prefix_cam1,
                                       executable='static_transform_publisher',
                                       name=f'{var_cam1_base_frame_optical}_tf',
                                       output="screen",
                                       arguments=["--x", "0", "--y", "0", "--z", "0", 
                                                  "--roll", f"{var_cam_optical_to_base_roll}", 
                                                  "--pitch", f"{var_cam_optical_to_base_pitch}", 
                                                  "--yaw", f"{var_cam_optical_to_base_yaw}", 
                                                  "--frame-id", f"{var_cam1_base_frame}",
                                                  "--child-frame-id", f"{var_cam1_child_frame}"]
                                   )

    cam1_map_to_base_tf_node_desc = Node(
                                   package='tf2_ros',
                                   namespace=var_ns_prefix_cam1,
                                   executable='static_transform_publisher',
                                   name=f'{var_cam1_base_frame}_tf',
                                   output="screen",
                                   arguments=["--x", f"{var_cam1_pos_x}", 
                                              "--y", f"{var_cam1_pos_y}", 
                                              "--z", var_cam1_pos_z, 
                                              "--roll", f"{var_cam1_roll}", 
                                              "--pitch", f"{var_cam1_pitch}", 
                                              "--yaw", f"{var_cam1_yaw}", 
                                              "--frame-id", f"{var_cam1_parent_frame}", 
                                              "--child-frame-id", f"{var_cam1_base_frame}"]
                               )

    #Dynammic Reconfigure

    return LaunchDescription([
        arg_input_mode_desc,
        arg_output_mode_desc,
        arg_input_file_name_or_ros_topic_prefix_name_desc, 

        arg_frame_type_desc,
        arg_ab_threshold_desc,
        arg_confidence_threshold_desc,
        arg_config_file_name_of_tof_sdk_desc,

        arg_enable_ransac_floor_detection_desc,
        arg_ransac_distance_threshold_mtr_desc,
        arg_ransac_max_iterations_desc,
        arg_discard_distance_threshold_mtr_desc,

        arg_camera_height_from_floor_in_mtr_desc,
        arg_enable_fallback_floor_detection,
        arg_fallback_floor_height_offset_mtr_desc,

        arg_enable_compression_op_image_topics_desc,

        adi_3dtof_floor_detector_node_desc,
        cam1_base_to_optical_tf_node_desc,
        cam1_map_to_base_tf_node_desc
    ])