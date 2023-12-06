'''ADI 3D ToF Floor Detector Example Node with RViz'''
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    #Arguments
    arg_rostopic_cam_prefix_desc = DeclareLaunchArgument('arg_rostopic_cam_prefix', default_value='cam1', description='Input ROS topic name')
    arg_enable_pointcloud_output_desc = DeclareLaunchArgument('arg_enable_pointcloud_output', default_value='True', description='Point cloud outputflag')

    adi_3dtof_floor_detector_node_desc = Node(
                                        package='adi_3dtof_floor_detector',
                                        namespace= 'host',
                                        executable='adi_3dtof_floor_detector_example_node',
                                        name='adi_3dtof_floor_detector_example_node',
                                        output="screen",
                                        parameters=[{
                                            'param_rostopic_cam_prefix': LaunchConfiguration('arg_rostopic_cam_prefix'),
                                            'param_enable_pointcloud_output': LaunchConfiguration('arg_enable_pointcloud_output')
                                        }],
                                        on_exit=launch.actions.Shutdown()
                                    )
    rviz_desc = Node(
                    package='rviz2',
                    namespace= 'host',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', [PathJoinSubstitution(([
                                        FindPackageShare('adi_3dtof_floor_detector'),
                                        'rviz',
                                        'adi_3dtof_floor_detector_example.rviz'
                                        ]))]]
                    )

    return LaunchDescription([
        arg_rostopic_cam_prefix_desc,
        arg_enable_pointcloud_output_desc,

        adi_3dtof_floor_detector_node_desc,
        rviz_desc
    ])
