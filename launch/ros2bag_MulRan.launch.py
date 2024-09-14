import os
import sys
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():        
    data_folder_path = LaunchConfiguration('data_folder_path', default='~/Documents')
    sequence_name = LaunchConfiguration('sequence_name', default='KAIST01')
    dst_path = LaunchConfiguration('dst_path', default='~/Documents/mulran')

    declare_data_folder_path_arg = DeclareLaunchArgument(
        'data_folder_path',
        default_value=data_folder_path,
        description='Path to MULRAN data folder'
    )
    
    declare_sequence_name_arg = DeclareLaunchArgument(
        'sequence_name',
        default_value=sequence_name,
        description='Sequence name'
    )    

    declare_dst_path_arg = DeclareLaunchArgument(
        'dst_path',
        default_value=dst_path,
        description='Destination path for saving bag file'
    )    

    params = [
        {
            'data_folder_path': data_folder_path,
            'sequence_name': sequence_name,
            'dst_path': dst_path,
            'if_write_imu': True,
            'if_write_lidar': True,
            'if_write_gps': True,
            'if_write_radar': False,
            'if_write_gt': True
        }
    ]

    player_node = launch_ros.actions.Node(
        package='ros2bag_mulran',
        executable='bagWriter',
        name='bagWriter',
        output='screen',
        parameters=params
    )

    return launch.LaunchDescription([    
        declare_data_folder_path_arg,  
        declare_sequence_name_arg,    
        declare_dst_path_arg,	         
        player_node
  ])
