#!/usr/bin/env python3



# import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource



# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
   

    
    # include xml launch file
    #######################################################################
    launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_bringup'),
                'launch', 'my_robot_gazebo.launch.xml')
        )
    )
    ########################################################################
    # urdf_path = os.path.join(get_package_share_path('my_robot_description'),
    #                          'urdf', 'my_robot.urdf.xacro')
    
    # robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{'robot_description': robot_description}]
    # )

    # world = os.path.join(
    #     get_package_share_directory('my_robot_bringup'),
    #     'worlds',
    #     'class_room.world'
    # )

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            
    #     ),
    #     launch_arguments={"-topic robot_description -entity my_robot" }
    # )

   
    # gazebo_ros = Node(
    #     package="robot_state_publisher",
    #     executable="spawn_entity.py",
    #     arguments=['-topic robot_description -entity my_robot']
    # )

    # urdf_path = os.path.join(get_package_share_path('my_robot_description'),
    #                          'urdf', 'my_robot.urdf.xacro')
    
    # robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{'robot_description': robot_description}]
    # )
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    # included_launch = launch_ros.actions.IncludeLaunchDescription(package='my_robot_bringup', launch='my_robot_gazebo.launch.xml')  
    # regular_node = launch_ros.actions.Node( package='bar', node_executable='my_node', output='screen')


    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('my_robot_bringup'),
            'map',
            'classroom.yaml'))

    # param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('my_robot_bringup'),
            'param',
            'fork.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'rviz',
        'urdf_config.rviz')

    return LaunchDescription([
        
        # joint_state_publisher_gui_node,
        # included_launch,
        # regular_node,
        # gzserver_cmd,
        # gzclient_cmd,
        # robot_state_publisher_node,
        
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        launch_include
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
            
        #     arguments=['-topic robot_description -entity my_robot'])
            
    ])