import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    ]

    # Package name
    package_name = 'learn_one'
    
    #nav2_params = os.path.join(
    #get_package_share_directory(package_name),
    #'config',
    #'nav2_params.yaml'
    #)
    

    spawn_x_val = '-6.5'
    spawn_y_val = '-4.2'
    spawn_z_val = '0.3'
    spawn_yaw_val = '0.0'

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # Gazebo launch file
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Spawn entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'weedx' ,
                                   '-x', spawn_x_val,
                                    '-y', spawn_y_val,
                                    '-z', spawn_z_val,
                                    '-Y', spawn_yaw_val],
                                    output='screen')

    # Spawner nodes
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )
    '''
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom/filtered', 'base_footprint']
    )'''

    ekf_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory(package_name), 
                    'launch', 'ekf_gazebo.launch.py')
    ])
    )

    '''
    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    ]),
    launch_arguments={
        'use_sim_time': 'true',
        'params_file': nav2_params,
    }.items()
    )'''

#    covariance_splitter_node = Node(
#        package='learn_one',
#        executable='covariance_splitter.py',  # MUST include .py extension
#        name='covariance_splitter',
#        output='screen',
#        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
#    )


    # Launch everything
    return LaunchDescription(declared_arguments + [
        rsp,
        gazebo,
        twist_mux,
        #static_tf,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        #nav2_launch,
        ekf_launch
        #covariance_splitter_node
    ])
