import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    ]

    # Package name
    package_name = 'learn_one'

    '''
    default_launch_dir = 'launch'
    nav2_params_path = '/home/hassan/dev_ws/learn_one/config/nav2_params.yaml'
    rviz_config_file_path = '/home/hassan/dev_ws/learn_one/config/drive_bot.rviz'
    

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_launch_dir = os.path.join(pkg_share, default_launch_dir)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    nav2_params_path = os.path.join(pkg_share, nav2_params_path)
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    '''

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
                        arguments=['-topic', 'robot_description', '-entity', 'weedx', '-x', spawn_x_val,
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

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    ekf_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory(package_name), 
                    'launch', 'ekf_gazebo.launch.py')
    ])
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'amcl_params.yaml'),
                    {'use_sim_time': True}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['amcl']}]
    )

    '''
    declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
        
    declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')

    declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')

    declare_params_file_cmd = DeclareLaunchArgument(
    name='params_file',
    default_value=nav2_params_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

    

    declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

    start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

    start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments = {'namespace': namespace,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'autostart': autostart}.items())
    '''



#    covariance_splitter_node = Node(
#        package='learn_one',
#        executable='covariance_splitter.py',  # MUST include .py extension
#        name='covariance_splitter',
#        output='screen',
#        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
#    )
    ld = LaunchDescription()
    #ld.add_action(declare_slam_cmd)
    #ld.add_action(declare_rviz_config_file_cmd)
    #ld.add_action(declare_params_file_cmd)
    #ld.add_action(declare_use_rviz_cmd)
    #ld.add_action(declare_namespace_cmd)
    #ld.add_action(declare_use_namespace_cmd)
    #ld.add_action(declare_autostart_cmd)

    #ld.add_action(start_rviz_cmd)
    #ld.add_action(start_ros2_navigation_cmd)

    # Launch everything
    return LaunchDescription(declared_arguments + [
        rsp,
        gazebo,
        twist_mux,
        static_tf,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ld,
        ekf_launch
        #covariance_splitter_node
    ])

    
