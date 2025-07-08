
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    multi_robots_avoidance_log_level = LaunchConfiguration('multi_robots_avoidance_log_level', default='info')

    # get pkg path
    multi_robots_pkg_path = get_package_share_directory('nav2_multi_robots_avoidance')    

    # get param file path
    params_file_path = os.path.join(multi_robots_pkg_path, 'param', 'config.yaml')   

    priority = 1
    try:
        if 'ROBOT_PRIORITY' in os.environ:
            priority = int(os.environ.get('ROBOT_PRIORITY'))
            print(f'get ROBOT_PRIORITY value {priority} from os.environment.')
        else:
            print('using default ROBOT_PRIORITY value 1')
            priority = 1
    except:
        print("Please declare ROBOT_PRIORITY!")
        priority = 1

    dummy_namespace_name = "mk"
    try:
        if 'CAPELLA_ROS_NAMESPACE' in os.environ:
            dummy_namespace_name = os.environ.get('CAPELLA_ROS_NAMESPACE')
            print(f'get CAPELLA_ROS_NAMESPACE value {dummy_namespace_name} from os.environment.')
        else:
            print('using default CAPELLA_ROS_NAMESPACE value mk')
            dummy_namespace_name = 'mk'
    except:
        print("Please declare CAPELLA_ROS_NAMESPACE!")
        dummy_namespace_name = 'mk'

    collision_radius_check_threshold = 10.0
    try:
        if 'COLLISION_RADIUS_CHECK_THRESHOLD' in os.environ:
            collision_radius_check_threshold = os.environ.get('COLLISION_RADIUS_CHECK_THRESHOLD')
            print(f'get COLLISION_RADIUS_CHECK_THRESHOLD value {collision_radius_check_threshold} from os.environment.')
        else:
            print('using default COLLISION_RADIUS_CHECK_THRESHOLD value 10.0')
            collision_radius_check_threshold = 10.0
    except:
        print("Please declare COLLISION_RADIUS_CHECK_THRESHOLD!")
        collision_radius_check_threshold = 10.0

    global_pose_filter_threshold = 2.82
    try:
        if 'GLOBAL_POSE_FILTER_THRESHOLD' in os.environ:
            global_pose_filter_threshold = os.environ.get('GLOBAL_POSE_FILTER_THRESHOLD')
            print(f'get GLOBAL_POSE_FILTER_THRESHOLD value {global_pose_filter_threshold} from os.environment.')
        else:
            print('using default GLOBAL_POSE_FILTER_THRESHOLD value 2.82')
            global_pose_filter_threshold = 2.82
    except:
        print("Please declare GLOBAL_POSE_FILTER_THRESHOLD!")
        global_pose_filter_threshold = 2.82

    # multi_robots_avoidance Node
    nav2_multi_robots_avoidance_node = Node(
        executable='multi_robots_avoidance',
        package='nav2_multi_robots_avoidance',
        name='multi_robots_avoidance',
        output='screen',
        parameters=[params_file_path, {"use_sim_time": False, "priority": priority, 
                                       'dummy_namespace_name': dummy_namespace_name, 
                                       'collision_radius_check_threshold': collision_radius_check_threshold, 
                                       'global_pose_filter_threshold': global_pose_filter_threshold}
                    ],
        arguments=['--ros-args', '--log-level', ['multi_robots_avoidance:=', multi_robots_avoidance_log_level]],
        respawn=True,
    )

    launch_description.add_action(nav2_multi_robots_avoidance_node)

    return launch_description





