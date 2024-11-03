
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    multi_robots_pkg_path = get_package_share_directory('nav2_multi_robots_avoidance')    

    # get param file path
    params_file_path = os.path.join(multi_robots_pkg_path, 'param', 'config.yaml')   

    
    namespace_name = LaunchConfiguration("namespace_name")
    namespace_name = "mk8"


    # multi_robots_avoidance Node
    nav2_multi_robots_avoidance_node = Node(
        executable='multi_robots_avoidance',
        package='nav2_multi_robots_avoidance',
        name='multi_robots_avoidance',
        namespace=namespace_name,
        output='screen',
        parameters=[params_file_path],
        remappings=[("/tf", "/robot1/tf"),
                    ("/tf_static", "/robot1/tf_static")],
    )

    launch_description.add_action(nav2_multi_robots_avoidance_node)

    return launch_description





