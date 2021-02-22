import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import LaunchConfiguration, PythonExpression

image_topic_ = LaunchConfiguration('image_topic', default="image_raw")
camera_name = LaunchConfiguration('camera_name', default="/camera_color_frame")

image_topic = [camera_name, '/', image_topic_]
info_topic = [camera_name, "/camera_info"]
config = os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11_filter.yaml') 

def generate_launch_description():
    
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        parameters=[config],
        remappings=[
            ("/image", image_topic), 
            ("/camera_info", info_topic)]
    )
        
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
