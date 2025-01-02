from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # publisher
  fake_realsense_node = Node(
    package='foundation_pose',
    executable='fake_realsense',
    name = "fake_realsense_node"
  )
  # subscriber
  foundation_pose_node = Node(
    package='foundation_pose',
    executable='foundation_pose',
    name = "foundation_pose_node"
  )

  return LaunchDescription([
    fake_realsense_node,
    foundation_pose_node
  ])

