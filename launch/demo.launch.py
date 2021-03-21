import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  # urdf_file_name = 'r2d2.urdf.xml'

  # print("urdf_file_name : {}".format(urdf_file_name))

#   urdf = os.path.join(
#       get_package_share_directory('urdf_tutorial'),
#       urdf_file_name)

  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='turtlesim',
          executable='turtlesim_node',
          name='sim',
          output='screen',
          emulate_tty=True),
      Node(
          package='turtlesim',
          executable='turtle_teleop_key',
          name='teleop',
          # output='screen',
          prefix='xterm -e'),
      Node(
          package='learning_tf2',
          executable='turtle_tf2_broadcaster',
          name='turtle1_tf2_broadcaster',
          # parameters=[{'turtle': 'turtle1'}],
          arguments=[('turtle1')],
          output='screen',
          emulate_tty=True),
      Node(
          package='learning_tf2',
          executable='turtle_tf2_broadcaster',
          name='turtle2_tf2_broadcaster',
          # parameters=[{'turtle': 'turtle2'}],
          arguments=[('turtle2')],
          output='screen',
          emulate_tty=True),
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          output='screen',
          emulate_tty=True),
  ])