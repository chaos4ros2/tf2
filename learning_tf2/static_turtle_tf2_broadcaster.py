#!/usr/bin/env python
# import rospy
import rclpy
from rclpy.node import Node

# to get commandline arguments
import sys

# because of transformations
# import tf
# from tf_conversions.transformations import quaternion_from_euler

# static transformsを簡単に配信するにはStaticTransformBroadcasterを使う、そのためにtf2_rosモジュールをインポートする
import tf2_ros
import geometry_msgs.msg

# 参考 https://github.com/benbongalon/ros2-urdf-tutorial/blob/master/urdf_tutorial/urdf_tutorial/state_publisher.py
from geometry_msgs.msg import Quaternion

from math import sin, cos, pi

class StatePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('my_static_tf2_broadcaster')

        if len(sys.argv) < 8:
            self.get_logger().info('Invalid number of parameters\nusage: '
                         './static_turtle_tf2_broadcaster.py '
                         'child_frame_name x y z roll pitch yaw')
            sys.exit(0)
        else:
            if sys.argv[1] == 'world':
                self.get_logger().info('Your static turtle name cannot be "world"')
                sys.exit(0)

            # StaticTransformBroadcasterオブジェクト(インスタンス)の作成、transformationsを送るため（over the wireの意味はわからない、有線？）
            # tf2_node = rclpy.create_node('my_static_tf2_broadcaster')
            self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

            # TransformStampedオブジェクトはメッセージの格納用
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            # 値を詰め込む前にmetaデータを代入する
            now = self.get_clock().now()
            static_transformStamped.header.stamp = now.to_msg()
            static_transformStamped.header.frame_id = "world"
            static_transformStamped.child_frame_id = sys.argv[1]

            # 6D pose(translation and rotation)を代入する 
            static_transformStamped.transform.translation.x = float(sys.argv[2])
            static_transformStamped.transform.translation.y = float(sys.argv[3])
            static_transformStamped.transform.translation.z = float(sys.argv[4])

            quat = euler_to_quaternion(float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
            static_transformStamped.transform.rotation.x = quat[0]
            static_transformStamped.transform.rotation.y = quat[1]
            static_transformStamped.transform.rotation.z = quat[2]
            static_transformStamped.transform.rotation.w = quat[3]

            # StaticTransformBroadcasterのsendTransform()関数でtransformを送信する
            self.broadcaster.sendTransform(static_transformStamped)

            rclpy.spin(self)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
   main()