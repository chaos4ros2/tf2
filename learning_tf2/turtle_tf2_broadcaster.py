#!/usr/bin/env python
import rclpy
from rclpy.node import Node

# to get commandline arguments
import sys

# because of transformations
# import tf

# static transformsを簡単に配信するにはStaticTransformBroadcasterを使う、そのためにtf2_rosモジュールをインポートする
import tf2_ros
import geometry_msgs.msg

import turtlesim.msg
# 参考 https://github.com/benbongalon/ros2-urdf-tutorial/blob/master/urdf_tutorial/urdf_tutorial/state_publisher.py
from geometry_msgs.msg import Quaternion

# エラー解消：TypeError: Expected QoSProfile or int, but received <class 'rclpy.parameter.Parameter'>
# node.create_subscriptionの3番目引数、意味不明
from rclpy.qos import QoSProfile

from math import sin, cos, pi

class StatePublisher(Node):
    def __init__(self):
        # 書き換え参考 https://docs.ros.org/en/foxy/Contributing/Migration-Guide-Python.html
        rclpy.init()
        # rospy.init_node('tf2_turtle_broadcaster')

        self.qos_profile = QoSProfile(depth=10)

        super().__init__('tf2_turtle_broadcaster')
        
        # turtlename = node.declare_parameter('~turtle')
        self.turtlename = sys.argv[1]
        # rospy.Subscriber('/%s/pose' % turtlename,
        #                  turtlesim.msg.Pose,
        #                  handle_turtle_pose,
        #                  turtlename)
        self.create_subscription(turtlesim.msg.Pose, '/%s/pose' % self.turtlename, self.handle_turtle_pose, self.qos_profile)
        rclpy.spin(self)

    def handle_turtle_pose(self, msg):
        # なぜmsgを渡せれるかは不明
        self.br = tf2_ros.TransformBroadcaster(self, qos = self.qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
    
        t = geometry_msgs.msg.TransformStamped()
        now = self.get_clock().now()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = self.turtlename
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        # カメは二次元でしか動かないので、zを0.0に
        t.transform.translation.z = 0.0
        
        # TypeError: 'Quaternion' object is not subscriptableになるので、rrclpyで回転を計算する方法を探す
        # q = euler_to_quaternion(0, 0, msg.theta)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        t.transform.rotation = euler_to_quaternion(0, 0, msg.theta) # roll,pitch,yaw
    
        self.br.sendTransform(t)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

if __name__ == '__main__':
    main()

def main():
    node = StatePublisher()