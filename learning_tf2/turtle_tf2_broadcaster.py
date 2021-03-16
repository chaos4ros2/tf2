#!/usr/bin/env python
import rclpy

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

def handle_turtle_pose(node, msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = node.get_clock().now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = euler_to_quaternion(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    # 書き換え参考 https://docs.ros.org/en/foxy/Contributing/Migration-Guide-Python.html
    rclpy.init(args=sys.argv)
    # rospy.init_node('tf2_turtle_broadcaster')

    qos_profile = QoSProfile(depth=10)

    node = rclpy.create_node('tf2_turtle_broadcaster')
    turtlename = node.declare_parameter('~turtle')
    # rospy.Subscriber('/%s/pose' % turtlename,
    #                  turtlesim.msg.Pose,
    #                  handle_turtle_pose,
    #                  turtlename)
    node.create_subscription(turtlesim.msg.Pose, '/%s/pose' % turtlename, qos_profile, handle_turtle_pose, turtlename)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
