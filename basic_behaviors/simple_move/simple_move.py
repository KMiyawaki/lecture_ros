#!/usr/bin/env python
# -*- coding: utf-8 -*-

import angles
import math
import sys
import roslib
import rospy
import nav_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def get_yaw(odom):
    q = (odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z,
         odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(q)
    return euler[2]


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1.0):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message


def go_straight_by_time(time_limit, linear_vel=0.4, cmd_vel="/cmd_vel"):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = linear_vel
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        pub.publish(vel)
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    pub.publish(vel)


def go_straight_by_distance(distance, time_limit=999, linear_vel=0.4, topic='/odom', cmd_vel="/cmd_vel", msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    sensor_msg = SensorMessageGetter(
        topic, nav_msgs.msg.Odometry, msg_wait)
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    vel = Twist()
    vel.linear.x = linear_vel
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = get_yaw(msg)
            rospy.loginfo(
                'Recv odometry. (x, y, theta) = (%.2f, %.2f, %.2f)', x, y, math.degrees(yaw))
            # http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html をよく見よう。
            # msg.pose.pose.position.x などで座標が分かる。
        pub.publish(vel)
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    pub.publish(vel)


def main():
    rospy.init_node('simple_move')
    rospy.loginfo("C19XXX Robot Taro")  # 受講者の情報を表示する。
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    go_straight_by_time(2.0)


if __name__ == '__main__':
    main()
