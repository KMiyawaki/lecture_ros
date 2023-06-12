#!/usr/bin/env python
# -*- coding: utf-8 -*-

import angles
import math
import sys
import roslib
import rospy
import nav_msgs.msg
import sensor_msgs.msg
import tf
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def get_localized_pose(listener, time_limit=10.0, target='map', source='base_link'):
    try:
        listener.waitForTransform(
            target, source, rospy.Time(), rospy.Duration(time_limit))
        (trans, rot) = listener.lookupTransform(
            target, source, rospy.Time(0))
        (_, _, yaw) = euler_from_quaternion(rot)
        return (trans[0], trans[1], yaw)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        raise e


def go_straight_by_distance_with_localization(listener, distance, time_limit=999, linear_vel=0.4, cmd_vel="/cmd_vel"):
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
        try:
            (x, y, yaw) = get_localized_pose(listener)
        except Exception as e:
            rospy.logerr(str(e))
            return

        rospy.loginfo(
            'Recv localized pose. (x, y, theta) = (%.2f, %.2f, %.2f)', x, y, math.degrees(yaw))

        pub.publish(vel)
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    vel.linear.x = 0.0
    pub.publish(vel)


def main():
    rospy.init_node('simple_move')
    rospy.loginfo("C19XXX Robot Taro")  # 受講者の情報を表示する。
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    listener = tf.TransformListener()  # このクラスのインスタンス生成は一度だけ。
    go_straight_by_distance_with_localization(listener, 2.0, 3.0)


if __name__ == '__main__':
    main()
