#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import roslib
import rospy
from geometry_msgs.msg import Quaternion, Twist


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


def main():
    rospy.init_node('simple_move')
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    go_straight_by_time(2.0)


if __name__ == '__main__':
    main()
