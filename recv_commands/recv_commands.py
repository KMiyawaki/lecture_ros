#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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


def check_command(time_limit, topic='/chatter', msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    sensor_msg = SensorMessageGetter(
        topic, String, msg_wait)
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            rospy.loginfo("Recv command %s", msg.data)
            return msg.data
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time
    return None


def main():
    rospy.init_node('recv_commands')
    rospy.loginfo("C19XXX 工大 太郎")
    # Action Client
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
    while not ac.wait_for_server(rospy.Duration(5)):
        rospy.loginfo("Waiting for the move_base action server to come up")

    rospy.loginfo("The server comes up")

    while not rospy.is_shutdown():
        command = check_command(30)
        if command is None:
            continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
