#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import rosnode
import sensor_msgs.msg


def check_ros_node(target="/stageros"):
    # import rosnode が必要
    nodes = rosnode.get_node_names()
    if target in nodes:
        return True
    return False


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


def check_laser(time_limit, topic='/base_scan', msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    # シミュレーションかどうかをチェック
    is_simulation = check_ros_node()
    rospy.loginfo("is_simulation = " + str(is_simulation))
    sensor_msg = SensorMessageGetter(
        topic, sensor_msgs.msg.LaserScan, msg_wait)
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            rospy.loginfo('Recv msg. class = %s', msg.__class__.__name__)
            # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html をよく見よう。
            # len(msg.ranges) がスキャンデータの個数
            # msg.ranges[i] で i 番目のセンサデータが分かる。
        rospy.sleep(0.1)
        diff = rospy.get_time() - start_time


def main():
    rospy.init_node('check_laser')
    rospy.loginfo("C19XXX 工大 太郎")
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    check_laser(3)


if __name__ == '__main__':
    main()
