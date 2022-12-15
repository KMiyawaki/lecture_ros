#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1):
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


def image_processing(cv_image_in, pub_cmd_vel):
    func = sys._getframe().f_code.co_name
    height = cv_image_in.shape[0]
    width = cv_image_in.shape[1]
    channels = cv_image_in.shape[2]
    kernel = np.ones((5, 5), np.uint8)
    rospy.loginfo("%s: Recv image (%d x %d)", func, width, height)
    cv_result = np.zeros((height, width, channels), dtype="uint8")  # 処理結果の画像
    hsv = cv2.cvtColor(cv_image_in, cv2.COLOR_BGR2HSV)  # HSV 形式に変換
    red = cv2.inRange(hsv, (0, 40, 40), (40, 255, 255))  # ここのパラメータを調整する
    red_pixels = cv2.countNonZero(red)  # 赤の画素数を数える
    # http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
    cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel, red, iterations=2)
    cv_result[red > 127] = (0, 0, 255)  # 抽出された部分を赤く塗りつぶす
    if cv2.__version__[0] == '3':
        _, contours, _ = cv2.findContours(
            red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭を抽出する
    if cv2.__version__[0] == '4':
        contours, _ = cv2.findContours(
            red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 輪郭を抽出する
    cv2.drawContours(cv_result, contours, -1, (0, 255, 0), 5)  # 輪郭を描画する
    
    for i in range(0, len(contours)):
        if len(contours[i]) > 0:
            x, y, w, h = cv2.boundingRect(contours[i]) # 輪郭を囲む矩形を得る
            size = w * h
            if size < 500: # あまりにも面積が小さいものは除外する
                continue
            cv2.rectangle(cv_result, (x, y), (x + w, y + h),
                          (255, 255, 255), 5)  # 輪郭を囲む矩形を描画する

    return (cv_result, red_pixels)  # 結果をタプルで返す


def color_track(time_limit, topic='/video_source/raw', topic_result_img='/image_processing/result_image', topic_result='/image_processing/result', topic_cmd_vel='cmd_vel' , msg_wait=1.0):
    func = sys._getframe().f_code.co_name
    rospy.loginfo('Executing ' + func)
    sensor_msg = SensorMessageGetter(topic, Image, msg_wait)
    cv_bridge = CvBridge()
    pub_result_img = rospy.Publisher(
        topic_result_img, Image, queue_size=10)
    pub_result = rospy.Publisher(
        topic_result, String, queue_size=10)
    pub_cmd_vel=rospy.Publisher(
        topic_cmd_vel, Twist, queue_size=10)
    diff = 0
    start_time = rospy.get_time()
    while diff < time_limit:
        msg = sensor_msg.get_msg()
        if msg is not None:
            rospy.loginfo('Recv msg. class = %s', msg.__class__.__name__)
            try:
                cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                (cv_result, red_pixels) = image_processing(cv_image, pub_cmd_vel)
                send = cv_bridge.cv2_to_imgmsg(cv_result, "bgr8")
                pub_result_img.publish(send)
                text = "R " + str(red_pixels)
                pub_result.publish(text)
                rospy.loginfo(text)
            except Exception as e:
                rospy.logerr("%s:%s", rospy.get_name(), str(e))
            rospy.sleep(0.05)
        diff = rospy.get_time() - start_time


def main():
    rospy.init_node('color_tracking')
    rospy.loginfo("C19XXX 工大 太郎")
    rospy.sleep(1)  # 起動直後は rospy.Time.now() がゼロを返す．
    color_track(10)


if __name__ == '__main__':
    main()
