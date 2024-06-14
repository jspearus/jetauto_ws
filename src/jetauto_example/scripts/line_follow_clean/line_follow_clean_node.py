#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 巡线清障
import os
import sys
import cv2
import math
import rospy
import signal
import threading
import numpy as np
import jetauto_sdk.pid as pid
import jetauto_sdk.misc as misc
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from jetauto_interfaces.msg import ColorsInfo, ColorDetect, LineROI, ROI
from jetauto_interfaces.srv import SetColorDetectParam, SetCircleROI, SetLineROI
sys.path.append('/home/jetauto/jetauto_software/jetauto_arm_pc')
import action_group_controller as controller

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree

class LineFollowCleanNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.running = True
        self.image = None
        self.count = 0
        self.count_stop = 0
        self.stop = False
        self.line_color = 'black'
        self.line_x = None
        self.temp_line_x = None
        self.object_blue = 'blue' 
        self.object_red = 'red'
        self.object_green = 'green'
        self.center = None
        self.temp_center = None
        self.stop_threshold = 0.4
        self.scan_angle = math.radians(90)  # radians
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.pid_x = pid.PID(0.001, 0.0, 0.0)
        #self.pick_roi = [235, 265, 305, 335]  #[y_min, y_max, x_min, x_max]
        self.pick_roi = [284, 314, 320, 350]  #[y_min, y_max, x_min, x_max]
        self.start_pick = False
        
        signal.signal(signal.SIGINT, self.shutdown)
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)  # 订阅雷达
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)  # 底盘控制
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        rospy.Subscriber('/color_detect/image_result/', Image, self.image_callback)
        
        controller.runAction('line_follow_init')
        
        self.debug = rospy.get_param('~debug', False)
        if self.debug:
            controller.runAction('move_object_debug')
            rospy.sleep(5)
            controller.runAction('line_follow_init')
        threading.Thread(target=self.pick, daemon=True).start()
        
        self.mecanum_pub.publish(Twist())

        line_roi = LineROI()
        line_roi.roi_up.x_min = 0
        line_roi.roi_up.x_max = 640
        line_roi.roi_up.y_min = 220
        line_roi.roi_up.y_max = 230
        line_roi.roi_up.scale = 0.0

        line_roi.roi_center.x_min = 0  
        line_roi.roi_center.x_max = 640
        line_roi.roi_center.y_min = 280
        line_roi.roi_center.y_max = 290
        line_roi.roi_center.scale = 0.1

        line_roi.roi_down.x_min = 0
        line_roi.roi_down.x_max = 640
        line_roi.roi_down.y_min = 340
        line_roi.roi_down.y_max = 350
        line_roi.roi_down.scale = 0.9

        res = rospy.ServiceProxy('/color_detect/set_line_roi', SetLineROI)(line_roi)
        if res.success:
            print('set roi success')
        else:
            print('set roi fail')
        
        object_roi = ROI()
        object_roi.x_min = 0
        object_roi.x_max = 640
        object_roi.y_min = 0
        object_roi.y_max = 400
        
        res = rospy.ServiceProxy('/color_detect/set_circle_roi', SetCircleROI)(object_roi)
        if res.success:
            print('set roi success')
        else:
            print('set roi fail')

        msg_black = ColorDetect()
        msg_black.color_name = self.line_color
        msg_black.detect_type = 'line'
        msg_blue = ColorDetect()
        msg_blue.color_name = self.object_blue
        msg_blue.detect_type = 'circle'
        msg_red = ColorDetect()
        msg_red.color_name = self.object_red
        msg_red.detect_type = 'circle'
        msg_green = ColorDetect()
        msg_green.color_name = self.object_green
        msg_green.detect_type = 'circle'
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_black, msg_blue, msg_red, msg_green])
        if res.success:
            print('set color success')
        else:
            print('set color fail')

        self.line_follow() 
 
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_color_callback(self, msg):
        line_x = None
        center = None
        for i in msg.data:
            if i.color == self.line_color:
                line_x = i.x
            elif i.color == self.object_blue or i.color == self.object_red or i.color == self.object_green:
                center = i
        self.temp_line_x = line_x
        self.temp_center = center

    def pick(self):
        while self.running:
            if self.start_pick:
                self.mecanum_pub.publish(Twist())
                rospy.sleep(0.5)
                controller.runAction('move_object')
                self.start_pick = False
            else:
                rospy.sleep(0.01)

    def line_follow(self):
        while self.running:
            self.line_x = self.temp_line_x
            self.center = self.temp_center
            if self.line_x is not None and not self.start_pick:
                twist = Twist()
                if self.center is not None:
                    if self.center.y > 100 and abs(self.center.x - self.line_x) < 100 and not self.debug:
                        self.pid_x.SetPoint = (self.pick_roi[1] + self.pick_roi[0])/2
                        self.pid_x.update(self.center.y)
                        self.pid.SetPoint = 320
                        self.pid.update(self.center.x)
                        twist.linear.x = misc.set_range(self.pid_x.output, -0.08, 0.08)
                        twist.angular.z = misc.set_range(self.pid.output, -0.5, 0.5)
                        if abs(twist.linear.x) <= 0.0065:
                            self.count += 1
                            rospy.sleep(0.01)
                            if self.count > 50:
                                self.count = 0
                                self.start_pick = True
                        else:
                            self.count = 0
                    elif self.debug:
                        print(self.center.y - 15, self.center.y + 15, self.center.x - 15, self.center.x + 15)
                        cv2.rectangle(self.image, (self.center.x - 25, self.center.y - 25,), (self.center.x + 25, self.center.y + 25), (0, 0, 255), 2)
                    else:
                        self.pid.SetPoint = 320
                        self.pid.update(self.line_x)
                        twist.linear.x = 0.08
                        twist.angular.z = misc.set_range(self.pid.output, -0.8, 0.8)
                elif not self.debug:
                    self.pid.SetPoint = 320
                    self.pid.update(self.line_x)
                    twist.linear.x = 0.15
                    twist.angular.z = misc.set_range(self.pid.output, -0.8, 0.8)
                if not self.stop:
                    self.mecanum_pub.publish(twist)
                else:
                    self.mecanum_pub.publish(Twist())
            else:
                self.mecanum_pub.publish(Twist())
                rospy.sleep(0.01)
            if self.image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(self.image, (self.pick_roi[2] - 30, self.pick_roi[0] - 30), (self.pick_roi[3] + 30, self.pick_roi[1] + 30), (0, 255, 255), 2)
                cv2.imshow('image', self.image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.running = False
        self.mecanum_pub.publish(Twist())
        controller.runAction('line_follow_init')
        rospy.signal_shutdown('shutdown')

    def image_callback(self, ros_image):
        self.image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面

    def lidar_callback(self, lidar_data):
        # 数据大小 = 扫描角度/每扫描一次增加的角度
        if self.lidar_type in ['A1', 'A2']:
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1] # 左半边数据
            right_ranges = lidar_data.ranges[min_index:max_index][::-1] # 右半边数据

        # 根据设定取数据
        angle = self.scan_angle / 2
        angle_index = int(angle / lidar_data.angle_increment + 0.50)
        left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        # 取左右最近的距离
        min_dist_left = left_range[left_nonzero].min()
        min_dist_right = right_range[right_nonzero].min()
        if min_dist_left < self.stop_threshold or min_dist_right < self.stop_threshold: 
            self.stop = True
        else:
            self.count_stop += 1
            if self.count_stop > 5:
                self.count_stop = 0
                self.stop = False

if __name__ == "__main__":
    LineFollowCleanNode('line_follow_clean')
