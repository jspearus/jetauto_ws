#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/17
# @author:aiden
# 颜色检测
import cv2
import math
import rospy
import signal
import numpy as np
import jetauto_sdk.fps as fps
from sensor_msgs.msg import Image
import jetauto_sdk.common as common
from jetauto_interfaces.msg import ColorInfo, ColorsInfo
from jetauto_interfaces.srv import SetColorDetectParam, SetCircleROI, SetLineROI

class ColorDetectNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.running = True
        self.detect_type = {}
        self.target_colors = None
        self.fps = fps.FPS()  # fps计算器
        self.lab_data = common.get_yaml_data("/home/jetauto/jetauto_software/lab_tool/lab_config.yaml")
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.line_roi = rospy.get_param('~roi_line')
        self.circle_roi = rospy.get_param('~roi_circle')
        self.rect_roi = rospy.get_param('~roi_rect')
        self.weight_sum = 1.0
        use_depth_cam = rospy.get_param('~use_depth_cam', True)
        if use_depth_cam:
            self.camera_type = 'Stereo'
            camera = rospy.get_param('/depth_camera/camera_name', 'camera')  # 获取参数
            self.camera_sub = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback)  # 摄像头订阅
        else:
            self.camera_type = 'Mono'
            camera = rospy.get_param('/usb_cam_name', 'usb_cam')  # 获取参数
            self.camera_sub = rospy.Subscriber('/%s/image_raw'%camera, Image, self.image_callback)  # 摄像头订阅
        self.info_publisher = rospy.Publisher(self.name + '/color_info', ColorsInfo, queue_size=1)
        self.result_publisher = rospy.Publisher(self.name + '/image_result', Image, queue_size=1)  # 图像处理结果发布
        rospy.Service(self.name + '/set_param', SetColorDetectParam, self.set_param_srv_callback)
        rospy.Service(self.name + '/set_line_roi', SetLineROI, self.set_line_roi_srv)
        rospy.Service(self.name + '/set_circle_roi', SetCircleROI, self.set_circle_roi_srv)
        rospy.Service(self.name + '/set_rect_roi', SetCircleROI, self.set_rect_roi_srv)
        while True:
            if self.image is not None:
                rospy.set_param(self.name + '/start', True)
                break
            else:
                rospy.sleep(0.01)
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False

    def set_circle_roi_srv(self, msg):
        self.circle_roi['x_min'] = msg.data.x_min
        self.circle_roi['x_max'] = msg.data.x_max
        self.circle_roi['y_min'] = msg.data.y_min
        self.circle_roi['y_max'] = msg.data.y_max

        return [True, 'set_circle_roi']

    def set_rect_roi_srv(self, msg):
        self.rect_roi['x_min'] = msg.data.x_min
        self.rect_roi['x_max'] = msg.data.x_max
        self.rect_roi['y_min'] = msg.data.y_min
        self.rect_roi['y_max'] = msg.data.y_max

        return [True, 'set_rect_roi']

    def set_line_roi_srv(self, msg):
        roi_up = msg.data.roi_up
        roi_center = msg.data.roi_center
        roi_down = msg.data.roi_down
        self.line_roi['roi_up'] = [roi_up.y_min, roi_up.y_max, roi_up.x_min, roi_up.x_max, roi_up.scale]
        self.line_roi['roi_center'] = [roi_center.y_min, roi_center.y_max, roi_center.x_min, roi_center.x_max,roi_center.scale]
        self.line_roi['roi_down'] = [roi_down.y_min, roi_down.y_max, roi_down.x_min, roi_down.x_max, roi_down.scale]

        return [True, 'set_line_roi']

    def set_param_srv_callback(self, msg):
        if len(msg.data) == 1:
            self.target_colors = [msg.data[0].color_name, ]
            self.detect_type[msg.data[0].color_name] = msg.data[0].detect_type
        else:
            self.target_colors = []
            for i in msg.data:
                self.target_colors.append(i.color_name)
                self.detect_type[i.color_name] = i.detect_type
        return [True, 'set_param']

    def image_proc(self):
        while self.running:
            if self.image is not None and self.target_colors is not None:
                self.result_image = self.image.copy()
                h, w = self.image.shape[:2]
                img_lab = cv2.cvtColor(self.image, cv2.COLOR_BGR2LAB)  # bgr转lab
                img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪
                x = 0
                y = 0
                radius = 0
                centroid_sum = 0
                line_color = None
               
                max_area_rect = 0
                color_area_max_rect = ''    
                areaMaxContour_rect = None

                max_area_circle = 0
                color_area_max_circle = ''    
                areaMaxContour_circle = None
                for i in self.target_colors:
                    if self.detect_type[i] == 'line' and i != '':
                        line_color = i
                        for roi in self.line_roi:
                            roi_value = self.line_roi[roi]
                            blob = img_blur[roi_value[0]:roi_value[1], roi_value[2]:roi_value[3]]  # 截取roi
                            mask = cv2.inRange(blob, tuple(self.lab_data['lab'][self.camera_type][i]['min']), tuple(self.lab_data['lab'][self.camera_type][i]['max']))  # 二值化
                            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
                            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
                            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓
                            max_contour_area = common.get_area_max_contour(contours, 0)[0]  # 获取最大面积对应轮廓
                            if max_contour_area is not None:
                                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形
                                box = np.int0(cv2.boxPoints(rect))  # 四个角
                                for j in range(4):
                                    box[j, 1] = box[j, 1] + roi_value[0]
                                cv2.drawContours(self.result_image, [box], -1, common.range_rgb[i], 2)  # 画出四个点组成的矩形

                                # 获取矩形对角点
                                pt1_x, pt1_y = box[0, 0], box[0, 1]
                                pt3_x, pt3_y = box[2, 0], box[2, 1]
                                # 线的中心点
                                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                                cv2.circle(self.result_image, (int(line_center_x), int(line_center_y)), 5, common.range_rgb[i], -1)  # 画出中心点
                                centroid_sum += line_center_x * roi_value[-1]
                    elif i != '':
                        if self.detect_type[i] == 'rect':
                            blob = img_blur[self.rect_roi['y_min']:self.rect_roi['y_max'], self.rect_roi['x_min']:self.rect_roi['x_max']]
                        else:
                            blob = img_blur[self.circle_roi['y_min']:self.circle_roi['y_max'], self.circle_roi['x_min']:self.circle_roi['x_max']]
                        mask = cv2.inRange(blob, tuple(self.lab_data['lab'][self.camera_type][i]['min']), tuple(self.lab_data['lab'][self.camera_type][i]['max']))  # 二值化
                        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
                        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
                        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓
                        max_contour_area = common.get_area_max_contour(contours, 0)[0]  # 获取最大面积对应轮廓
                        if max_contour_area is not None:
                            area = math.fabs(cv2.contourArea(max_contour_area))
                            if self.detect_type[i] == 'rect':
                                if area > max_area_rect:  # 找最大面积
                                    max_area_rect = area
                                    color_area_max_rect = i
                                    areaMaxContour_rect = max_contour_area
                            else:
                                if area > max_area_circle:  # 找最大面积
                                    max_area_circle = area
                                    color_area_max_circle = i
                                    areaMaxContour_circle = max_contour_area
                colors_info = ColorsInfo()
                color_info_list = []
                center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点
                if line_color is not None and int(center_pos) != 0:
                    color_info = ColorInfo()
                    color_info.width = w
                    color_info.height = h
                    color_info.color = line_color
                    color_info.x = int(center_pos)
                    color_info.y = h - 50 
                    color_info_list.append(color_info)
                if areaMaxContour_rect is not None:
                    rect = cv2.minAreaRect(areaMaxContour_rect)  # 最小外接矩形
                    box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                    for i in range(4):
                        box[i, 0] = box[i, 0] + self.rect_roi['x_min']
                        box[i, 1] = box[i, 1] + self.rect_roi['y_min']
                    cv2.drawContours(self.result_image, [box], -1, common.range_rgb[color_area_max_rect], 2)  # 画出四个点组成的矩形

                    # 获取矩形对角点
                    pt1_x, pt1_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    # 中心点
                    x, y = int((pt1_x + pt3_x) / 2), int((pt1_y + pt3_y) / 2)
                    
                    color_info = ColorInfo()
                    color_info.width = w
                    color_info.height = h
                    color_info.color = color_area_max_rect
                    color_info.x = x
                    color_info.y = y
                    color_info.angle = int(rect[2])
                    color_info_list.append(color_info)
                if areaMaxContour_circle is not None:
                    ((x, y), radius) = cv2.minEnclosingCircle(areaMaxContour_circle)  # 获取最小外接圆
                    x = int(x) + self.circle_roi['x_min']
                    y = int(y) + self.circle_roi['y_min']
                    radius = int(radius)            
                    cv2.circle(self.result_image, (x, y), radius, common.range_rgb[color_area_max_circle], 2)  # 画圆
                    
                    color_info = ColorInfo()
                    color_info.width = w
                    color_info.height = h
                    color_info.color = color_area_max_circle
                    color_info.x = x
                    color_info.y = y
                    color_info.radius = radius
                    color_info_list.append(color_info)
                colors_info.data = color_info_list
                self.info_publisher.publish(colors_info)
                self.fps.update()
                result_image = self.fps.show_fps(self.result_image)
                self.result_publisher.publish(common.cv2_image2ros(result_image, self.name))
            else:
                rospy.sleep(0.01)

        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

if __name__ == "__main__":
    ColorDetectNode('color_detect')
