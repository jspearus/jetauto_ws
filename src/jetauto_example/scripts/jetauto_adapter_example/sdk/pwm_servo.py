#!/usr/bin/python3
# coding=utf8
import time
import threading
import Jetson.GPIO as GPIO

mode = GPIO.getmode()
if mode == 1 or mode is None:  # 是否已经设置引脚编码
    GPIO.setmode(GPIO.BCM)  # 设为BCM编码

GPIO.setwarnings(False)  # 关闭警告打印

class PWMServo:
    def __init__(self,
                 servo = 1,
                 gpio = None, 
                 min_position=500,
                 max_position=2500,
                 min_duration=0.02,
                 max_duration=30000,
                 deviation=0):
        if gpio is None:
            if servo == 1:
                self.gpio = 13
            else:
                self.gpio = 12
        else:
            self.gpio = gpio
        self.min_position = min_position
        self.max_position = max_position
        self.min_duration = min_duration
        self.max_duration = max_duration
        self.deviation = deviation

        self.inc_times = 0
        self.pos_cur = 1500
        self.pos_set = self.pos_cur
        self.pos_inc = 0
        self.lock = threading.Lock()
        
        self.pwm = GPIO.PWM(self.gpio, 50)

    def start(self):
        self.pwm.start(7.5)
        threading.Thread(target=self.update_pos_task, daemon=True).start()

    def get_position(self):
        """
        :return:
        """
        return self.pos_cur

    def set_position(self, new_pos, duration=0):
        """
        :param new_pos:
        :param duration:
        :return:
        """
        new_pos = self.min_position if new_pos < self.min_position else new_pos
        new_pos = self.max_position if new_pos > self.max_position else new_pos
        new_pos = int(new_pos)
        duration = self.min_duration if duration < self.min_duration else duration
        duration = self.max_duration if duration > self.max_duration else duration
        inc_times = int(duration / 20 + 0.5)
        with self.lock:
            self.inc_times = inc_times
            self.pos_set = new_pos
            self.pos_inc = (self.pos_cur - new_pos) / inc_times

    def update_pos_task(self):
        while True:
            with self.lock:
                try:
                    self.inc_times -= 1
                    if self.inc_times > 0:
                        pos_cur = self.pos_set + int(self.pos_inc * self.inc_times)
                        #此处输入值为百分比值 50% 输入就是50
                        self.pwm.ChangeDutyCycle((pos_cur + self.deviation) / 200.0) # 1500 / 20000 * 100
                        self.pos_cur = pos_cur
                    elif self.inc_times == 0:
                        #此处输入值为百分比值 50% 输入就是50
                        self.pwm.ChangeDutyCycle((self.pos_set + self.deviation) / 200.0) # 1500 / 20000 * 100
                        self.pos_cur = self.pos_set
                    else:
                        self.inc_times = -1
                except Exception as e:
                    break
            time.sleep(0.02)

    def set_deviation(self, new_deviation=0):
        """
        set deviation

        :param new_deviation:
        :return:
        """
        if not -300 < new_deviation < 300:
            raise ValueError("new deviation out range. it must be betweent -300~300")
        else:
            self.deviation = int(new_deviation)

    def get_deviation(self):
        """
        :return:
        """
        return self.deviation
