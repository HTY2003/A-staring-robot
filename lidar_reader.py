#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
from scipy.interpolate import *
import time

max_range = 8
img_size = 601
times = np.zeros((3), dtype=np.uint64)
buffer = np.zeros((3,1440), dtype=np.float32)
angles_smaller = np.deg2rad(np.arange(180, 540, 0.5))
angles_smoothed = np.concatenate((np.deg2rad(np.arange(180,360,0.5)), np.deg2rad(np.arange(0,180,0.5))))
angles = np.deg2rad(np.arange(0,720,0.5))
angles2 = np.deg2rad(np.arange(0,360,0.5))
prev_ranges_smoothed = [[None]]

class LaserScanReader(object):
    def __init__(self):
        self.value = 0
        self.new = False
        rospy.init_node('pc')
        self.pub = rospy.Publisher('/pcout', Int32MultiArray, queue_size = 10)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        
    def callback(self, data):
        debug = True

        if debug:
            cv2.namedWindow("debug", cv2.WINDOW_AUTOSIZE)

        # rospy.loginfo(rospy.get_caller_id() + " %s", data.header.stamp.nsecs)

        # constrain and scale
        img = np.zeros((img_size+50, img_size+50, 3), dtype=np.uint8)
        ranges = np.array(data.ranges)
        ranges = np.concatenate((ranges, ranges))
        ranges = np.clip(ranges, 0, max_range)
        ranges = np.divide(np.multiply(ranges, img_size // 2 - 25), max_range)

        # rolling average
        buffer[0] = buffer[1]
        buffer[1] = buffer[2]
        buffer[2] = ranges
        ranges = np.mean(buffer, axis = 0)

        times[0] = times[1]
        times[1] = times[2]
        times[2] = data.header.stamp.nsecs

        # plot raw data
        if debug:
            x = np.add(np.int16(np.multiply(ranges, np.cos(angles))), img_size // 2)
            y = np.add(np.int16(np.multiply(ranges, np.sin(angles))), img_size // 2)
            img[y[:], x[:], 0] = 255

        # plot interpolated data
        spl = make_smoothing_spline(angles, ranges, lam = 0.001)
        ranges_smoothed = spl(angles_smaller) #take 180deg to 540deg
        
        if debug:
            x_smoothed = np.add(np.int16(np.multiply(ranges_smoothed, np.cos(angles_smoothed))), img_size // 2)
            y_smoothed = np.add(np.int16(np.multiply(ranges_smoothed, np.sin(angles_smoothed))), img_size // 2)
            img[y_smoothed[:], x_smoothed[:], 1] = 255

        # do math
        if times[2] < times[1]:
            delta_t = int(1000000000 + times[2] - times[1])
        else:
            delta_t = times[2] - times[1]

        dr_dth = np.zeros((720), dtype = np.float32)
        dr_dt = np.zeros((720), dtype = np.float32)
        dth_dt = np.zeros((720), dtype = np.float32)

        for i in range(0, 720):
            dr_dth[i] = ranges_smoothed[(i+1)%720] - ranges_smoothed[i-1]
            if prev_ranges_smoothed[0][0] != None:
                dr_dt[i] = (ranges_smoothed[i] - prev_ranges_smoothed[0][i]) / delta_t * 1000000000

        for i in range(0,720):
            num = 0
            den = 0
            for j in range(-16, 17):
                num += dr_dth[(i+j)%720] * dr_dt[(i+j)%720]
                den += np.square(dr_dth[(i+j)%720])
            dth_dt[i] = num/den

        prev_ranges_smoothed[0] = ranges_smoothed

        dth_dt = np.concatenate((dth_dt, dth_dt))
        spl2 = make_smoothing_spline(angles, dth_dt, lam = 0.2)
        dth_dt_smoothed = spl2(angles_smaller)

        dth_dt_smoothed = np.abs(dth_dt_smoothed)

        dx_smoothed = np.int16(np.multiply(dth_dt_smoothed, np.cos(angles2)))
        dy_smoothed = np.int16(np.multiply(dth_dt_smoothed, np.sin(angles2)))

        net_x = np.mean(dx_smoothed)
        net_y = np.mean(dy_smoothed)

        mag = np.sqrt(np.square(net_x) + np.square(net_y))
        angle = np.rad2deg(np.arctan2(net_y, net_x))

        # print(angle, mag)

        if mag > 3.5:
            self.value = round(angle)
            if (self.value < 0):
                self.value += 360
            self.new = True
            if debug:
                print(mag, self.value)

        if debug:
            dx_smoothed = np.add(np.clip(dx_smoothed,-img_size // 2,img_size // 2), img_size // 2)
            dy_smoothed = np.add(np.clip(dy_smoothed,-img_size // 2,img_size // 2) , img_size // 2)
            img[dy_smoothed[:], dx_smoothed[:], 2] = 255
            cv2.imshow("debug", img)
            cv2.waitKey(10)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.new:
                self.new = False
                turn = 0

                if (self.value < 178):
                    turn = int(180-self.value)

                elif (self.value > 182):
                    turn = -int(self.value - 180)

                self.pub.publish(Int32MultiArray(data=[turn]))
            r.sleep()

if __name__ == '__main__':
    a = LaserScanReader()
    a.run()
