#! /usr/bin/env python

import rospy
import actionlib

# from com2009_actions.msg import CameraSweepAction, CameraSweepGoal
from com2009_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback
import numpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# setup a cmd_vel publisher and an odom subscriber:
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry




class colour_search(object):

    def __init__(self):
        
        self.m00 = 0
        self.m00_min = 10000

        self.goalc = Point()
        self.goalc.x = -1.46
        self.goalc.y = 0.23

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.startup=True
        self.current_theta_z=0
        self.turn_right=False
        self.turn_left=True
        self.tb3_lidar = Tb3LaserScan()
        self.move=False
        self.turn=False
        self.move_forward=False
        self.vel_cmd = Twist()

        self.hasPrint = False


    def scan_callback(self, scan_data):
        self.front = min(min(scan_data.ranges[0:20]),min(scan_data.ranges[340:359]))
        self.left = min(scan_data.ranges[85:95])
        self.right = min(scan_data.ranges[265:275])
        self.min_distance = min(min(scan_data.ranges[0:70]),min(scan_data.ranges[290:359]))

    def camera_callback(self,msg):
    
        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)# turn bgr image to hsv image for detection 
        lower_blue = numpy.array([[100, 200, 100]])
        upper_blue = numpy.array([124, 255, 240])
        lower_green = numpy.array([35, 110, 106])#35
        upper_green = numpy.array([74, 255, 255])
        lower_red = numpy.array([0, 200, 100])
        upper_red  = numpy.array([20, 255, 255])
        lower_Turquoise = numpy.array([90, 150, 100])
        upper_Turquoise  = numpy.array([100, 255, 255])
        lower_Purple = numpy.array([150, 200, 100])
        upper_Purple = numpy.array([180, 255, 255])
        lower_yellow = numpy.array([30, 200, 100])
        upper_yellow = numpy.array([50, 255, 255])

        green_mask= cv2.inRange(hsv, lower_green, upper_green)#get the green one from the image 
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)#get the blue one from the image
        red_mask = cv2.inRange(hsv, lower_red, upper_red)#get the red one from the image
        Turquoise_mask = cv2.inRange(hsv, lower_Turquoise, upper_Turquoise)#get the Turquoise one from the image
        Purple_mask = cv2.inRange(hsv, lower_Purple, upper_Purple)#get the  Purple one from the image
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)#get the yellow one from the image

        red_M = cv2.moments(red_mask)#get the size of red area
        green_M = cv2.moments(green_mask)#get the size of green area
        blue_M = cv2.moments(blue_mask)#get the size of blue area
        Turquoise_M = cv2.moments(Turquoise_mask)#get the size of Turquoise area
        Purple_M = cv2.moments(Purple_mask)#get the size of Purple area
        yellow_M = cv2.moments(yellow_mask)#get the size of yellow area

        self.cx_red = int(red_M['m10']/(red_M['m00']+1e-10))
        self.cx_green = int(green_M['m10']/(green_M['m00']+1e-10))
        self.cx_blue = int(blue_M['m10']/(blue_M['m00']+1e-10))
        self.cx_Turquoise = int(Turquoise_M['m10']/(Turquoise_M['m00']+1e-10))
        self.cx_Purple = int(Purple_M['m10']/(Purple_M['m00']+1e-10))
        self.cx_yellow = int(yellow_M['m10']/(yellow_M['m00']+1e-10))

        if self.cx_red!=0 and red_M['m00'] > 100000:
            self.detectred = True
        else:
            self.detectred = False
        if self.cx_green!=0 and green_M['m00'] > 100000:
            self.detectgreen = True
        else:
            self.detectgreen = False
        if self.cx_blue!=0 and blue_M['m00'] > 100000:
            self.detectblue = True
        else:
            self.detectblue = False
        if self.cx_Turquoise!=0 and Turquoise_M['m00'] > 100000:
            self.detectTurquoise = True
        else:
            self.detectTurquoise = False
        if self.cx_Purple!=0 and Purple_M['m00'] > 100000:
            self.detectPurple = True
        else:
            self.detectPurple = False
        if self.cx_yellow!=0 and yellow_M['m00'] > 100000:
            self.detectyellow = True
        else:
            self.detectyellow = False

    def judge_desired_angle(self):
        if self.left >= self.right:
            if self.current_angle>315 or self.current_angle<45:
                return 90
            if self.current_angle>45 and self.current_angle<135:
                return 180
            if self.current_angle>135 and self.current_angle<225:
                return 270
            if self.current_angle>225 or self.current_angle<315:
                return 0 
        if self.left < self.right:
            if self.current_angle>315 or self.current_angle<45:
                return 270
            if self.current_angle>45 and self.current_angle<135:
                return 0
            if self.current_angle>135 and self.current_angle<225:
                return 90
            if self.current_angle>225 or self.current_angle<315:
                return 180 

    def send_goal(self, sweep_angle, image_count):
        self.goal.sweep_angle = sweep_angle
        self.goal.image_count = image_count
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def main(self):

        while not self.ctrl_c:
            
            # get initial yaw value
           
            self.search = False
            
            
            countc_0 = 0            
            countc_1 = 0
            countc_2 = 0
            countc_3 = 0
            countc_4 = 0

            #START ZONE C - BLUE 
            if self.x0 >= 2.0 and self.x0 <= 2.1 and self.y0 >= 1.9 and self.y0 <= 2.0:
                

                if self.turn_left:
                    if abs(self.theta_z0 - self.theta_z) <= pi/2:
                        self.vel = Twist()
                        self.vel.angular.z = 0.2                   
                        self.pub.publish(self.vel)
                    else:
                        self.vel = Twist()
                        self.vel.angular.z = 0.0
                        self.current_theta_z=self.theta_z
                        self.pub.publish(self.vel)
                        self.turn_right=True
                        self.turn_left=False

                if self.turn_right:
                    if abs(self.current_theta_z - self.theta_z) <= pi/2:
 
                        self.vel = Twist()
                        self.vel.angular.z = -0.2                   
                        self.pub.publish(self.vel)
                    else:
                        self.vel = Twist()
                        self.vel.angular.z = 0.0
                        self.pub.publish(self.vel)
                        self.move=True
                        self.turn_right=False


          
            
            while self.move:
                if self.y<=1.98 and self.y>=1.48:
                  self.vel = Twist()
                  self.vel.linear.x = 0.2                   
                  self.pub.publish(self.vel)
                else:
                    self.turn=True
                    self.move=False
                    self.current_theta_z=self.theta_z
                    self.x0 = self.x
                    self.y0 = self.y

            wait=0
            
            if self.turn:
                    if abs(self.current_theta_z - self.theta_z) <= pi/2:
 
                        self.vel = Twist()
                        self.vel.angular.z = -0.2                   
                        self.pub.publish(self.vel)
                    else:
                        self.vel = Twist()
                        self.vel.angular.z = 0.0
                        self.pub.publish(self.vel)
                        self.turn=False
                        self.move_forward=True

            while self.move_forward:
                if self.y<=1.98 and self.y>=1.48:
                  self.vel = Twist()
                  self.vel.linear.x = 0.2                   
                  self.pub.publish(self.vel)
                else:
                    self.turn=True
                    self.move=False
                    self.current_theta_z=self.theta_z
                    self.x0 = self.x
                    self.y0 = self.y




               

                    
            
if __name__ == '__main__':
    rospy.init_node("task3_client")
    avoid_client()
    rospy.spin()