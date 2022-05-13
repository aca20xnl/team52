#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import random

from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction

from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
import numpy as np
import math
import datetime as dt
import os
import time

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image


class Task5Server(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):

        self.other_colour_detected = False
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.speed = True
        self.box_colour = ""

        self.colorfound = 'null'
        self.colordec = []

        self.color = ['red','yellow','green','turquoise','blue','purple']

        global state_, state_dict_
        state_ = 0
        state_dict_ = {
            0: 'detect initial colour',

            1: 'turn left',
            2: 'follow the wall',
            3: 'turn right',

            4: 'found target',
            5: 'scan_surroundings',

        }

        self.init_maze_runner()

        self.init_colour_search()

    def init_colour_search(self):
        #rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        #self.robot_controller = MoveTB3()
        #self.turn_vel_fast = -0.5
        #self.turn_vel_slow = -0.1
        #self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        #self.move_rate = '' # fast, slow or stop
        #self.stop_counter = 0

        #self.ctrl_c = False
        #rospy.on_shutdown(self.shutdown_ops)

        #self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000

        self.lower = [(-1.8, 217, 100),(25, 130, 100),(55, 130, 100),(85, 135, 100),(115, 225, 100),(145, 150, 100)]
        self.upper = [(3.3, 255, 255),(33, 255, 255),(65, 255, 255),(93, 255, 255),(130, 255, 255),(155, 255, 255)]

        


    def init_maze_runner(self):
        self.actionserver = actionlib.SimpleActionServer("/task5_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        
    
        self.camera_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.front_min = -1
        self.back_min = 0
        self.left_min = 0
        self.right_min = -1

        global regions_
        regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        
    
    def camera_callback(self, img_data):
        """
        global state_
        if state_ != 0:
            return
        """
        #print("1")
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        #print("RUNNIG")
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        #crop_y = int((height/2) - (crop_height/2))
        crop_y = 300 + (crop_height / 2)

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # print(len(hsv_img))

        #color = ['red','yellow','green','turquoise','blue','purple']

        self.colordec = []
        for i in range(6):
            mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            m = cv2.moments(mask)
            if m['m00'] > self.m00_min:
                self.colordec.append(self.color[i])
        
        if self.box_colour != "":
            index = self.color.index(self.box_colour)
            mask = cv2.inRange(hsv_img, self.lower[index], self.upper[index])
            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)
            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

        print(self.other_colour_detected)

        if len(self.colordec) != 0 and self.other_colour_detected == False:
            for colour in self.colordec:
                if colour != self.box_colour and self.box_colour != "":
                    print("Detected: " + colour)
                    self.other_colour_detected = True
                    self.speed = False
        
        if self.other_colour_detected == True and self.box_colour in self.colordec and self.m00 >= 71000000:
            self.change_state(4)
        
        """
        global state_
        if state_ == 4:
            return
        if len(self.colordec) != 0:
            if self.colordec[0] == self.box_colour and self.other_colour_detected == True:
                self.change_state(5)
        """

    def scan_callback(self, scan_data):

        #print("scan_callbvack1")
        global state_
        #print(state_)
        """
        if state_ == 4:
            #Front
            left_arc = scan_data.ranges[0:30]
            right_arc = scan_data.ranges[-30:]
            front_arc = np.array(left_arc[::-1] + right_arc[::-1])
            arc_angles = np.arange(-30, 30)
            self.front_min = front_arc.min()
        """
        

        #print("scan_callbvack2")
        #Front
        left_arc = scan_data.ranges[0:30]
        right_arc = scan_data.ranges[-30:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        arc_angles = np.arange(-30, 30)
        self.front_min = front_arc.min()
        #print(self.front_min)

        #Back
        back_arc = np.array(scan_data.ranges[150: 210])
        arc_angles = np.arange(150, 210)
        self.back_min = back_arc.min()
        
        #Left
        left_arc = np.array(scan_data.ranges[60: 120])
        arc_angles = np.arange(60, 120)
        self.left_min = left_arc.min()
        
        #Right
        right_arc = np.array(scan_data.ranges[270:300])
        arc_angles = np.arange(270, 300)
        self.right_min = right_arc.min()

        #Fleft
        fleft_arc = np.array(scan_data.ranges[30: 60])
        arc_angles = np.arange(60, 120)
        self.fleft_min = left_arc.min() 

        #Fright
        fright_arc = np.array(scan_data.ranges[300: 330])
        arc_angles = np.arange(60, 120)
        self.fright_min = fright_arc.min() 

        global regions_
        regions_ = {
            'right':  right_arc.min(),
            'fright': fright_arc.min(),
            'front':  front_arc.min(),
            'fleft':  fleft_arc.min(),
            'left':   left_arc.min(),
        }

        if state_ != 1 or state_ != 2 or state_ != 3:
            self.take_action()




    def dist_from_start(self):
        pos_x = self.robot_odom.posx
        pos_y = self.robot_odom.pos_y

        return abs(pos_x**2 + pos_y**2)

    def find_surround_color(self):
        out = []
        self.robot_controller.set_move_cmd(0,0)
        self.robot_controller.publish()
        time.sleep(1)
        self.robot_controller.set_move_cmd(0,0.61)
        self.robot_controller.publish()
        timer = 0
        while timer < 11:
            print("surround colour loop")
            for color in self.colordec:
                if color not in out:
                    out.append(color)
            time.sleep(1)
            timer += 1
        
        if len(out) > 1:
            if self.box_colour in out:
                self.change_state(4)
        else:
            self.change_state(1)
        
    
    def find_init_color(self):
        #dectecting origin color
        self.robot_controller.set_move_cmd(0,0)
        self.robot_controller.publish()
        time.sleep(1)
        
        self.robot_controller.set_move_cmd(0,0.6)
        self.robot_controller.publish()
        time.sleep(3)

        self.box_colour = self.colordec[0]

        self.robot_controller.set_move_cmd(0,0)
        self.robot_controller.publish()
        time.sleep(1)
        self.robot_controller.set_move_cmd(0,-0.6)
        self.robot_controller.publish()
        time.sleep(3.1)
        self.robot_controller.set_move_cmd(0,0)
        self.robot_controller.publish()
        time.sleep(0.5)
        
        self.change_state(1)
        print("Found Colour")        

    def change_state(self, state):
        if state is not state_:
            global state_, state_dict_
            print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
            state_ = state


    def take_action(self):
        global regions_
        regions = regions_
    
        state_description = ''
    
        front_d = 0.4
        right_d = 0.4
        left_d = 0.4

        success = False

        if regions['front'] > front_d and regions['fright'] > right_d and regions['right'] > right_d:
            state_description = 'case 1 - too far from right wall - turn right'
            self.change_state(3)
        elif regions['front'] > front_d and regions['fright'] > right_d and regions['right'] < right_d:
            state_description = 'case 2 - close to right wall - go forward'
            self.change_state(3)
        elif regions['front'] > front_d and regions['fright'] < right_d and regions['right'] < right_d:
            state_description = 'case 3 - very close to right wall - turn right'
            self.change_state(2)
        elif regions['front'] < front_d and regions['fright'] < right_d and regions['right'] < right_d:
            state_description = 'case 4 - close to right wall and forward wall - turn left' 
            self.change_state(1)
        

        return
    

    def action_server_launcher(self, goal):
        r = rospy.Rate(20)
        while True:
            if self.speed:
                speed_ = [0.3,0.5]
            else:
                speed_ = [0.2,0.4]
            print("state:",state_)
            if state_ == 0:
                #self.robot_controller.set_move_cmd(linear=0.2, angular=-0.5)
                self.find_init_color()
            elif state_ == 1:
                self.robot_controller.set_move_cmd(linear=0.0, angular=speed_[1])
            elif state_ == 2:
                self.robot_controller.set_move_cmd(linear=speed_[0], angular=0.0)
            elif state_ == 3:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-speed_[1])
            elif state_ == 4:
                self.robot_controller.set_move_cmd(0,0)
                self.robot_controller.publish()
                time.sleep(0.5)
                self.robot_controller.set_move_cmd(0.1,0)
                self.robot_controller.publish()
                time.sleep(1)
                self.robot_controller.set_move_cmd(0,0)
                self.robot_controller.publish()
                print("done")
                time.sleep(200)
                break
            else:
                rospy.logerr('Unknown state!')

            self.robot_controller.publish()

            r.sleep()

        if success:
            rospy.loginfo('Camera sweep completed sucessfully.')
            self.result.image_path = self.base_image_path
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()

if __name__ == '__main__':
    rospy.init_node('task5_server') 
    Task5Server()
    rospy.spin()