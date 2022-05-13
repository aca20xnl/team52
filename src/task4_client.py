#! /usr/bin/env python3

import rospy
import actionlib

# from com2009_actions.msg import CameraSweepAction, CameraSweepGoal
from com2009_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback
import numpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class avoid_client(object):
   
    def feedback_callback(self, feedback_data):
        if self.firstin:
            self.inityaw = feedback_data.current_angle
            #print self.inityaw
            self.firstin = False
            self.search_angle = self.inityaw
        self.current_angle = feedback_data.current_angle

    def __init__(self):
        
        self.goal = CameraSweepGoal()
        
        self.client = actionlib.SimpleActionClient("/task3_server", CameraSweepAction)
        self.client.wait_for_server()
        self.firstin=True

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)
        rospy.sleep(0.5)
        self.send_goal(0,0)
        rospy.sleep(0.5)
        self.image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        rate = rospy.Rate(10)
        #find target color 
        while not rospy.is_shutdown():
            desired_angle = self.inityaw+180
            if desired_angle>360:
                desired_angle = desired_angle - 360
            self.send_goal(desired_angle,0)
            if abs(self.current_angle-desired_angle)<2 or abs(abs(self.current_angle-desired_angle)-360)<2:
                if self.cx_red !=0:
                    self.hasfind=True
                    self.targetcolour=1
                    print("SEARCH INITIATED: The target beacon colour is Red")
                if self.cx_green !=0:
                    self.hasfind=True
                    self.targetcolour=2
                    print ("SEARCH INITIATED: The target beacon colour is Green") 
                if self.cx_blue !=0:
                    self.hasfind=True
                    self.targetcolour=3
                    print("SEARCH INITIATED: The target beacon colour is Blue")
                if self.cx_Turquoise !=0:
                    self.hasfind=True
                    self.targetcolour=4
                    print("SEARCH INITIATED: The target beacon colour is Turquoise")
                if self.cx_Purple !=0:
                    self.hasfind=True
                    self.targetcolour=5
                    print("SEARCH INITIATED: The target beacon colour is Purple")
                if self.cx_yellow !=0:
                    self.hasfind=True
                    self.targetcolour=6
                    print("SEARCH INITIATED: The target beacon colour is yellow")
                break;
            rate.sleep()
        #turn to init yaw    
        while not rospy.is_shutdown():
            self.send_goal(self.inityaw,0)
            if abs(self.current_angle-self.inityaw)<0.5 or abs(abs(self.current_angle-self.inityaw)-360)<0.5:
                break;
            rate.sleep()

        #start moving     
        while not rospy.is_shutdown():
            if((self.targetcolour==1 and self.detectred) or (self.targetcolour==2 and self.detectgreen) or (self.targetcolour==3 and self.detectblue) or (self.targetcolour==4 and self.detectTurquoise) or (self.targetcolour==5 and self.detectPurple) or (self.targetcolour==6 and self.detectyellow)):
                print("BEACON DETECTED: Beaconing initiated.")
                break
            else:
                if self.front<0.6 and (abs(self.current_angle-self.search_angle)<5 or abs(abs(self.current_angle-self.search_angle)-360)<5):
                    self.search_angle = self.judge_desired_angle()    
                self.send_goal(self.search_angle,7)
            rate.sleep()
           
        #approach target color    
        while not rospy.is_shutdown():
            if self.targetcolour==1:
                self.imageerror=(960.0-self.cx_red)/100
            if self.targetcolour==2:
                self.imageerror=(650.0-self.cx_green)/100    
            if self.targetcolour==3:
                self.imageerror=(900.0-self.cx_blue)/100
            if self.targetcolour==4:
                self.imageerror=(960.0-self.cx_Turquoise)/100
            if self.targetcolour==5:
                self.imageerror=(960.0-self.cx_Purple)/100
            if self.targetcolour==6:
                self.imageerror=(960.0-self.cx_yellow)/100

            if abs(self.imageerror)<10 and self.min_distance<0.27:
                self.send_goal(self.current_angle,0)
                print("BEACONING COMPLETE: The robot has now stopped.") 
            else:
                self.send_goal(self.imageerror,self.targetcolour)
            rate.sleep()

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
        

if __name__ == '__main__':
    rospy.init_node("task3_client")
    avoid_client()
    rospy.spin()