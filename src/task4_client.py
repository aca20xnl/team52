#!/usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import CameraSweepAction, CameraSweepGoal
import numpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from task4_tb3 import Tb3LaserScan

class Client(object):
   
    def feedback_callback(self, feedback_data):
        if self.init:
            #initial angle
            self.initial_angle = feedback_data.current_angle
            self.init = False
            self.find_angle = self.initial_angle 

        #current angle
        self.current_arc = feedback_data.current_angle

    def __init__(self):

        self.init=True

        self.goal = CameraSweepGoal()
        self.tb3_lidar = Tb3LaserScan()
        
        
        
        self.client = actionlib.SimpleActionClient("/task4_server", CameraSweepAction)
        self.client.wait_for_server()
        

        
        self.send_goal(0,100)
        rospy.sleep(0.5)
        self.image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        rate = rospy.Rate(10)

        
        self.sense_blue=False
        self.sense_red=False
        self.sense_green=False
        self.sense_turquiose=False
        self.sense_yellow=False
        self.sense_purple=False
        

        self.shut_down= rospy.on_shutdown(self.shutdown_ops)
        self.start=True
       

        
        while self.start:
            
            ideal_angle = self.initial_angle+90
            if ideal_angle>360:
                ideal_angle = ideal_angle - 360
            self.send_goal(ideal_angle,0)
            if abs( self.current_arc-ideal_angle)<=1 or abs(abs( self.current_arc-ideal_angle)-360)<=1:
                if self.blue_cy !=0:
                    self.sense_blue=True
                    print ("SEARCH INITIATED: The target beacon colour is Blue" )
                if self.red_cy !=0:
                    self.sense_red=True
                    print ("SEARCH INITIATED: The target beacon colour is Red")
                if self.green_cy !=0:
                    self.sense_green=True
                    print ("SEARCH INITIATED: The target beacon colour is Green")
                if self.turquoise_cy !=0:
                    self.sense_turquiose=True
                    print ("SEARCH INITIATED: The target beacon colour is Turquoise" )
                if self.yellow_cy !=0:
                    self.sense_yellow=True
                    print ("SEARCH INITIATED: The target beacon colour is Yellow" )
                if self.purple_cy !=0:
                    self.sense_purple=True
                    print ("SEARCH INITIATED: The target beacon colour is Purple" )

                self.start=False
                
                break
            rate.sleep()
         
      
        self.back=True
      
        
        #turn back to initial angle
        while self.back:

            self.send_goal(self.initial_angle,0)
            if (abs(abs(self.current_arc-self.initial_angle)-360)<=0.1) or  abs(self.current_arc-self.initial_angle)<=0.1:
                self.move=True
                self.back=False
                break
            rate.sleep()

       

        while self.move:
            
           if (( self.sense_blue and self.detect_blue) or (self.sense_red and self.detect_red) or (self.sense_green and self.detect_green) or (self.sense_turquiose and self.detect_turquiose) or  (self.sense_purple and self.detect_purple)):
                print ("TARGET DETECTED : Beaconing initiated.")
                self.continue_move=True
                self.move=False
                break
    

           else:
                if self.tb3_lidar.front<0.6 and (abs( abs(abs( self.current_arc- self.find_angle)-360)<=0.1 or self.current_arc- self.find_angle)<=0.1 ):  
                    self.find_angle = self.change_angle() 

                    
                self.send_goal(self.find_angle,10)
           rate.sleep()

        

        while self.continue_move:
        
            if self.tb3_lidar.min_distance>0.7:
                 self.send_goal(self.find_angle,102)
                
            # elif self.tb3_lidar.min_distance<0.7 and (( self.sense_blue and self.detect_blue) or (self.sense_red and self.detect_red) or (self.sense_green and self.detect_green) or (self.sense_turquiose and self.detect_turquiose) or  (self.sense_purple and self.detect_purple)):
            #      self.send_goal(self.search_angle,100)
            else:
                if self.tb3_lidar.min_distance<0.7:
                    
                 self.find_angle = self.change_angle() 
                    
                self.send_goal(self.find_angle,10)
            # elif  self.tb3_lidar.min_distance>0.6 and (int(self.green_m['m10']/(self.green_m['m00']+1e-10)))!=0 and self.green_m['m00'] > 100000:
            #    self.send_goal(self.search_angle,102)
            #    print(1)
            

            rate.sleep()



    def change_angle(self):
        if self.tb3_lidar.left <= self.tb3_lidar.right:
            if  self.current_arc>315 or  self.current_arc<45:
                return 270
            if  self.current_arc>45 and  self.current_arc<135:
                return 0
            if  self.current_arc>135 and  self.current_arc<225:
                return 90
            if  self.current_arc>225 or  self.current_arc<315 :
                return 180 
        else :
            if  self.current_arc<45 or   self.current_arc>315:
                return 90
            if  self.current_arc>45 and  self.current_arc<135:
                return 180
            if self.current_arc>135 and  self.current_arc<225:
                return 270
            if  self.current_arc>225 or  self.current_arc<315:
                return 0 

           

    def camera_callback(self,msg):
    
        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)# turn bgr image to hsv image for detection 
        #Thresholds for ["Blue", "Red", "Green", "Turquoise","Yellow","Purple"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100),(30, 200, 100),(140, 200, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255),(45, 255, 255),(155, 200, 100)]

        for i in range(6):
            if i == 0:
                blue_mask= cv2.inRange(hsv, numpy.array([115, 224, 100]),numpy.array([130, 255, 255]))
                self.blue_m = cv2.moments(blue_mask)
            elif i==1:
                red_mask = cv2.inRange(hsv, numpy.array([0, 185, 100]),numpy.array([10, 255, 255]))
                self.red_m = cv2.moments(red_mask)
            elif i==2:
                green_mask = cv2.inRange(hsv, numpy.array([25, 110, 105]),numpy.array([70, 255, 255]))
                self.green_m = cv2.moments(green_mask)
            elif i==3:
                turquoise_mask = cv2.inRange(hsv,numpy.array([75, 150, 100]),numpy.array([100, 255, 255]))
                self.turquoise_m = cv2.moments(turquoise_mask)
            elif i==4:
                yellow_mask = cv2.inRange(hsv, numpy.array([30, 200, 100]),numpy.array([45, 255, 255]))
                self.yellow_m = cv2.moments(yellow_mask)
            elif i==5:
                purple_mask = cv2.inRange(hsv, numpy.array([140, 200, 100]),numpy.array([155, 200, 100]))
                self.purple_m = cv2.moments(purple_mask)


        self.blue_cy = int(self.blue_m['m10']/(self.blue_m['m00']+1e-10))
        self.red_cy =int(self.red_m['m10']/(self.red_m['m00']+1e-10))
        self.green_cy = int(self.green_m['m10']/(self.green_m['m00']+1e-10))
        self.turquoise_cy = int(self.turquoise_m['m10']/(self.turquoise_m['m00']+1e-10))
        self.yellow_cy = int(self.yellow_m['m10']/(self.yellow_m['m00']+1e-10))
        self.purple_cy = int(self.purple_m['m10']/(self.purple_m['m00']+1e-10))
        

        if (int(self.blue_m['m10']/(self.blue_m['m00']+1e-10)))!=0 and self.blue_m['m00'] > 100000:
            self.detect_blue = True
        else:
            self.detect_blue = False

        if (int(self.red_m['m10']/(self.red_m['m00']+1e-10)))!=0 and self.red_m['m00'] > 100000:
            self.detect_red = True
        else:
            self.detect_red = False

        if (int(self.green_m['m10']/(self.green_m['m00']+1e-10)))!=0 and self.green_m['m00'] > 100000:
            self.detect_green = True
        else:
             self.detect_green = False
        
        if(int(self.turquoise_m['m10']/(self.turquoise_m['m00']+1e-10)))!=0 and self.turquoise_m['m00'] > 100000:
            self.detect_turquiose = True
        else:
            self.detect_turquiose = False

        if(int(self.yellow_m['m10']/(self.yellow_m['m00']+1e-10)))!=0 and self.yellow_m['m00'] > 100000:
            self.detect_yellow = True
        else:
            self.detect_yellow = False

        if (int(self.purple_m['m10']/(self.purple_m['m00']+1e-10)))!=0 and self.purple_m['m00'] > 100000:
            self.detect_purple = True
        else:
            self.detect_purple = False


    def send_goal(self, sweep_angle, image_count):
        self.goal.sweep_angle = sweep_angle
        self.goal.image_count = image_count
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def shutdown_ops(self):
        self.send_goal(0,0)
        self.ctrl_c = True


        

if __name__ == '__main__':
    rospy.init_node("task4_client")
    Client()
    rospy.spin()