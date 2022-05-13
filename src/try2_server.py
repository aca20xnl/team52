#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import random

# Import some image processing modules:
#import cv2
#from cv_bridge import CvBridge

# Import all the necessary ROS message types:
#from com2009_actions.msg import CameraSweepFeedback, CameraSweepResult, CameraSweepAction
#from sensor_msgs.msg import CompressedImage
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
# from move_tb3 import MoveTB3
# from tb3_odometry import TB3Odometry
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan


# Import some other useful Python Modules
import numpy as np
import math
import datetime as dt
import os

class AvoidServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/obstacle_avoid_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        #self.base_image_path = '/home/student/myrosdata/week5_images'
        self.camera_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)
        #self.cv_image = CvBridge()

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
    
    def scan_callback(self, scan_data):


        self.scan_data = scan_data

        #Left
        left_arc = scan_data.ranges[17:40]

        front_arc = np.array(left_arc[::-1])

        arc_angles = np.arange(17, 40)
        
        self.min_left_dist = front_arc.min()
        self.min_left_angle = arc_angles[np.argmin(front_arc)]

        self.max_left_dist = front_arc.max()
        self.max_left_angle = arc_angles[np.argmax(front_arc)]

        #Right
        right_arc = scan_data.ranges[-40:-17]

        front_arc = np.array(right_arc[::-1])

        arc_angles = np.arange(-40, -17)
        
        self.min_right_dist = front_arc.min()
        self.min_right_angle = arc_angles[np.argmin(front_arc)]

        self.max_right_dist = front_arc.max()
        self.max_right_angle = arc_angles[np.argmax(front_arc)]

       

        #small front
        left_arc = scan_data.ranges[0:26]
        right_arc = scan_data.ranges[-25:]

        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        arc_angles = np.arange(-25, 26)
        
        self.min_front_distance = front_arc.min()
        self.min_front_angle = arc_angles[np.argmin(front_arc)]

        self.max_front_distance = front_arc.max()
        self.max_front_angle = arc_angles[np.argmax(front_arc)]

        # front
        left_arc = scan_data.ranges[0:51]
        right_arc = scan_data.ranges[-50:]

        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        arc_angles = np.arange(-50, 51)
        
        self.min_distance = front_arc.min()
        self.object_angle = arc_angles[np.argmin(front_arc)]

        self.max_distance = front_arc.max()
        self.max_angle = arc_angles[np.argmax(front_arc)]

        # 360
        left_arc = scan_data.ranges[0:181]
        right_arc = scan_data.ranges[-180:]

        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        arc_angles = np.arange(-180, 180)

        self.min_around_distance = front_arc.min()
        self.min_around_angle = arc_angles[np.argmin(front_arc)]

        self.max_around_distance = front_arc.max()
        self.max_around_angle = arc_angles[np.argmax(front_arc)]
    

        

    def min_dist(self, left, right):

        left_arc =self.scan_data.ranges[0:left]
        right_arc = self.scan_data.ranges[right:]

        arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        arc_angles = np.arange(right, left)

        return arc.min()

    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        #self.robot_controller.set_move_cmd(linear=goal.fwd_velocity)

       # self.robot_controller.set_move_cmd(linear=1, angular=0.1)
        #self.robot_controller.publish()


        state = 0 
        target_angle = -1
        turning = False
        turn_target_dist = 0
        has_target = False
        target_dist = 0
        target_pos_x = 0.0
        target_pos_y = 0.0
        while True:
           
           
            if state == 0:
                if has_target == False:
                    if self.min_front_distance > 0.7:
                        #print(5)
                        has_target = True
                        target_dist = self.min_front_distance
                        target_pos_x = self.robot_odom.posx + turn_target_dist * math.sin(self.min_front_angle)
                        target_pos_y = self.robot_odom.posy + turn_target_dist * math.cos(self.min_front_angle)
                        self.robot_controller.set_move_cmd(linear=0.4, angular=0.0)
                    else:
                        
                        if turning == False:
                            #print(8)
                            if self.max_left_dist < self.max_right_dist:
                                #print(9)
                                target_angle = self.max_left_angle
                                turn_target_dist = self.max_left_dist
                                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
                            else:
                                #print(10)
                                target_angle = self.max_left_angle
                                turn_target_dist = self.max_right_dist
                                self.robot_controller.set_move_cmd(linear=0.0, angular=0.6)
                            turning = True
                        else:
                            #print(11)
                            
                            target_x = self.robot_odom.posx + turn_target_dist * math.sin(target_angle)
                            target_y = self.robot_odom.posy + turn_target_dist * math.cos(target_angle)

                            inc_x = target_x - self.robot_odom.posx
                            inc_y = target_y - self.robot_odom.posy

                            angle_to_goal = math.atan2 (inc_y, inc_x)

                            #print("Diff", angle_to_goal - self.robot_odom.yaw)
                            if abs(angle_to_goal - self.robot_odom.yaw) < 0.1:
                                #print("Diff", diff)
                                turning = False
                                has_target = True
                                target_dist = self.max_front_distance
                                target_pos_x = self.robot_odom.posx + turn_target_dist * math.sin(self.max_front_angle)
                                target_pos_y = self.robot_odom.posy + turn_target_dist * math.cos(self.max_front_angle)
                                self.robot_controller.set_move_cmd(linear=0.4, angular=0.0)
                            
                            elif target_angle < 0.0 and turning == False:
                                
                                self.robot_controller.set_move_cmd(linear=0.0, angular=0.6)
                            elif turning == False and target_angle >= 0.0:
                                
                                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
                        
                        
                        
                        #self.robot_controller.publish()
                        
                else:

                    print(-1)
                    if self.min_front_distance <= 0.7:
                        print(-2)
                        has_target = False
                        turning = False
                        target_pos_x = 0.0
                        target_pos_y = 0.0
                        self.robot_controller.set_move_cmd(linear=0.0, angular=0.0)
                    elif math.sqrt((self.robot_odom.posx - target_pos_x)**2 + (self.robot_odom.posy - target_pos_y)**2) > 2.5:
                        state = 1
                        
                        self.robot_controller.set_move_cmd(linear=0.0, angular=0.0)
                        print(-3)
                        
                    print(math.sqrt((self.robot_odom.posx - target_pos_x)**2 + (self.robot_odom.posy - target_pos_y)**2))
            else:
                print("Cyka")
                random_number = random.random()
                if random_number <= 0.5:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=0.5)
                else:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=-0.5)
                
                self.robot_controller.publish()
                rospy.sleep(3)

                state = 0
                has_target = False
                turning = False
                target_pos_x = 0.0
                target_pos_y = 0.0
                
                
                    
            self.robot_controller.publish()
           

            
           

            

        

        




        


        """
        success = True
        if goal.sweep_angle <= 0 or goal.sweep_angle > 180:
            print("Invalid sweep_angle.  Select a value between 1 and 180 degrees.")
            success = False
        if goal.image_count <=0:
            print("I can't capture a negative number of images!")
            success = False
        elif goal.image_count > 50:
            print("Woah, too many images! I can do a maximum of 50.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print("Request to capture {} images over a {} degree sweep".format(goal.image_count, goal.sweep_angle))

        # calculate the angular increments over which to capture images:
        ang_incs = goal.sweep_angle/float(goal.image_count)
        print("Capture an image every {:.3f} degrees".format(ang_incs))

        turn_vel = 0.2 # rad/s
        full_sweep_time = radians(goal.sweep_angle)/abs(turn_vel)

        print("The full sweep will take {:.5f} seconds".format(full_sweep_time))

        # set the robot velocity:
        self.robot_controller.set_move_cmd(0.0, turn_vel)
        
        # Get the current robot odometry (yaw only):
        ref_yaw = self.robot_odom.yaw
        start_yaw = self.robot_odom.yaw

        # Get the current date and time and create a timestamp string of it
        # (to use when we construct the image filename):
        start_time = dt.datetime.strftime(dt.datetime.now(),'%Y%m%d_%H%M%S')
        
        i = 0
        while i < goal.image_count:
            self.robot_controller.publish()
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo('Cancelling the camera sweep.')
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            
            if abs(self.robot_odom.yaw - ref_yaw) >= ang_incs:
                # increment the image counter
                i += 1
                
                # populate the feedback message and publish it:
                rospy.loginfo('Captured image {}'.format(i))
                self.feedback.current_image = i
                self.feedback.current_angle = abs(self.robot_odom.yaw)
                self.actionserver.publish_feedback(self.feedback)

                # update the reference odometry:
                ref_yaw = self.robot_odom.yaw

                # save the most recently captured image:
                cv2.imwrite(os.path.join(self.base_image_path, "{}_img{:03.0f}.jpg".format(start_time, i)), 
                    self.current_camera_image)
        
        if success:
            rospy.loginfo('Camera sweep completed sucessfully.')
            self.result.image_path = self.base_image_path
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
        """
            
if __name__ == '__main__':
    rospy.init_node('camera_sweep_action_server')
    AvoidServer()
    rospy.spin()
