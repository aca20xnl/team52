#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow, pi
import numpy as np

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
     

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.angle_closet_object=0.0
     
        
        self.vel_cmd = Twist()
        

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
    
    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x0 = pos_x
        self.y0 = pos_y
        self.theta_z = yaw 

      

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:11]
        right_arc = scan_data.ranges[-10:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        # self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
        self.left_angle=np.array(left_arc[::-1]).min()
        self.right_angle=np.array(right_arc[::-1]).min()


        self.min_distance=self.front_arc[self.front_arc !=0.0].min()
    
    def action_server_launcher(self, goal: SearchGoal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid velocity.  Select a value between 0 and 0.26 m/s.")
            success = False
        if goal.approach_distance <= 0.2:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"Request to move at {goal.fwd_velocity:.3f}m/s "
                f"and stop {goal.approach_distance:.2f}m "
                f"infront of any obstacles")

        # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0)
        startTime= rospy.get_rostime()
        duration=rospy.get_rostime()-startTime 
        self.turn = False
        self.walk=False
        path_rad=0.6

        lin_vel=0.26
        
        # position_x=[]
        # position_y=[]
        while duration.secs-startTime.secs <=90:
            if self.tb3_lidar.min_distance > 0.55:
                self.vel_cmd.linear.x=lin_vel
                self.vel_cmd.angular.z=lin_vel/path_rad
                self.pub.publish(self.vel_cmd)

                # duration=rospy.get_rostime()-startTime 
                # self.vel_controller.set_move_cmd(goal.fwd_velocity, 0)    
                # self.vel_controller.publish()
                self.angle_closet_object=self.tb3_lidar.closest_object_position
                self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        #       # populate the feedback message and publish it:
                self.feedback.current_distance_travelled = self.distance
                self.actionserver.publish_feedback(self.feedback)
                

                
               
            else:
                
                self.turn=True
                self.x0 = self.x
                self.y0 = self.y
                
                self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        #       # populate the feedback message and publish it:
                self.feedback.current_distance_travelled = self.distance
                self.actionserver.publish_feedback(self.feedback)
                

                
            # else:
                # self.vel_controller.set_move_cmd(0, 0)
                # self.vel_controller.publish()
            wait=0
            current_theta_z=self.theta_z
            turn_right=False
            turn_left=False
            
            # if self.angle_closet_object<0:
            #    turn_left=True
            # else:
            #    turn_right=True
            while self.turn :
                # while turn_left==True and self.turn:
                #     if abs(current_theta_z- self.theta_z) >= self.tb3_lidar.min_distance and wait > 5:
                #         self.walk=True
                #         self.turn=False
                #         self.finish_turn=False
                #         self.theta_z0 = self.theta_z
                #         wait = 0
                                    
                                    
                                
                #     else:
                #         self.vel = Twist()
                #         self.vel.angular.z = 0.2
                #         self.pub.publish(self.vel)
                #         wait += 1
                    
                    

                    
                    
                

                # while turn_right==True and self.turn:
                    if self.tb3_lidar.left_angle<=self.tb3_lidar.right_angle:
                        if abs(current_theta_z- self.theta_z) >= self.tb3_lidar.min_distance and wait > 5:


                            self.walk=True
                            self.turn=False
                            self.theta_z0 = self.theta_z
                            wait = 0
                            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
                        # populate the feedback message and publish it:
                            self.feedback.current_distance_travelled = self.distance
                            self.actionserver.publish_feedback(self.feedback)

                        
                        
                    
                        else:
                            self.vel = Twist()
                            self.vel.angular.z = -0.7
                            self.pub.publish(self.vel)
                            wait += 1
                            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
                            # populate the feedback message and publish it:
                            self.feedback.current_distance_travelled = self.distance
                            self.actionserver.publish_feedback(self.feedback)

                    else:
                        
                        if abs(current_theta_z- self.theta_z) >= self.tb3_lidar.min_distance and wait > 5:


                            self.walk=True
                            self.turn=False
                            self.theta_z0 = self.theta_z
                            wait = 0
                            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
                        # populate the feedback message and publish it:
                            self.feedback.current_distance_travelled = self.distance
                            self.actionserver.publish_feedback(self.feedback)

                        
                        
                    
                        else:
                            self.vel = Twist()
                            self.vel.angular.z = -1
                            self.pub.publish(self.vel)
                            wait += 1
                            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
                            # populate the feedback message and publish it:
                            self.feedback.current_distance_travelled = self.distance
                            self.actionserver.publish_feedback(self.feedback)
                    # if self.tb3_lidar.left>=self.tb3_lidar.right:
                    #     if abs(current_theta_z- self.theta_z) >= self.tb3_lidar.min_distance and wait > 5:


                    #         self.walk=True
                    #         self.turn=False
                    #         self.theta_z0 = self.theta_z
                    #         wait = 0
                    #         self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
                    #     # populate the feedback message and publish it:
                    #         self.feedback.current_distance_travelled = self.distance
                    #         self.actionserver.publish_feedback(self.feedback)

                        
                        
                    
                    #     else:
                    #         self.vel = Twist()
                           
                    #         self.vel.angular.z = 0.26
                    #         self.pub.publish(self.vel)
                    #         wait += 1
                    #         self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
                    #         # populate the feedback message and publish it:
                    #         self.feedback.current_distance_travelled = self.distance
                    #         self.actionserver.publish_feedback(self.feedback)

            

            

            
                 



                 
           
           
        # self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        # # populate the feedback message and publish it:
        # self.feedback.current_distance_travelled = self.distance
        # self.actionserver.publish_feedback(self.feedback)
        #     # elif self.walk:
            #      self.vel_controller.set_move_cmd(0, 0)
            #      self.vel_controller.publish()

      

        
        # while self.tb3_lidar.min_distance > goal.approach_distance:
        #     self.vel_controller.publish()
        #     self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        #     # populate the feedback message and publish it:
        #     self.feedback.current_distance_travelled = self.distance
        #     self.actionserver.publish_feedback(self.feedback)


        # while self.turn_left: 
        #     # wait=0
        #     # if wait <5:
            
        #     self.vel = Twist()
        #     self.vel.angular.z = -0.2
        #     status = "turning"
        #     self.pub.publish(self.vel)
        #     self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
        #     # populate the feedback message and publish it:
        #     self.feedback.current_distance_travelled = self.distance
        #     self.actionserver.publish_feedback(self.feedback)
        #     print(self.theta_z)


        #     if abs(self.theta_z0 - self.theta_z) <= pi/4:
        #         self.walk=True
        #         self.turn=False
        #         print(self.theta_z)
        #         break

        
       
        

        if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position

            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()