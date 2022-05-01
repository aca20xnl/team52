#!/usr/bin/env python3
# a template for the move_square exercise

import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
import math
# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

class PathEight():

    def callback_function(self, odom_data):

        position_x = odom_data.pose.pose.position.x
        position_y = odom_data.pose.pose.position.y

        orientation_x = odom_data.pose.pose.position.x
        orientation_y = odom_data.pose.pose.position.y
        orientation_z = odom_data.pose.pose.position.z
        orientation_w = odom_data.pose.pose.position.z

        (roll,pitch,yaw) = euler_from_quaternion([orientation_x,orientation_y,orientation_z,orientation_w,'sxyz'])

        self.x = position_x
        self.y = position_y
        self.theta_z = yaw

        self.loop_rate += 1

        if self.loop_rate % self.rate_val == 0:
            print(f"x = {self.x:.2f} [m], y = {self.y:.2f} [m], yaw = {self.theta_z:.1f} [degrees].")


    def __init__(self):

        rospy.init_node('path_eight', anonymous=True)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
       
        self.rate_val = 10
        self.rate = rospy.Rate(self.rate_val) # hz
        self.loop_rate = 0 
            
        self.vel_cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The 'path_eight' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x=0.0
        self.vel_cmd.angular.z=0.0
        print("Stopping the robot")
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True

    def main_loop(self):
        startTime= rospy.get_rostime()
            
        path_rad=0.5
        lin_vel=0.13
        duration=rospy.get_rostime()-startTime 

        while duration.secs<2*math.pi*abs(path_rad)/abs(lin_vel):  
              
            duration=rospy.get_rostime()-startTime        

            self.vel_cmd.linear.x=lin_vel
            self.vel_cmd.angular.z=lin_vel/path_rad
                

            self.pub.publish(self.vel_cmd)

        while duration.secs>2*math.pi*abs(path_rad)/abs(lin_vel) and duration.secs<=2*2*math.pi*abs(path_rad)/abs(lin_vel)+1:

            path_rad=-0.5
                
            duration=rospy.get_rostime()-startTime        

            self.vel_cmd.linear.x=lin_vel
            self.vel_cmd.angular.z=lin_vel/path_rad
                

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()
        

if __name__ == '__main__':
    vel_ctrl = PathEight()
    try:
        vel_ctrl.main_loop()
    except rospy.ROSInterruptException:
        pass
