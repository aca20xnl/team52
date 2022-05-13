#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi
import numpy as np


class Maze:
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
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def callback_lidar(self, lidar_data):
        """Obtain a subset of the LaserScan.ranges array corresponding to a +/-10 degree arc in front of it.
        Convert this subset to a numpy array to allow for more advanced processing."""
        rightleft_arc = lidar_data.ranges[270:295]
        rightright_arc = lidar_data.ranges[250:270]
        right_arc = np.array(rightleft_arc + rightright_arc)
        # find the miniumum object distance within the frontal laserscan arc:
        self.rightobject_distance = right_arc.min()

        frontleft_arc = lidar_data.ranges[0:10]
        frontright_arc = lidar_data.ranges[-10:]
        front_arc = np.array(frontleft_arc + frontright_arc)
        # find the miniumum object distance within the frontal laserscan arc:
        self.frontobject_distance = front_arc.min()

    def __init__(self):
        node_name = "maze"
        
        self.startup = True
        self.turnright = False
        self.turnleft = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def print_stuff(self, a_message):
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")

    def main_loop(self):
        status = ""
        wait = 0
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
                status = "init"

            elif self.turnright:
                if abs(self.theta_z0 - self.theta_z) >= (pi/2)-0.05 and wait > 5:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    self.turnright = False
                    self.vel = Twist()
                    self.theta_z0 = self.theta_z
                    status = "turn-fwd transition"
                    wait = 0

                    while sqrt(pow(self.x0 - self.x, 2) + pow(self.y0 - self.y, 2)) <= 0.30:
                    #
                        self.vel.linear.x = 0.1
                        self.pub.publish(self.vel)
                        print(f"Front Lidar range: {self.frontobject_distance:.2f}, "f"Right Lidar range: {self.rightobject_distance:.2f}, ")
                        continue

                else:
                    self.vel = Twist()
                    self.vel.angular.z = -0.2
                    status = "turning"
                    wait += 1

            elif self.turnleft:
                if abs(self.theta_z0 - self.theta_z) >= (pi/2)-0.05 and wait > 5:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    self.turnleft = False
                    self.vel = Twist()
                    self.theta_z0 = self.theta_z
                    status = "turn-fwd transition"
                    wait = 0

                    # while sqrt(pow(self.x0 - self.x, 2) + pow(self.y0 - self.y, 2)) <= 0.35:
                    # #
                    #     self.vel.linear.x = 0.1
                    #     self.pub.publish(self.vel)
                    #     print(f"Front Lidar range: {self.frontobject_distance:.2f}, "f"Right Lidar range: {self.rightobject_distance:.2f}, ")
                    #     continue

                else:
                    self.vel = Twist()
                    self.vel.angular.z = 0.2
                    status = "turning"
                    wait += 1

            else:

                # if there's a wall in front and on the right then turn left
                if self.frontobject_distance <= 0.35 and self.rightobject_distance < 0.5:
                    self.vel = Twist()
                    self.turnleft = True
                    self.x0 = self.x
                    self.y0 = self.y
                    print("left")

                    # while sqrt(pow(self.x0 - self.x, 2) + pow(self.y0 - self.y, 2)) <= 0.1:
                    # # slightly move forward after turning
                        
                    #     self.vel.linear.x = 0.1
                    #     self.pub.publish(self.vel)
                    #     continue

                # if there's not a wall on the right then turn right
                if  self.rightobject_distance > 0.5:
                    self.vel = Twist()
                    self.turnright = True
                    self.x0 = self.x
                    self.y0 = self.y
                    
                    # while sqrt(pow(self.x0 - self.x, 2) + pow(self.y0 - self.y, 2)) <= 0.07:
                    # # slightly move forward after turning
                        
                    #     self.vel.linear.x = 0.1
                    #     self.pub.publish(self.vel)
                    #     continue
                    
                        
                else:
                    self.vel = Twist()
                    self.vel.linear.x = 0.1
                    self.theta_z0 = self.theta_z
                    status = "moving forwards"
                    print(f"Front Lidar range: {self.frontobject_distance:.2f}, "f"Right Lidar range: {self.rightobject_distance:.2f}, ")
            self.pub.publish(self.vel)
            #self.print_stuff(status)
            #print(abs(self.theta_z))
            self.rate.sleep()

if __name__ == '__main__':
    maze_instance = Maze()
    try:
        maze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass