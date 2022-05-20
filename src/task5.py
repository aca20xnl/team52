#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
from operator import le
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


from tb3 import Tb3Odometry

import numpy as np

class Task3(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def scan_callback(self, scan_data):
        arc_front_left = scan_data.ranges[0:31]
        arc_front_right = scan_data.ranges[-30:]
        
        front_arc = np.array(arc_front_left[::-1] + arc_front_right[::-1])
        front_right_arc = np.array(scan_data.ranges[300:330])
        front_left_arc = np.array(scan_data.ranges[30:60])
        right_arc = np.array(scan_data.ranges[270:300])
        left_arc = np.array(scan_data.ranges[60:120])

        front_arc = front_arc[front_arc != 0.0]
        front_right_arc = front_right_arc[front_right_arc != 0.0]
        front_left_arc = front_left_arc[front_left_arc != 0.0]
        right_arc = right_arc[right_arc != 0.0]
        left_arc = left_arc[left_arc != 0.0]


        global arcs
        arcs = {
            'front_arc':  min(front_arc),
            'front_right_arc': min(front_right_arc),
            'front_left_arc':  min(front_left_arc ),
            'right_arc':  min(right_arc),
            'left_arc':   min(left_arc),
        }

        self.navigate()
        
    def __init__(self):

        self.node_name = "/task3_action_server"
        self.actionserver = actionlib.SimpleActionServer(self.node_name, SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.tb3_odom = Tb3Odometry()
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) 
        self.lidar = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.vel = Twist()

        rospy.on_shutdown(self.shutdownhook)
        self.ctrl_c = False

        global arcs, current_move, move_plans
        current_move = 'move towards wall'

        arcs = {            
            'front_arc': 0,
            'front_right_arc': 0,
            'front_left_arc': 0,
            'right_arc': 0,
            'left_arc': 0,
        }

        move_plans = {
            'move towards wall' : 'move towards wall',
            'turn right' : 'turn right',
            'go straight' : 'go straight',
            'turn left' : 'turn left',
        }
        
    def navigate(self):
        
        global arcs, current_move
        front_dist_from_wall = 0.4
        left_dist_from_wall = 0.36
        right_dist_from_wall = 0.36

        front_has_spaces = arcs['front_arc'] > front_dist_from_wall
        front_left_has_spaces = arcs['front_left_arc'] > left_dist_from_wall
        left_has_spaces = arcs['left_arc'] > left_dist_from_wall
        front_right_has_spaces = arcs['front_right_arc'] > right_dist_from_wall
        right_has_spaces = arcs['right_arc'] > right_dist_from_wall

        if front_has_spaces and front_left_has_spaces and left_has_spaces:
            current_move = 'turn left'
            print("moved left as there's lots of space on front and left side")
        elif front_has_spaces and front_left_has_spaces and not left_has_spaces:
            current_move = 'turn left'
            print("moved straight as there's enough space on left side")
        elif front_has_spaces and not front_left_has_spaces and not left_has_spaces:
            current_move = 'go straight'
            print("turned right as there's very little space on left side")
        elif not front_has_spaces and not front_left_has_spaces and not left_has_spaces:
            current_move = 'turn right'
            print("turned right as there's no space on front and left side")
        elif front_has_spaces and not front_right_has_spaces and not right_has_spaces:
            current_move = 'turn left'
            print("turned left as there's very little space on right side")
        return

        
        

    def action_server_launcher(self, goal):
        r = rospy.Rate(10)
        success = True
        # global current_move 

        # self.vel = Twist ()
        # self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) 



        while success and not self.ctrl_c:
            if current_move == 'move towards wall':
                self.vel.linear.x = 0.26
                self.vel.angular.z = 0.6
                self.vel_pub.publish(self.vel)
            elif current_move == 'turn right':
                self.vel.linear.x = 0.0
                self.vel.angular.z = -0.6
                self.vel_pub.publish(self.vel)
            elif current_move == 'go straight':
                self.vel.linear.x = 0.26
                self.vel.angular.z = 0.0
                self.vel_pub.publish(self.vel)
            elif current_move == 'turn left':
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.6
                self.vel_pub.publish(self.vel)
            else:
                rospy.logerr('Oh No! Undefined move.')

        r.sleep()

        if success:
            rospy.loginfo('Robot sucessfully navigated the maze.')
            self.result.image_path = self.base_image_path
            self.actionserver.set_succeeded(self.result)
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.vel_pub.publish(self.vel)
       

if __name__ == '__main__':
    rospy.init_node('task3_action_server')
    Task3()
    rospy.spin()