#!/usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback
from tf.transformations import euler_from_quaternion

class action_client(object):

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


   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            # print(f"FEEDBACK: Currently travelled {self.distance:.3f} m")

    def __init__(self):
        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)
        self.goal = SearchGoal()
        self.client = actionlib.SimpleActionClient("/search_action_server", 
                    SearchAction)
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdown_ops)
        self.distance = 0.0
        self.i = 0

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach

        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(velocity = 0.25, approach = 0.7)
        prempt = False
        while self.client.get_state() < 2:
            if self.distance >= 10:

                rospy.logwarn("Cancelling goal now...")
                self.client.cancel_goal()
                rospy.logwarn("Goal Cancelled")
                prempt = True
                break

            self.rate.sleep()
        
        self.action_complete = True
        print(f"Task Completed")
     

if __name__ == '__main__':
    client_instance = action_client()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass