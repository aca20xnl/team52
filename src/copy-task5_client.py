#! /usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction, SearchGoal

class Task5Client(object):
   
    def feedback_callback(self, feedback_data):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            #print("FEEDBACK: Currently travelled {:.3f} m".format(self.distance))

    def __init__(self):
 
        self.action_complete = False
        
        rospy.init_node("task5_client")

        self.rate = rospy.Rate(1)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient("/task5_server", 
                    SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)


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
        self.send_goal(velocity = 0.1, approach = 0.5)
        prempt = False
        while self.client.get_state() < 2:
            #print("FEEDBACK: Currently travelled {:.3f} m, STATE: Current state code is {}".format(self.distance, self.client.get_state()))
            

            self.rate.sleep()
        
        self.action_complete = True
        print("RESULT: Action State = {}".format(self.client.get_state()))
        if prempt:
            print("RESULT: Action preempted after travelling 2 meters")
        else:
            result = self.client.get_result()
            print("RESULT: closest object {:.3f} m away at a location of {:.3f} degrees".format(result.closest_object_distance, result.closest_object_angle))

if __name__ == '__main__':
    ac_object = Task5Client()
    try:
        ac_object.main()
    except rospy.ROSInterruptException:
        pass