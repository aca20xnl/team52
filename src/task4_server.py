#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib


from com2009_msgs.msg import CameraSweepFeedback, CameraSweepAction, CameraSweepResult

# Import some other modules from within this package
from task4_tb3 import Tb3Move, Tb3Odometry




class Server(object):

    feedback = CameraSweepFeedback() 
    result = CameraSweepResult()

    def __init__(self):

        self.actionserver = actionlib.SimpleActionServer("/task4_server", CameraSweepAction, self.action_server_launcher, auto_start=False)
        self.init = True
        self.vel_controller = Tb3Move()
        self.pos = Tb3Odometry()

       
        self.actionserver.start()
        

    

    def turn(self,ideal_angle):
        self.diff=ideal_angle-(self.pos.yaw+180)
        self.turn_left=False
        self.turn_right=False

        if self.diff <-180:
            self.diff = self.diff+360

        elif self.diff >180:
            self.diff = self.diff-360
        
        vel= self.diff/10
        if vel>0.7:
            self.turn_left=True
        elif vel<-0.7:
            self.turn_right=True

        if self.turn_left:
            vel=0.7
        elif self.turn_right:
            vel=-0.7
            
            

        
        return vel

        
    def action_server_launcher(self, goal):
        if self.init:
            angle=self.pos.yaw
            self.feedback.current_angle = angle+180
            self.init = False
        else:
            
            vel_z = self.turn(goal.sweep_angle)
            #only change velocity of angular z
            if (goal.image_count==0):
                self.vel_controller.set_move_cmd(0,vel_z)
            #change velocity of angular z and velocity of linear x
            elif (goal.image_count==10):
                self.vel_controller.set_move_cmd(0.17,vel_z)
            #change velocity of linear x
            elif (goal.image_count==102):
                self.vel_controller.set_move_cmd(0.15,0)
            #stop robot
            elif(goal.image_count==100):
                self.vel_controller.set_move_cmd(0,0)
          
            self.vel_controller.publish()
            angle=self.pos.yaw
            self.feedback.current_angle = angle+180

            
        self.actionserver.publish_feedback(self.feedback)
        self.actionserver.set_succeeded(self.result)
        
          
       

if __name__ == '__main__':
    rospy.init_node('task4_server')
    Server()
    rospy.spin()
