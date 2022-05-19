#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
from operator import le
import rospy
import roslaunch
        # import rospy
import actionlib
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# from nav_msgs.msg import OccupancyGrid, Path


from tb3 import Tb3Odometry

import numpy as np
from sensor_msgs.msg import Image

cvbridge_interface = CvBridge()
base_image_path = Path("/home/student/snaps")
base_image_path.mkdir(parents=True, exist_ok=True)

class Task3(object):
    feedback = SearchFeedback() 
    result = SearchResult()
   
  
  
    


    def __init__(self):
        rospy.on_shutdown(self.shutdownhook) 
        self.node_name = "/task3_action_server"
        self.actionserver = actionlib.SimpleActionServer(self.node_name, SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        
      
        


        self.ctrl_c = False

        self.tb3_odom = Tb3Odometry()
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) 
        self.lidar = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        
        
        # self.map = OccupancyGrid()
        # self.sub_map = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        self.vel = Twist()

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
            
            
            
            # elif front_has_spaces and front_right_has_spaces and not right_has_spaces:
            #     current_move = 'turn right'
            #     print("moved straight as there's enough space on right side")
            # elif front_has_spaces and not front_right_has_spaces and not right_has_spaces:
            #     current_move = 'go straight'
            #     print("turned left as there's very little space on right side")
            # elif not front_has_spaces and not front_right_has_spaces and not right_has_spaces:
            #     current_move = 'turn left'
            #     print("turned right as there's no space on front and right side")
            # elif front_has_spaces and front_left_has_spaces and left_has_spaces:
            #     current_move = 'turn right'
            #     print("moved left as there's lots of space on front and left side")
            # elif front_has_spaces and front_left_has_spaces and not left_has_spaces:
            #     current_move = 'turn right'
            #     print("moved straight as there's enough space on left side")
            # elif front_has_spaces and not front_left_has_spaces and not left_has_spaces:
            #     current_move = 'go straight'
            #     print("turned right as there's very little space on left side")
            # elif not front_has_spaces and not front_left_has_spaces and not left_has_spaces:
            #     current_move = 'turn right'
            #     print("turned left as there's no space on front and left side")
            # return
        
    def camera_callback(self,msg):
        
            cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)# turn bgr image to hsv image for detection 
            #Thresholds for ["Blue", "Red", "Green", "Turquoise","Yellow","Purple"]
            self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 106), (75, 150, 100),(30, 200, 100),(140, 200, 100)]
            self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255),(45, 255, 255),(155, 200, 100)]

            for i in range(6):
                if i == 0:
                    blue_mask= cv2.inRange(hsv, numpy.array([115, 224, 100]),numpy.array([130, 255, 255]))
                    self.blue_m = cv2.moments(blue_mask)
                elif i==1:
                    red_mask = cv2.inRange(hsv, numpy.array([0, 185, 100]),numpy.array([10, 255, 255]))
                    self.red_m = cv2.moments(red_mask)
                elif i==2:
                    green_mask = cv2.inRange(hsv, numpy.array([25, 150, 106]),numpy.array([70, 255, 255]))
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


            self.blue_cy = int(self.blue_m['m10']/(self.blue_m['m00']+1e-5))
            self.red_cy =int(self.red_m['m10']/(self.red_m['m00']+1e-5))
            self.green_cy = int(self.green_m['m10']/(self.green_m['m00']+1e-5))
            self.turquoise_cy = int(self.turquoise_m['m10']/(self.turquoise_m['m00']+1e-5))
            self.yellow_cy = int(self.yellow_m['m10']/(self.yellow_m['m00']+1e-5))
            self.purple_cy = int(self.purple_m['m10']/(self.purple_m['m00']+1e-5))
            

         

    def shutdownhook(self):
            # print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
            self.ctrl_c = True
            
        
    def action_server_launcher(self, goal):
            r = rospy.Rate(10)
            success = True

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
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.vel_pub.publish(self.vel)
            
            if success:
                rospy.loginfo('Robot sucessfully navigated the maze.')
                # self.result.image_path = self.base_image_path
                self.actionserver.set_succeeded(self.result)
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.vel_pub.publish(self.vel)

def show_and_save_image(img, img_name):
            full_image_path = base_image_path.joinpath(f"the_beacon.jpg")

            cv2.imshow(img_name, img)
            cv2.waitKey(0)

            cv2.imwrite(str(full_image_path), img)
            print(f"Saved an image to '{full_image_path}'\n"
                f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
                f"file size = {full_image_path.stat().st_size} bytes")
        

        


       
    # def map_callback(self,data):
    #     valid=False
    #     self.map = data
    #     map_size=0
    #     while valid is False:
    #         map_size = randrange(len(data.data))
    #         map_cell_value=data.data[map_size]

    
    
def find_colour(self, img_data):
            global waiting_for_image  
            try:
                cv_img = cvbridge_interface .imgmsg_to_cv2(img_data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
                
                
            
            if self.blue_cy !=0 or self.red_cy !=0 or self.green_cy !=0 or self.yellow_cy !=0:
                height, width, channels = cv_img.shape

                print(f"Obtained an image of height {height}px and width {width}px.")

            show_and_save_image(cv_img, img_name = "the_beacon")

rospy.Subscriber("/camera/rgb/image_raw", Image, find_colour)


       

if __name__ == '__main__':
    rospy.init_node('task3_action_server')
    Task3()
    rospy.spin()
    
    # map_path = " $(find team52)/maps/task5_map"

    # rospy.init_node("map_getter", anonymous=True)

    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()

    # print(f"Saving map at time: {rospy.get_time()}...")
    # node = roslaunch.core.Node(package="map_server",
    #                                 node_type="map_saver",
    #                                 args=f"-f {map_path}")
    # process = launch.launch(node)


