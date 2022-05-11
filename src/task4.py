#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from tb3 import Tb3Move

# setup a cmd_vel publisher and an odom subscriber:
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry



class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        # The robot turns on the spot whilst obtaining images from its camera 
        # (by subscribing to the /camera/rgb/image_raw topic).  
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = 0.5
        self.turn_vel_slow = 0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
        # self.turn_vel_fast_right = -0.5
        # self.turn_vel_slow_right = -0.1

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)


        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)#
        self.vel = Twist()

        self.search = False
        self.get_ready = False
        self.move_to_target = False
        self.stop = False
        self.turn = False
        self.go = False

        self.prepare = False
        
        self.m00 = 0
        self.m00_min = 10000

        self.goalc = Point()
        self.goalc.x = -1.46
        self.goalc.y = 0.23

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.startup=True

        self.hasPrint = False


        # Task 3  
        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Camera images are obtained and cropped, then a threshold is applied to the cropped 
        # images so as to detect the blue pillar in the simulated environment.
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        #  Task 3
        # create a single mask to accommodate all four dectection colours:
        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        pixel_center = hsv_img[crop_y,crop_x]
        hue_value = pixel_center[0]
       
        # hasPrint = False
        if self.m00 > self.m00_min and self.cy >= 560-100 and self.cy <= 560+100:
            if hue_value >= 115 and hue_value <= 130:
                if self.hasPrint == False:
                    print("SEARCH INITIATED: The target beacon colour is Blue.")
                    self.hasPrint = True
            elif hue_value >= 1 and hue_value <= 10:
                if self.hasPrint == False:
                    print("SEARCH INITIATED: The target beacon colour is Red.")
                    self.hasPrint = True
            elif hue_value >= 25 and hue_value <= 70:
                if self.hasPrint == False:
                    print("SEARCH INITIATED: The target beacon colour is Green.")
                    self.hasPrint = True
            if hue_value >= 75 and hue_value <= 100:
                if self.hasPrint == False:
                    print("SEARCH INITIATED: The target beacon colour is Turquoise.")
                    self.hasPrint = True
        
        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

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

    def main(self):

        while not self.ctrl_c:
            
            # get initial yaw value
            current_theta_z=self.theta_z
            self.search = False
            countc_0 = 0            
            countc_1 = 0
            countc_2 = 0
            countc_3 = 0
            countc_4 = 0

            # START ZONE C - BLUE 
            if self.x0 >= 2.0 and self.x0 <= 2.1 and self.y0 >= 1.9 and self.y0 <= 2.0:

                if abs(self.theta_z0 - current_theta_z) <= pi/2:
                    self.vel = Twist()
                    self.vel.angular.z = 0.2                   
                    self.pub.publish(self.vel)
                else:
                    self.vel = Twist()
                    self.vel.angular.z = 0.0
                    self.pub.publish(self.vel)
                    self.prepare = True

            while self.prepare:
                countc_0 += 1
                self.vel = Twist()
                self.vel.angular.z = -0.2                   
                self.pub.publish(self.vel)

                while countc_0>80000:
                    self.vel = Twist()
                    self.vel.angular.z = 0.0                   
                    self.pub.publish(self.vel)
                    self.prepare = False

                    
            
if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
