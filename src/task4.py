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
from math import sqrt, pow, pi#
from tf.transformations import euler_from_quaternion




class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        # The robot turns on the spot whilst obtaining images from its camera 
        # (by subscribing to the /camera/rgb/image_raw topic).  
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
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
        
        self.m00_blue = 0
        self.m00_red = 0
        self.m00_green = 0
        self.m00_turq = 0
        self.m00_min = 10000

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

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
        # for i in range(4):
        #     if i == 0:
        #         mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #     else:
        #         mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        # for i in range(2):
        #     if i == 0:
        #         mask = cv2.inRange(hsv_img, (115, 224, 100), (130, 255, 255))
        #         print("SEARCH INITIATED: The target beacon colour is blue.")
        #     # elif i == 1:
        #     #     mask = cv2.inRange(hsv_img, (25, 150, 100), (70, 255, 255))
        #     #     print("SEARCH INITIATED: The target beacon colour is green.")
        #     else:
        #         # mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #         mask = cv2.inRange(hsv_img, (25, 150, 100), (70, 255, 255))
        #         print("SEARCH INITIATED: The target beacon colour is green.")
        
        # Code for blue pillar
        lower_blue = (115, 224, 100)
        upper_blue = (130, 255, 255)
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        res_blue = cv2.bitwise_and(crop_img, crop_img, mask = mask_blue)

        m_blue = cv2.moments(mask_blue)
        self.m00_blue = m_blue['m00']
        self.cy_blue = m_blue['m10'] / (m_blue['m00'] + 1e-5)

        if self.m00_blue > self.m00_min:
            cv2.circle(crop_img, (int(self.cy_blue), 200), 10, (0, 0, 255), 2)

        # Code for red pillar
        lower_red = (0, 185, 100)
        upper_red = (10, 255, 255)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        res_red = cv2.bitwise_and(crop_img, crop_img, mask = mask_red)

        m_red = cv2.moments(mask_red)
        self.m00_red = m_red['m00']
        self.cy_red = m_red['m10'] / (m_red['m00'] + 1e-5)

        if self.m00_red > self.m00_min:
            cv2.circle(crop_img, (int(self.cy_red), 200), 10, (0, 0, 255), 2)
            
        # Code for green pillar
        lower_green = (25, 150, 100)
        upper_green = (70, 255, 255)
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)
        res_green = cv2.bitwise_and(crop_img, crop_img, mask = mask_green)

        m_green = cv2.moments(mask_green)
        self.m00_green = m_green['m00']
        self.cy_green = m_green['m10'] / (m_green['m00'] + 1e-5)

        if self.m00_green > self.m00_min:
            cv2.circle(crop_img, (int(self.cy_green), 200), 10, (0, 0, 255), 2)

        # Code for turquoise pillar
        lower_turq = (75, 150, 100)
        upper_turq = (100, 255, 255)
        mask_turq = cv2.inRange(hsv_img, lower_turq, upper_turq)
        res_turq = cv2.bitwise_and(crop_img, crop_img, mask = mask_turq)

        m_turq = cv2.moments(mask_turq)
        self.m00_turq = m_turq['m00']
        self.cy_turq = m_turq['m10'] / (m_turq['m00'] + 1e-5)

        if self.m00_turq > self.m00_min:
            cv2.circle(crop_img, (int(self.cy_turq), 200), 10, (0, 0, 255), 2)

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
        
        self.x0 = pos_x
        self.y0 = pos_y
        self.theta_z = yaw 

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            # If the robot can't see a blue pillar then it turns on the spot quickly.
            
            # Once detected, the centroid of the blue blob representing the pillar is 
            # calculated to obtain its current location in the robot's viewpoint.
            
            # As soon as the blue pillar comes into view the robot starts to turn more 
            # slowly instead.
            
            # The robot stops turning as soon as it determines that the pillar is situated 
            # directly in front of it, using the cy component of the blue blob centroid to 
            # determine this.
            
            # The robot then waits for a while and then starts to turn again.
            
            # The whole process repeats until it finds the blue pillar once again. 

            current_theta_z=self.theta_z
            self.vel = Twist()
            self.vel.linear.x = 0
            self.vel.angular.z = -1.8
            self.pub.publish(self.vel)
            # turn_right=False
            # turn_left=False
        
            if abs(current_theta_z- self.theta_z) >= pi/2:
                self.vel = Twist()
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
            
            
            # BLUE
            if self.m00_blue > self.m00_min:
                # blue blob detected
                if self.cy_blue >= 560-100 and self.cy_blue <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        print("SEARCH INITIATED: The target beacon colour is blue.")
                        while self.move_rate == 'stop':
                            self.vel.linear.x = 1
                            self.pub.publish(self.vel)
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

            # RED
            if self.m00_red > self.m00_min:
                # red blob detected
                if self.cy_red >= 560-100 and self.cy_red <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        print("SEARCH INITIATED: The target beacon colour is red.")
                        while self.move_rate == 'stop':
                            self.vel.linear.x = 1
                            self.pub.publish(self.vel)
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

            # GREEN 
            if self.m00_green > self.m00_min:
                # green blob detected
                if self.cy_green >= 560-100 and self.cy_green <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        print("SEARCH INITIATED: The target beacon colour is green.")
                        while self.move_rate == 'stop':
                            self.vel.linear.x = 1
                            self.pub.publish(self.vel)
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

            # # TURQUOISE 
            # if self.m00_turq > self.m00_min:
            #     # turquoise blob detected
            #     if self.cy_turq >= 560-100 and self.cy_turq <= 560+100:
            #         if self.move_rate == 'slow':
            #             self.move_rate = 'stop'
            #             print("SEARCH INITIATED: The target beacon colour is turquoise.")
            #             while self.move_rate == 'stop':
            #                 self.vel.linear.x = 1
            #                 self.pub.publish(self.vel)
            #     else:
            #         self.move_rate = 'slow'
            # else:
            #     self.move_rate = 'fast'


            # self.vel.linear.x = 0
            # self.vel.angular.z = 0.5
            # # self.pub.publish(self.vel)
            # duration = rospy.get_rostime()

            # while duration.secs<6:
            #     self.pub.publish(self.vel)

            # self.vel.linear.x = 0
            # self.vel.angular.z = 0
            # self.pub.publish(self.vel)


            # TURQUOISE 
            if self.m00_turq > self.m00_min:
                # turquoise blob detected
                if self.cy_turq >= 560-100 and self.cy_turq <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        print("SEARCH INITIATED: The target beacon colour is turquoise.")
                        while self.move_rate == 'stop':
                            self.vel.linear.x = 1
                            self.pub.publish(self.vel)
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

    
                
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00_turq:.0f} pixels is in view at y-position: {self.cy_turq:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop':
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            
            # self.robot_controller.set_move_cmd(0.0, 1.0)
            self.vel.linear.x = 1
            self.pub.publish(self.vel)

            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
