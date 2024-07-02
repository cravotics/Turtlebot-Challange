#!/usr/bin/env python3
# Importing the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

# Defining the Class PaperFollowerNode
class PaperFollowerNode(Node):
    def __init__(self):
        # Intializing the paper follower node
        super().__init__('paper_follower_node')
        # Creating a subscription for getting the camera feed.
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        # Initializing the variables.
        self.switch = 0 # Initializing the switch variable. 
        self.stop_l =0  # Initializing the stop_l variable.
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Creating a publisher to publish the velocity commands.
        self.stop_subscriber = self.create_subscription(Bool, '/stop', self.stop_callback, 10) # Creating a subscriber to subscribe to the stop signal.
        self.processed_image_publisher = self.create_publisher(Image, '/processed_image', 10) # Creating a publisher to publish the processed image.
        self.bridge = CvBridge() # Creating a CvBridge object.
        self.right_start_time = 0 # Initializing the right start time.
        self.fov_horizontal = 62.2  # Setting the horizontal field of view of the camera to 62.2 degrees.
        self.state = "searching"  # Initializing the state for searching.
        self.contour_areas = []  # Initializing a list to store contour areas.
        self.no_paper= 0 # Initializing the no paper variable.
        self.stop = 0  # Initializing the stop variable.
    
    # Callback function for stopping the robot.
    def stop_callback(self, msg):
        # If the stop signal is received, stopping the robot.
        if msg.data: 
            self.get_logger().info('Stop signal received, stopping the robot.')
            twist = Twist()
            twist.linear.x = 0.0  # Setting the linear velocity to 0
            twist.angular.z = 0.0  # Setting the angular velocity to 0
            self.publisher.publish(twist)
            self.stop = 1
        else:
            self.get_logger().info('Continue signal received, resuming normal operation.')
            self.stop = 0

    # Function for finding and drawing the paper in the image.
    def find_and_draw_paper(self, image):
        # Cropping the image for getting the bottom half of the image.
        height = image.shape[0]
        cropped_image = image[height // 2:height, :]  
        # Converting the cropped image to HSV color space.
        hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

        # Defining range for white color
        lower_white = np.array([0, 0, 168], dtype=np.uint8)
        upper_white = np.array([172, 111, 255], dtype=np.uint8)

        # Applying threshold to the HSV image to perform the white color mask and morphological operations.
        mask = cv2.inRange(hsv, lower_white, upper_white)
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Finding contours in the mask.
        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, image, 0  

        # Getting the largest contour .
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculating the area of the largest contour.
        contour_area = cv2.contourArea(largest_contour)
        if contour_area == None:
            contour_area = 0
        
        # Calculating the centroid of the largest contour.
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            return None, None, image, 0

        # Drawing the largest contour and the centroid to the image.
        cv2.drawContours(cropped_image, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(cropped_image, (cX, cY), 5, (255, 0, 0), -1)
        image[height // 2:height, :] = cropped_image

        return cX, image.shape[1], image,contour_area

    # Function for calculating the angle required to turn based on the centroid of the paper.
    def calculate_turn_angle(self, cX, image_width):
        # Calculating the horizontal field of view in radians.
        fov_rad = math.radians(self.fov_horizontal)

        # Calculating the offset from the center of the image.
        offset_from_center = cX - (image_width / 2)

        # Calculating the angle as a proportion to the field of view.
        angle_to_turn = (offset_from_center / (image_width / 2)) * (self.fov_horizontal / 2)

        # Determining the direction to turn the robot towards right.
        direction = "right" if offset_from_center > 0 else "left"

        return angle_to_turn, direction
    
    # Function for searching for the paper.
    def search_for_paper(self):
        # Turning the robot left and right to search for paper.
        twist = Twist()
        if self.state == "searching":
            twist.angular.z = 0.3  # Turning left
            twist.linear.x = 0.0
            self.state = "turning_left"
            self.publisher.publish(twist)

        elif self.state == "turning_left":
            twist.angular.z = -0.3  # Turning right
            twist.linear.x = 0.0
            self.state = "turning_right"
            self.publisher.publish(twist)
        elif self.state == "turning_right":
            self.state = "returning"  # Returning to home if paper is not found
        self.publisher.publish(twist)
    
    # Function for returning to the home position.
    def go_to_home_position(self):
        twist = Twist()
        twist.angular.z = 0.0
        self.publisher.publish(twist)
    # Function for publishing the processed image.
    def publish_processed_image(self, processed_image):
        processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
        self.processed_image_publisher.publish(processed_image_msg)
    
    # Callback function for the camera image.
    def image_callback(self, msg):
        # Converting the ROS image message to OpenCV image.
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cX, image_width, processed_image, con_area = self.find_and_draw_paper(cv_image)
        self.contour_areas.append(con_area)
        
        # Checking if the contour_areas list has more than 25 elements and removing the extra elements.
        if len(self.contour_areas) > 25:
                self.contour_areas.pop(0)

        # Calculating the average of contour area.
        avg_contour_area = sum(self.contour_areas) / len(self.contour_areas)
        if avg_contour_area <3000:
            self.switch =1
        else:
            self.switch = 0
        twist = Twist()

        # Checking if the stop signal is received and calculating the angle to turn the robot.
        if not self.stop:
            if cX is not None and self.switch ==0:
                self.no_paper= 0
                angle, direction = self.calculate_turn_angle(cX, image_width)
                self.state = "driving" # Setting the state to driving.
                # Calculating the angular velocity and linear velocity based on the angle.
                angular_z = -angle * math.pi / 180  
                twist.angular.z = angular_z
                twist.linear.x = 0.1 
                self.publisher.publish(twist)
            else:
                self.get_logger().info('No paper detected, searching...')
                
                # Checking if the robot is turning left and whether the paper is detected.
                if self.state != "turning_left" and self.no_paper==0:
                    self.left_start_time = time.time()
                    self.get_logger().info(f'started turning at {self.left_start_time} seconds.')
                    self.state = "turning_left"
                    self.no_paper=1
                    
                if time.time() - self.left_start_time <= 5.5 and self.no_paper==1:
                    twist.angular.z = 0.5 

                elif time.time() - self.left_start_time > 5.5 and self.stop_l == 0:
                    self.state = "searching"  # Setting the state to searching.
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.publisher.publish(twist)
                    self.stop_l = 1
                    self.right_start_time = time.time()
                
                if time.time()- self.right_start_time <=5.5 and self.stop_l == 1:
                    twist.angular.z = -0.5
                
                elif time.time() - self.right_start_time > 5.5 and self.stop_l == 1:
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.publisher.publish(twist)
                    self.stop_l = 0                

                # Checking if the state is returning and going to the home position.
                if self.state == "returning":
                    self.go_to_home_position()
                else:
                    self.publisher.publish(twist)
                self.state = "turning" # Setting the state to turning.
                if self.state == "returning":
                    self.go_to_home_position()
                    self.switch = 0

            # Publishing the image processed for visualization
            self.publish_processed_image(processed_image)

# Main function to initialize the node and spin the node.
def main(args=None):
    rclpy.init(args=args)
    paper_follower_node = PaperFollowerNode()
    rclpy.spin(paper_follower_node)
    paper_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

