#!/usr/bin/env python3
# Importing the necessary libraries
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import numpy as np
from collections import defaultdict
from geometry_msgs.msg import Twist
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
import time

# Defining the class CameraSubscriber
class CameraSubscriber(Node):
    def __init__(self):
        # Initializing the CameraSubscriber node.
        super().__init__('camera_subscriber')
        # Creating a subscription to the camera image topic.
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data
        )
        # Initializing variables for the program.
        self.bridge = CvBridge() # Creating a CvBridge object
        self.horizon_line_y = None # Checking if the horizon line has been detected 
        self.line_drawn = False # Flag for checking if the horizon line has been drawn
        self.prev_gray = None  # Storing previous frame in grayscale
        self.stored_intersections = [] # Creating a list to store the intersections 
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Creating a publisher to publish the velocity.
        self.stop_publisher = self.create_publisher(Bool, '/stop', 10) # Creating a publisher to publish the stop message.
        package_share_directory = get_package_share_directory('enpm673_final_proj') # Getting the directory of the package.
        cascade_path = os.path.join(package_share_directory, 'stop_sign_classifier_2.xml') # Getting the path of the cascade classifier.
        self.stop_sign_cascade = cv2.CascadeClassifier(cascade_path) # Creating a cascade classifier object.
        # Checking if the cascade classifier is empty.
        if self.stop_sign_cascade.empty():
            self.get_logger().error('Failed to load cascade classifier from {}'.format(cascade_path))
            raise Exception('Failed to load cascade classifier')
    
    # Function for publishing the velocity.
    def publish_constant_velocity(self, linear_speed, angular_speed):
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed  # Adjust this value to set your desired speed
        vel_msg.angular.z = angular_speed  # No angular movement
        self.velocity_publisher.publish(vel_msg)

    # Function for finding the intersections of the lines.
    def find_intersections(self, lines):
        intersections = []
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1 = lines[i][0]
                line2 = lines[j][0]
                theta1, rho1 = line1[1], line1[0]
                theta2, rho2 = line2[1], line2[0]
                A = np.array([
                    [np.cos(theta1), np.sin(theta1)],
                    [np.cos(theta2), np.sin(theta2)]
                ])
                b = np.array([[rho1], [rho2]])
                try:
                    intersection = np.linalg.solve(A, b)
                    intersections.append((int(intersection[0]), int(intersection[1])))
                except np.linalg.LinAlgError:
                    continue  # No solution, lines are parallel
        return intersections
    
    # Function for computing the horizon line.
    def compute_horizon_line(self, intersections, cv_image):
        y_coords = [y for _, y in intersections]
        if y_coords:
            y_hist = defaultdict(int)
            for y in y_coords:
                y_hist[y] += 1
            most_frequent_y = max(y_hist, key=y_hist.get)
            self.horizon_line_y = most_frequent_y
            self.stored_intersections = intersections

    # Function for drawing the horizon line.
    def draw_horizon_line(self, cv_image):
        if self.horizon_line_y is not None:
            self.line_drawn = True
            cv2.line(cv_image, (0, self.horizon_line_y), (cv_image.shape[1], self.horizon_line_y), (255, 0, 0), 2)

    # Callback function for the subscriber.
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Scaling down the image to speed up processing.
        scale_percent = 50 
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        gray_real = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Checking if the horizon line has been drawn and drawing the horizon line.
        if not self.line_drawn:
            print('Drawing horizon line')
            # Applying Gaussian blur to the image.
            blurred = cv2.GaussianBlur(gray, (3, 3), 0)
            # Using the Canny edge detection algorithm to detect the edges.
            edges = cv2.Canny(blurred, 75, 150)
            # Using the Hough line transform to detect the lines in the image.
            lines = cv2.HoughLines(edges, 0.85 , np.pi/180, 50, min_theta= (-np.pi/3), max_theta= (np.pi/3))

            # Checking if the lines are not None and the horizon line has not been detected.
            if lines is not None and self.horizon_line_y is None:
                n_line = []
                for line in lines:
                    rho, theta = line[0]
                    if abs(theta - np.pi/90) < np.pi/9:
                        continue
                    n_line.append(line)
                intersections = self.find_intersections(n_line)
                scaled_intersections = [(int(x * 100 / scale_percent), int(y * 100 / scale_percent)) for x, y in intersections]
                self.compute_horizon_line(scaled_intersections, cv_image)
        
        # Checking if the horizon line has been drawn and drawing the stop sign.
        if self.line_drawn:
            # Detecting the stop sign in the image using the cascade classifier.
            stop_signs = self.stop_sign_cascade.detectMultiScale(gray_real, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            # Checking if length of the stop signs is greater than 0 and drawing a bounding box around the stop sign.
            if len(stop_signs) > 0:
                stop_message = Bool(data=True)
                self.stop_publisher.publish(stop_message)
                for (x, y, w, h) in stop_signs:
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            else:
                stop_message = Bool(data=False)
                self.stop_publisher.publish(stop_message)
            # Drawing the intersections on the image.    
            for x, y in self.stored_intersections:
                x = int(x)
                y = int(y)
                try:
                    cv2.circle(cv_image, (x, y), 2, (0, 0, 255), -1)
                except:
                    print('Error drawing circle')
        self.draw_horizon_line(cv_image)
            
    
        # Checking if the horizon line has been drawn and detecting the motion in the image.
        if self.line_drawn:
            # Checking if the previous frame is available.
            if self.prev_gray is not None:
                # Computing the optical flow using the Farneback method
                flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                # Extracting the x and y components of the flow vector.
                motion_x, motion_y = flow[..., 0], flow[..., 1]
                # Masking the motion in the x direction.
                motion_mask = np.abs(motion_x) > 15
                # Converting the motion mask into an 8-bit image.
                motion_mask = motion_mask.astype(np.uint8) * 255
                
                # Applying a threshold to the motion mask.
                _, thresh = cv2.threshold(motion_mask, 60, 255, cv2.THRESH_BINARY)
                # Finding the contours in the thresholded image.
                contours, _ = cv2.findContours(thresh.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    # Scale contour bounding box back to original image size
                    x, y, w, h = x * 100 // scale_percent, y * 100 // scale_percent, w * 100 // scale_percent, h * 100 // scale_percent

                    # Check if the contour is below the horizon line
                    if y + h/2 >= self.horizon_line_y:
                        # Checking if the contour is large enough and stopping the robot.
                        if cv2.contourArea(contour) > 100:  
                            stop_message = Bool(data=True)
                            self.stop_publisher.publish(stop_message)
                            self.publish_constant_velocity(0.0, 0.0)
                            print(cv2.contourArea(contour))
                            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        elif len(stop_signs) == 0:
                            stop_message = Bool(data=False)
                            self.stop_publisher.publish(stop_message)
            self.prev_gray = gray
        cv2.imshow('Camera Image', cv_image)
        cv2.waitKey(1)

# Main function for initializing the node.
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
