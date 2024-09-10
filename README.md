# Turtlebot-Challenge
This was a hardware project based on TurtleBot 3 using perception and computer vision concepts

## Introduction
This project involves using TurtleBot3 to navigate through a series of papers while avoiding obstacles and reacting to stop signals. We achieved this using fundamental perception techniques, which are discussed below:

1. Horizon Line Detection
2. Paper Detection
3. Stop Sign Detection
4. Optical Flow
5. Testing in Simulation and Real-World Environment

   
<div align="center">
  <img src="https://github.com/cravotics/Turtlebot-Challange/assets/90138418/b7e1a330-b372-4e93-8ec0-f2edad7a2800" alt="challenge simulation gif git - Made with Clipchamp" width="600"/>
</div>
## Horizon Line Detection
The horizon line is created through the following steps:
- Taking the image feed through the sensor.
- Applying Gaussian blur to smoothen the image.
- Applying Canny edge detection to the image feed.
- Applying Hough transform by limiting the theta range to detect only horizontal lines.
- Detecting intersections between the lines and storing them.
- Extracting the Y-coordinates of all intersection points and selecting the most frequent Y-coordinate as the horizon line.

This horizon line is used as a reference where only objects below it are detected.

## Paper Detection
- An image of a white paper is processed to detect contours.
- The largest contour is presumed to be the paper.
- The centroid of the largest contour (paper) is calculated to determine its position with respect to the camera.
- Contours and centroid are drawn on the image for visual feedback.
- The robot follows the paper, considering it as its path.

## Stop Sign Detection
- Implemented Haar Cascade object detection using a pre-trained classifier to detect stop signs.
- Loaded an XML file containing a Haar Cascade classifier.
- Processed each frame from the image feed using the classifier to detect stop signs.
- The detected object is shown as a rectangle indicating the position and size of the stop sign.
- The robot stops when a stop sign is detected in front of it.

## Optical Flow
- Used Farneback’s Method, a dense algorithm that estimates flow by taking polynomial expansion of the image signal.
- This method returns a 2-channel array where each element is a 2D-vector pointing from the first frame to the second frame.
- Set a threshold according to the magnitude of vectors to check significant motion.
- The robot stops or slows down if a moving obstacle is visible below the horizon line in its field of view.

## Challenges Faced
- False positives of random objects detected in the real-world environment.
- Tuning the threshold for optical flow was challenging.
- Issues integrating all implementations into ROS.

## Videos
- [Gazebo Simulation](https://drive.google.com/file/d/12pGSvqSViUToGuILA8yy3oCMIlkS39Y1/view?usp=sharing)
- [Hardware Implementation](https://drive.google.com/file/d/16jdQquowrQ5lzWmg7DGVOj4qLhHYky2y/view?usp=sharing)

## Conclusion
We successfully implemented fundamental perception concepts into the TurtleBot3, enabling it to navigate through a series of papers while avoiding moving obstacles and responding to stop signals.

