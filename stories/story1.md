# SketchiBot: Current Progress
#### Comp Robo Fall 2015
#### Alex Crease, Jay Woo, Kai Levy

Hey all,
We are team Sketchibot, a team of three working with Neato's for our Computation Robotics class, aiming to create a drawing robot!

## The Goal
Simply put, our goal is to get the Neato to draw images inspired by visually-received, text-based prompts. That is, the robot will see a written prompt on it's camera. It will interpret that prompt and scour the web for an appropriate image, which it will then draw by moving around on a large piece of paper.

## Project Workflow

![Our ideal project workflow](../images/project_workflow.png)

## Progress So Far

### The Marker
In order to draw with our Neato, we needed to attach some drawing device. We went about this by first drilling a hole through the center of our devoted Neato, dubbed Sketchibot.

<img src="../images/surgery.jpg" alt="Neato Surgery!" width="450">

Then, we attached a marker to a 3-D printed part that Alex designed, which mounted to a Servo that is wired into the Raspberry Pi that controls the Neato.

<!-- TODO: @Alex: Put a render of the CAD, or a good pic of the attached marker? -->

At this point, with the help of Paul, we interfaced into the Raspberry Pi and created a file that would control the Servo with PWM, adapted from [here](http://www.toptechboy.com/raspberry-pi/raspberry-pi-lesson-28-controlling-a-servo-on-raspberry-pi-with-python/). This script also included a simple server that we would be able to connect to from our computers. With that, we were able create a ROS node that would connect to the Pi server upon connecting to a Neato, and would continually listen to the ROS topic, `/servo_command`, causing the marker to actuate up and down. It was successful, and we were able to draw things by manually driving the robot and the marker!

![Our first drawing!](../images/first_drawing.jpg)

### Path Planning
Once we had an actuating marker, we started tackling the process of planning a path for the Neato to draw things. Our first test was the Neato drawing a square, simply adapting an odometry-based square-driving program that we had made for a previous assignment. Click the image below to check it out!

[![Click me!](http://img.youtube.com/vi/3kpEW2YwMAc/0.jpg)](https://www.youtube.com/watch?v=3kpEW2YwMAc)

That worked, but it wasn't very robust or particularly precise. So we decided to examine techniques with setting waypoints for the robot via ROS's built-in navigation stack, using a map to keep track of it's own position. By using the 'move_base' node, it is possible to publish goals for the robot to travel to, and the node will automatically publish commands to /cmd_vel to drive the Neato along path to the waypoint. Theoretically, the points of the image can be passed to the goal topic in the 'move_base' node, and the Neato will try its best to go to each of the individual points.

However, the results, which can be viewed below, were less than stellar. Although the Neato reaches all of its individual goals with little error, the path-planning algorithm is sub-optimal. The overshoot is compensated for by turning the Neato around a full 180 degrees in order to get back to the waypoint location, producing much uglier images.

[![Click me!](http://img.youtube.com/vi/yITSF0bupNE/0.jpg)](https://www.youtube.com/watch?v=yITSF0bupNE)

Given this shortcoming, we've looked into more robust solutions. Using an IMU, the odometry data can be vastly improved to account for wheel slippages. By Kalman filtering the IMU data, the heading can be approximated much more accurately, which could really help with pointing the robot towards the next waypoint location. As for getting the position of the Neato, we could continue using our SLAM and particle filter code and possibly fuse that data with the IMU-corrected odometry readings to improve location tracking.

### Command Planning and Virtual Interface
<!-- TODO: @Alex: Fill this in? I don't really know how to talk about it -->


### Edge Detection and Contour Finding
We played around a bit with ways to convert our target images into drawable commands, and decided to try using Canny Edge Detection to find the most important edges to draw in order to get a good rendering of the image. This operation is built into OpenCV, and was fairly straightforward to get start on. Once we grey-scaled our target image, we simply ran the operation in order to get our a binarized image of the edges.

![Cow edges!](../images/cow_edges.png)

From here, we tried using OpenCV's contour finding to break these edges into strokes. After playing with the parameters, we got some pretty good results on our trial images!

![Contour plotting of a cow!](../images/cow_gif.gif)

The resultant data structure came out as a list of lists, where each inner list is a contour, and each contour a list of points. In essence, a collection of strokes in 'connect-the-dots' form. We noticed however that the strokes weren't ordered very logically, and would often jump around, which would be very inefficient for our robot. Additionally, a few times multiple strokes covered the same edges on the image-- we plan on exploring methods of pre-filtering to cut down on these inefficiencies.

## Decisions Going Forward
<!-- TODO: @Everyone: Do this -->
