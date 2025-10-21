-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
ROS2 Fundamental Concepts
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
This project is a ROS2 exam that tests core concepts using a simulated TurtleBot3 robot in Gazebo. The exam has three parts, each focusing on a key communication method (Publisher/Subscriber, Service, and Action) and requiring you to build ROS2 nodes.


Part A (Publisher/Subscriber): 
A circle_publisher node sends velocity commands to make the robot drive in a circle, while an odom_logger node subscribes to and logs the robot's position data.


Part B (Service):
A square_service_server makes the robot move in a square when called by a matching square_service_client.


Part C (Action):
A rotate_action_server rotates the robot by a set angle and provides feedback during the rotation. A rotate_action_client sends the goal and monitors its progress.


All components are built within a single ROS2 package to demonstrate the practical use of topics, services, and actions.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Section A: Publisher & Subscriber
-
• Task A1(Publisher)

Create a node name “circle_publisher” that continuously publishes desire

velocity commands (geometry_msgs/msg/Twist) to make Turtlebot3 move in a circle of radius ~0.5 meters.


• Task A2(Subscriber)

Create a node name “odom_logger” that subscribes to “/odom” and prints:

    - Robot’s x, y position.
  
    - Robot’s orientation (yaw angle)


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#open gazebo(turtlebot3)

ros2 launch midterm empty_world.launch.py


#circle py

ros2 run midterm circle_publisher.py


#logger

ros2 run midterm odom_logger.py


#square server

ros2 run midterm square_service_server.py


#square client

ros2 run midterm square_service_client.py


#rotate action server

ros2 run midterm rotate_action_server.py


#rotate action client

ros2 run midterm rotate_action_client.py

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
