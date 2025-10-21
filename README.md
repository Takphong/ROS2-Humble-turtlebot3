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


----------------------------------------------------------------------------------------------------------


Section B: Service
-
• Task B1 (Service Server)

Create a service server node named “square_service_server” with service type “std_srvs/srv/Empty”.

    - When called, the robot should move in a square path with 0.5 meter per side using the velocity commands.
    - After finishing, the robot should stop

• Task B2(Service Client)

Create a client node “square_service_client” to call your created service.


----------------------------------------------------------------------------------------------------------

Section C: Action
-
• Task C1(Action Server)

Create an action server node named “rotate_action_server” using a custom action definition “Rotate.action” :
<img width="720" height="225" alt="image" src="https://github.com/user-attachments/assets/c263e208-3f54-4253-8d0a-53c144d095ab" />


The server should:

- Rotate the Turtlebot3 in place by publishing “/cmd_vel”.

- Track the remaining angle of the robot for calculating the proper velocity through the simple P controller by the following concept:

<img width="833" height="183" alt="image" src="https://github.com/user-attachments/assets/a665ad59-11ac-42af-91be-046e664c28ec" />


    • Publish feedback every 0.1 second (10 Hz).
    • Stop and succeed when finished.


• Task C2(Action Client)

Create a client node “rotate_action_client” that:

    • Sends a goal angle (e.g., +3.14 radians or 180 degrees)
    • Prints feedback (remaining angle).
    • Prints the result when done: “Goal reached successfully” or “Goal aborted”.

    

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Command to run program in this file:
-

open gazebo(turtlebot3):

    ros2 launch midterm empty_world.launch.py


Section A:

    #circle py
    ros2 run midterm circle_publisher.py

    #logger
    ros2 run midterm odom_logger.py


Section B:

    #square server
    ros2 run midterm square_service_server.py

    #square client
    ros2 run midterm square_service_client.py


Section C:

    #rotate action server
    ros2 run midterm rotate_action_server.py

    #rotate action client
    ros2 run midterm rotate_action_client.py


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
