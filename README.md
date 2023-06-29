# SLAM with Gmapping and Moving Base
Welcome to the SLAM with Gmapping and Moving Base project! This repository contains a simulation of a robot using ROS (Robot Operating System) with advanced capabilities like simultaneous localization and mapping (SLAM) and autonomous movement. The robot is equipped with laser scanners for obstacle detection and can be controlled through various interfaces, including user input, keyboard control, and driving assistance.
<br>

Control of a robot in a simulated environment.


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#Introduction">Introduction</a></li>
    <li><a href="#Installation_and_Running">Installation_and_Running</a></li>
    <li><a href="#Fauctionality">Fauctionality</a></li>
    <li><a href="#How_it_works">How_it_works</a></li>
    <li><a href="#PseudoCode">PseudoCode</a></li>
    <li><a href="#Simulation_and_Results">Simulation_and_Results</a></li>
    <li><a href="#Improving_Robot_Movement">Improving_Robot_Movement</a></li>
  </ol>
</details>


## Introduction

The goal of this project is to develop an autonomous robot capable of navigating through a predefined map using SLAM techniques. The robot's movements can be controlled in different ways to facilitate easy interaction and testing. This README will guide you through the installation process and explain how to use the different functionalities provided.<br>


<!-- In this project, it is supposed to move and control this robot autonomously without any collision with the walls surrounding the whole complex path. Meanwhile, it is also essential to have a simple user interface to communicate with the robot in the case of increasing/decreasing the speed, resetting the robot position, etc. In addition, it is worth mentioning that all requirements should not be considered in more than two nodes.
 -->
This program manages a robot, endowed with laser scanners, which should move autonomously inside a map.<br>
You can use the user interface to:
<ol>
    <li>Let the robot autonomously reach a x,y coordinate inserted by the command line.</li>
    <li>Drive the robot with the keyboard.</li>
    <li>Drive the robot with the keyboard availing of simple driving assistance.</li>
</ol>

The map is this one:<br>
### Gazebo:
<p align="center">
<img src="https://user-images.githubusercontent.com/94115975/221689972-e9425967-7ca6-4488-aab5-f66a1d31d0a5.png" width="900" height="500">
</p>

### Rviz:

<p align="center">
<img src="https://user-images.githubusercontent.com/94115975/221690077-f9f397e6-4ba1-4c01-b25a-99ee7c71c638.png" width="900" height="500">
</p>


## Installation_and_Running

To get started, you need to install the ROS Navigation package, ROS move_base package, and ROS Gmapping package. Run the following commands to install the required packages:

Installing the ROS Navigation package: 
<pre><code>sudo apt-get install ros-noetic-navigation</code></pre>
<pre><code>sudo apt-get install ros-noetic-move-base</code></pre>
<pre><code> sudo apt-get install ros-noetic-gmapping </code></pre>

Once the packages are installed, clone this repository into your desired location using the following command:

<pre><code>git clone https://github.com/ParinazRmp/SLAM_with_Gmapping_and_Moving_Base.git </code></pre>

Move the "final_assignment" folder into the "src" folder of your ROS workspace. Navigate to the root folder of your ROS workspace and run the command:
<pre><code>catkin_make</code></pre>

Make sure you have the xterm tool installed on your system. If not, install it using the command:

<pre><code>sudo apt install xterm</code></pre>

Now you're ready to launch the simulation. Open a terminal and run the following command:

<pre><code>roslaunch final_assignment final_assignment.launch</code></pre>


## Fauctionality

The robot simulation provides the following functionalities:

1. Autonomous Navigation: You can command the robot to autonomously navigate to specific x,y coordinates by providing them through the command line.

2. Keyboard Control: You can manually control the robot's movement using keyboard inputs. A list of available commands will be displayed in the console.

3. Driving Assistance: The robot can be driven with the keyboard while benefiting from a driving assistance feature. The assistance system uses laser scanner data to detect obstacles and helps prevent collisions.

## How_it_works

The simulation uses the "simulation_gmapping.launch" file to run the simulated environment and the "move_base.launch" file to execute the move_base action. The move_base action provides topics such as 
<ul>
    <li>move_base/goal to publish the goal position;</li>
    <li>move_base/feedback to receive the feedback;</li> 
    <li>move_base/cancel to cancel the current goal.</li>
</ul>

The simulation uses the "simulation_gmapping.launch" file to run the simulated environment and the "move_base.launch" file to execute the move_base action. The move_base action provides topics such as "move_base/goal" for publishing the goal position, "move_base/feedback" for receiving feedback, and "move_base/cancel" for canceling the current goal.

The simulation employs three subscribers that run concurrently using the ROS AsyncSpinner class, enabling a multi-threaded architecture. These subscribers are responsible for:

- Monitoring the feedback from the robot's movement and checking if the goal position has been reached.
- Tracking the current goal coordinates.
- Processing laser scanner data and providing driving assistance if enabled.
The robot can autonomously navigate to a goal position, cancel the current goal, or be manually driven using keyboard commands.


### PseudoCode
The core behavior of the program can be described using the following pseudocode:
<pre><code>
FUNCTION manualDriving
    WHILE user input is not to quit
        TAKE user input through the keyboard
        EXECUTE corresponding task
        PUBLISH new robot velocity
    END WHILE
END FUNCTION

FUNCTION drivingAssistance WITH (msg)
    COMPUTE minimum distance on the right
    COMPUTE minimum distance in the middle
    COMPUTE minimum distance on the left
    
    IF driving assistance is enabled AND the robot is going against a wall THEN
        SET robot velocity TO 0
        PUBLISH robot velocity
    END IF

    IF a goal position is set THEN
        COMPUTE the time elapsed
        IF the time elapsed IS GREATER THAN 120 seconds THEN
            DELETE current goal
        END IF
    END IF
END FUNCTION

FUNCTION currentStatus WITH (msg) 
    SET current robot position
    COMPUTE the difference between the current robot position and the current goal position
    IF the difference IS LESS THAN 0.5 THEN
        STOP to compute the elapsed time
    END IF
END FUNCTION

FUNCTION currentGoal WITH (msg)
    SET current goal position
END FUNCTION

FUNCTION userInterface 
    WHILE user input is not to quit
        TAKE user input through the keyboard
        EXECUTE corresponding task
    END WHILE
END FUNCTION

FUNCTION main WITH (argc, argv)
    INITIALIZE the node "final_robot"

    SET the first publisher TO "move_base/goal"
    SET the second publisher TO "move_base/cancel"
    SET the third publisher TO "cmd_vel"

    SET the first subscriber TO "/move_base/feedback" WITH currentStatus
    SET the second subscriber TO "/move_base/goal" WITH currentGoal
    SET the third subscriber TO "/scan" WITH drivingAssistance

    INITIALIZE spinner WITH 3 threads
    START spinner
    CALL userInterface
    STOP spinner
    CALL ros::shutdown
    CALL ros::waitForShutdown

    RETURN 0
END FUNCTION

</code></pre>

## Simulation_and_Results

To see the simulation in action and review the results, please watch the following video:
https://user-images.githubusercontent.com/94115975/226293056-0559827e-3d23-459a-bca2-ee245631cdf8.mp4

![rosgraph](https://user-images.githubusercontent.com/94115975/221713615-f280fb6c-fa7d-4b0c-bf68-9f28d757a033.png)


## Robot_Movement_Improvement

One possible improvement for the driving assistance feature is to implement a smart algorithm that guides the robot in the right direction when it encounters an obstacle, instead of merely stopping it. This enhancement would improve the user experience and make the driving assistance more intuitive and efficient.

Feel free to explore and modify the code to implement additional improvements or functionalities based on your specific requirements and use cases.

