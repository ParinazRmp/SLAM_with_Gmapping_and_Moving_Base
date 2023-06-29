/**
 * Robotics control in a simulated environment
 * Robotics Engineering
 * @file final_robot.cpp
 *
 * @brief SLAM with Gmapping and Moving Base
 *
 * This file contains the implementation of a simulation of a robot using ROS (Robot Operating System)
 * with advanced capabilities like simultaneous localization and mapping (SLAM) and autonomous movement.
 * The robot is equipped with laser scanners for obstacle detection and can be controlled through various
 * interfaces, including user input, keyboard control, and driving assistance.
 *
 * @author Parinaz
 * @version 1.0
 * @date 01/06/2023
 */

#include <iostream>
#include <string>
#include <chrono>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"


// Declare the publishers
ros::Publisher pub_goal; /**< Publisher for sending goal positions */
ros::Publisher pub_canc; /**< Publisher for canceling the current goal */
ros::Publisher pub_vel;  /**< Publisher for sending robot velocities */

// Global variables
float lin_vel = 0.0; // Robot linear velocity
float ang_vel = 0.0; // Robot angular velocity
char key;           // User input from the keyboard to drive the robot

int drive_flag = 0; // Enable/Disable driving assistance
int time_flag = 0;  // Compute the time elapsed since the request of the current goal

int flag = 0;       // Just to manage printing of the option Enable/Disable driving assistance
int print_flag = 0; // Just to manage printing in drivingAssistance
int counter1 = 10;  // Just to manage printing in manualDriving
int counter2;       // Just to manage printing in userInterface

float x_goal; // Current goal coordinate x
float y_goal; // Current goal coordinate y

std::string id = ""; // Goal ID
std::chrono::high_resolution_clock::time_point t_start;
std::chrono::high_resolution_clock::time_point t_end;
 
#define DIST 0.35       // Minimum distance from the wall with the driving assistance enabled
#define POS_ERROR 0.5   // Position range error
#define MAX_TIME 120000000 // Maximum time to reach a goal (microseconds)

/**
 * @brief Provide a user interface to drive the robot autonomously.
 *        A driving assistance can be enabled (or clearly disabled).
 */
void manualDriving();

/**
 * @brief ROS Callback Function: check data from the robot's laser scanner and,
 *        if the driving assistance is enabled, help the user to not crush
 *        the robot against a wall.
 * @param msg The laser scan message
 */
void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg);

/**
 * @brief ROS Callback Function: check if the robot is on the goal position and,
 *        when there is a new goal, update the current Goal ID and save it in a
 *        global variable.
 * @param msg The action feedback message
 */
void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);

/**
 * @brief ROS Callback Function: check the current goal position and save its
 *        module coordinates x and y in two global variables.
 * @param msg The action goal message
 */
void currentGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

/**
 * @brief Provide a user interface to choose the modality to drive the robot.
 *        The options are:
 *        1. Automatic driving (insert the coordinates to reach).
 *           1.1 It is also possible to cancel the current goal.
 *        2. Manual driving (without driving assistance).
 *        3. Manual driving (with driving assistance);
 *           3.1 The driving assistance can be simply enabled or disabled.
 */
void userInterface();

/**
 * @brief Main function of the program.
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return 0 on successful execution
 */
int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "final_robot");
    ros::NodeHandle nh;

    // Define the publishers
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
    pub_canc = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Define the subscribers
    ros::Subscriber sub_pos = nh.subscribe("/move_base/feedback", 1000, currentStatus); // Current Status feedback
    ros::Subscriber sub_goal = nh.subscribe("/move_base/goal", 1000, currentGoal);     // Current Goal feedback
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, drivingAssistance);        // Laser scanner

    // Multi-threading
    ros::AsyncSpinner spinner(3);
    spinner.start();
    userInterface();
    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}

void manualDriving() {
    // Define possible commands with corresponding actions
    std::map<char, std::pair<float, float>> commandMap {
        {'w', {lin_vel, 0}},         // Go forward
        {'q', {lin_vel, ang_vel}},   // Curve left
        {'s', {-lin_vel, 0}},        // Go backward
        {'e', {lin_vel, -ang_vel}},  // Curve right
        {'a', {0, ang_vel}},         // Turn left
        {'d', {0, -ang_vel}},        // Turn right
        {'r', {0, 0}}                // Emergency stop
    };

    geometry_msgs::Twist robot_vel;
    counter2 = 10;
    key = 'e';

    // Starting message
    std::cout << "--- Manual Control ---" << std::endl;
    std::cout << "Linear velocity advised: 0.5" << std::endl;
    std::cout << "Angular velocity advised: 1.0" << std::endl << std::endl;

    while (key != 'f') {
        // Display commands list
        if (counter2 % 10 == 0) {
            std::cout << "Commands:" << std::endl
                      << "w - Go forward" << std::endl
                      << "s - Go backward" << std::endl
                      << "q - Curve left" << std::endl
                      << "e - Curve right" << std::endl
                      << "a - Turn left" << std::endl
                      << "d - Turn right" << std::endl
                      << "-----------------------------" << std::endl
                      << "z - Increase linear velocity" << std::endl
                      << "x - Decrease linear velocity" << std::endl
                      << "c - Increase angular velocity" << std::endl
                      << "v - Decrease angular velocity" << std::endl
                      << "-----------------------------" << std::endl
                      << "r - Emergency stop" << std::endl
                      << "f - Quit" << std::endl;
        }

        if (flag == 0)
            std::cout << "h - Enable driving assistance" << std::endl;
        else if (flag == 1)
            std::cout << "h - Disable driving assistance" << std::endl;

        // Take user input
        std::cout << "Command: ";
        std::cin >> key;

        print_flag = 0;

        // Check if key is in command map
        if(commandMap.count(key) > 0) {
            robot_vel.linear.x = commandMap[key].first;
            robot_vel.angular.z = commandMap[key].second;
        }
        else if (key == 'z') { // Increase linear velocity
            lin_vel = std::min(lin_vel + 0.1f, 1.0f); // limit max velocity
        }
        else if (key == 'x') { // Decrease linear velocity
            lin_vel = std::max(lin_vel - 0.1f, 0.0f);
        }
    }
}

// Add function prototypes
void findMinValues(const sensor_msgs::LaserScan::ConstPtr& msg, float& left, float& mid, float& right);
void checkMaxTime();

void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Local variables
    geometry_msgs::Twist robot_vel;
    float left = 30.0;
    float mid = 30.0;
    float right = 30.0;

    // Take the minimum values
    findMinValues(msg, left, mid, right);

    // Driving assistance
    if (drive_flag == 1 & ((mid < DIST && key == 'w') || (left < DIST && key == 'q') || (right < DIST && key == 'e'))) {
        if (print_flag == 0) {
            printf("\nThe robot is too close to the wall!\n");
            print_flag = 1;
        }
        robot_vel.linear.x = 0;
        robot_vel.angular.z = 0;
        pub_vel.publish(robot_vel);
    }

    // Check for the max time available to reach a goal point
    checkMaxTime();
}

void findMinValues(const sensor_msgs::LaserScan::ConstPtr& msg, float& left, float& mid, float& right) {
    int i;
    for (i = 0; i < 360; i++) { // On the right
        if (msg->ranges[i] < right)
            right = msg->ranges[i];
    }
    for (i = 300; i < 420; i++) { // In the middle
        if (msg->ranges[i] < mid)
            mid = msg->ranges[i];
    }
    for (i = 360; i < 720; i++) { // On the left
        if (msg->ranges[i] < left)
            left = msg->ranges[i];
    }
}

void checkMaxTime() {
    if (time_flag == 1) {
        t_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        if (time > MAX_TIME) {
            actionlib_msgs::GoalID canc_goal;
            printf("\nThe goal point can't be reached!\n");
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
            printf("Goal cancelled.\n");
            time_flag = 0;
        }
    }
}
void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
    // Take the current robot position
    float diff_x;
    float diff_y;
    float current_x = msg->feedback.base_position.pose.position.x;
    float current_y = msg->feedback.base_position.pose.position.y;


    // Take the module
    if (current_x < 0)
        current_x *= -1;
    if (current_y < 0)
        current_y *= -1;

    // Compute the error from the actual position and the goal position
    if (current_x >= x_goal)
        diff_x = current_x - x_goal;
    else
        diff_x = x_goal - current_x;
    if (current_y >= y_goal)
        diff_y = current_y - y_goal;
    else
        diff_y = y_goal - current_y;

    // The robot is on the goal position
    if (diff_x <= POS_ERROR && diff_y <= POS_ERROR)
        time_flag = 0;

    // Update the goal ID if there is a new goal
    if (id != msg->status.goal_id.id) {
        time_flag = 1;
        id = msg->status.goal_id.id;
        t_start = std::chrono::high_resolution_clock::now();
    }
}

void currentGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    x_goal = msg->goal.target_pose.pose.position.x;
    y_goal = msg->goal.target_pose.pose.position.y;

    // Take the module
    if (x_goal < 0)
        x_goal *= -1;
    if (y_goal < 0)
        y_goal *= -1;
}

void userInterface() {
    // Local variables
    move_base_msgs::MoveBaseActionGoal goal_pos;
    actionlib_msgs::GoalID canc_goal;
    std::string X, Y;
    double x, y;
    char in;

    while (in != '0') {
        // Print commands list
        if (counter1 % 10 == 0) {
            printf("\nChoose an action:\n"
                   "0 - Exit\n"
                   "1 - Insert new coordinates to reach\n"
                   "2 - Cancel the current goal\n"
                   "3 - Manual driving\n");
        }
        if (flag == 0)
            printf("4 - Enable driving assistance\n");
        else if (flag == 1)
            printf("4 - Disable driving assistance\n");

        // Take user input
        printf("Action (type the corresponding number): ");
        std::cin >> in;

        // Check input
        if (in != '0' && in != '1' && in != '2' && in != '3' && in != '4')
            printf("\nERROR: type '0', '1', '2', '3' or '4'.\n");

        counter1++;

        // Delete current goal
        if (in == '0') {
            time_flag = 0;
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
        }

        // Insert new coordinates to reach
        else if (in == '1') {
            // Take coordinates to reach from the user
            printf("\nInsert coordinates to reach:\n");
            printf("X: ");
            std::cin >> X;
            printf("Y: ");
            std::cin >> Y;

            x = atof(X.c_str());
            y = atof(Y.c_str());

            // Set new coordinates to reach
            goal_pos.goal.target_pose.header.frame_id = "map";
            goal_pos.goal.target_pose.pose.orientation.w = 1;

            goal_pos.goal.target_pose.pose.position.x = x;
            goal_pos.goal.target_pose.pose.position.y = y;

            // Publish new goal
            pub_goal.publish(goal_pos);
        }

        // Cancel the current goal
        else if (in == '2') {
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
            printf("Goal cancelled.\n");
        }

        // Manual drive
        else if (in == '3') {
            time_flag = 0;
            canc_goal.id = id;
            pub_canc.publish(canc_goal);
            manualDriving();
        }

        // Enable or Disable driving assistance
        else if (in == '4') {
            if (flag == 0) {
                drive_flag = 1;
                flag = 1;
                printf("\nDriving assistance enabled.\n");
            }
            else if (flag == 1) {
                drive_flag = 0;
                flag = 0;
                printf("\nDriving assistance disabled.\n");
            }
        }
    }
}
