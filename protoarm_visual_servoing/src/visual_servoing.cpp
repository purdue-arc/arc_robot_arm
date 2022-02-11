/* Author: Ashvin Iyer */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>

#include "protoarm_kinematics/kinematics.h"
#include "tools/pid_controller.h"

//TODO: find values
#define BOARD_SIZE 10
#define FREQUENCY 10
#define GOAL_HEIGHT 10

geometry_msgs::Pose board_position;
geometry_msgs::Pose arm_position;
geometry_msgs::Pose setpoint;

int main(int argc, char** argv) {
	ros::init(argc, argv, "visual_servoing");
	ros::NodeHandle node_handle("~");
    
    ros::Subscriber apriltag_sub = node_handle.subscribe("/apriltag", 10, update_pose);
    ros::Subscriber setpoint_sub = node_handle.subscribe("/move", 10, update_setpoint);
	
    ros::Rate loop_rate(FREQUENCY);
    float dt = 1.0 / FREQUENCY;
    Kinematics robot_arm = Kinematics();

    //TODO find PID values
    //Trying with x y z positions for now, might need to change it to polar + 2d arm coordinates if PID needs to be used with the exact axes of movement
    PIDController x_controller = PIDController(0, 0, 0);
    PIDController y_controller = PIDController(0, 0, 0);
    PIDController z_controller = PIDController(0, 0, 0);
    while (ros::ok()) {
        geometry_msgs::Pose current_goal = robot_arm.move_group_->getCurrentPose().pose;

        float x_vel = x_controller.calculate(arm_position.position.x, setpoint.position.x, dt);
        float y_vel = y_controller.calculate(arm_position.position.y, setpoint.position.y, dt);
        float z_vel = z_controller.calculate(arm_position.position.z, setpoint.position.z, dt);

        current_goal.position.x += x_vel * dt;
        current_goal.position.y += y_vel * dt;
        current_goal.position.z += z_vel * dt;

        robot_arm.moveToPoseGoal(current_goal);
        loop_rate.sleep();
    }
    
	return 0;
}

void update_pose(apriltag_ros::AprilTagDetectionArray detections) {
    //Assuming first tag is board, second tag is arm, might need to change
    board_position = detections.detections[0].pose.pose.pose;
    arm_position = detections.detections[1].pose.pose.pose;
}

std::vector<float> quat_to_euler(geometry_msgs::Quaternion q) { //Returns in RPY
    std::vector<float> angles(3);
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
}

void update_setpoint(std_msgs::String move) {
    float x_shift = (move.data[0] - 'a') * BOARD_SIZE;
    float y_shift = (move.data[1] - '0') * BOARD_SIZE;
    float angle = quat_to_euler(board_position.orientation)[2];
    x_shift *= std::cos(angle);
    y_shift *= std::sin(angle);
    setpoint.position.x = board_position.position.x + x_shift;
    setpoint.position.y = board_position.position.y + y_shift;
    setpoint.position.z = GOAL_HEIGHT;
}