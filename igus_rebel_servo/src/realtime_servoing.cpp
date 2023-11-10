/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : demo_pose.cpp
 *      Project   : moveit_servo
 *      Created   : 06/07/2023
 *      Author    : V Mohammed Ibrahim
 *      Description : Example of controlling a robot through pose commands via the C++ API.
 * 		Link	  : https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
 * 		Code	  : https://github.com/ros-planning/moveit2/blob/iron/moveit_ros/moveit_servo/demos/cpp_interface/demo_pose.cpp
 */

// ROS2 imports
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ imports
#include <atomic>
#include <chrono>
#include <mutex>

// Moveit2 imports
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>


using namespace moveit_servo;

namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("rt_pose_demo");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // The servo object expects to get a ROS node.
    const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("rt_servo_demo");

    // Get the servo parameters from the config file passed as parameter to this node
    const std::string param_namespace = "moveit_servo";
    const std::shared_ptr<const servo::ParamListener> servo_param_listener =
        std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
    const servo::Params servo_params = servo_param_listener->get_params();

    // The publisher to send trajectory message to the robot controller.
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
        demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                           rclcpp::SystemDefaultsQoS());

    // Create the servo object
    const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
        createPlanningSceneMonitor(demo_node, servo_params);
    Servo servo(demo_node, servo_param_listener, planning_scene_monitor);

	// create target pose publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub =
		demo_node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());
	

    // Wait for some time, so that the planning scene is loaded in rviz.
    // This is just for convenience, should not be used for sync in real application.
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // For syncing pose tracking thread and main thread.
    std::mutex pose_guard;
    std::atomic<bool> stop_tracking (false);

    // Set the command type for servo.
	// Objective: take as input the goal_pose and output the joint_trajectory required to reach it
    servo.setCommandType(CommandType::POSE);

    // The dynamically updated target pose.
    PoseCommand target_pose;
    target_pose.frame_id = servo_params.planning_frame;
    // generate random feasible pose
	target_pose.pose = Eigen::Isometry3d(Eigen::Translation3d(0.5160676836967468, 0.044317007064819336, 0.1));
	target_pose.pose.rotate(Eigen::Quaterniond(0.0, 0.0, -0.7443628133564246, 0.6677754129137345) );

	// create posestamped message with the target pose to be published
	geometry_msgs::msg::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = servo_params.planning_frame;
	target_pose_msg.header.stamp = rclcpp::Clock().now();
	tf2::convert(target_pose.pose, target_pose_msg.pose);
	target_pose_pub->publish(target_pose_msg);
	
    // The pose tracking lambda function that will be run in a separate thread
    auto pose_tracker = [&]() {
        KinematicState joint_state;
        rclcpp::WallRate tracking_rate(1.0 / servo_params.publish_period);
        while (!stop_tracking && rclcpp::ok()) {

            { // mutex lock on target_pose
                std::lock_guard<std::mutex> pguard(pose_guard);
                joint_state = servo.getNextJointState(target_pose);
            }
			
            StatusCode status = servo.getStatus();
            if (status != StatusCode::INVALID) {
                trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
			}
			if (status != StatusCode::NO_WARNING) {
				RCLCPP_INFO(LOGGER, "status = %s", servo.getStatusMessage().c_str());
			}
			tracking_rate.sleep();
        }
    };

    // Pose tracking thread will exit upon reaching this pose.
	Eigen::Isometry3d current_pose, target_pose_copy;
	{
		std::lock_guard<std::mutex> pguard(pose_guard);
		current_pose = target_pose.pose;
		target_pose_copy = target_pose.pose;
	}
    
    std::thread tracker_thread(pose_tracker);
    tracker_thread.detach();

    // The target pose (frame being tracked) moves by this step size each iteration.
    //Eigen::Vector3d linear_step_size{0.0, 0.0, -0.002};
    //Eigen::AngleAxisd angular_step_size(0.01, Eigen::Vector3d::UnitZ());

    // Frequency at which commands will be sent to the robot controller.
    rclcpp::WallRate command_rate(50);
    RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());

    while (!stop_tracking && rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> pguard(pose_guard);
            current_pose = servo.getEndEffectorPose();
            const bool satisfies_linear_tolerance = target_pose_copy.translation().isApprox(
                current_pose.translation(), servo_params.pose_tracking.linear_tolerance);
            const bool satisfies_angular_tolerance =
                target_pose_copy.rotation().isApprox(current_pose.rotation(), servo_params.pose_tracking.angular_tolerance);
            stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;
            // Dynamically update the target pose.
            //if (!satisfies_linear_tolerance)
                //target_pose.pose.translate(linear_step_size);
            //if (!satisfies_angular_tolerance)
                //target_pose.pose.rotate(angular_step_size);
			target_pose_pub->publish(target_pose_msg);
        }

        command_rate.sleep();
    }

    RCLCPP_INFO_STREAM(LOGGER, "REACHED : " << stop_tracking);
    stop_tracking = true;

    if (tracker_thread.joinable())
        tracker_thread.join();

    RCLCPP_INFO(LOGGER, "Exiting demo.");
    rclcpp::shutdown();
}