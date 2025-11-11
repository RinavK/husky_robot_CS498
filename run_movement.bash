#!/bin/bash

echo "=== Starting robot sequence ==="

echo "Step 1: Moving Husky in a circle"
ros2 topic pub --rate 10 /husky_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" &
PID=$!
sleep 10
kill $PID

echo "Stopping Husky"
ros2 topic pub --once /husky_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1

echo "Step 2: Moving arm"
ros2 topic pub --once /ur_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint', 'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'],points: [{positions: [0.0, -1.57, 0.0, 0.0, 1.0, 0.0],time_from_start: {sec: 4}}]}"
ros2 topic pub --once /ur_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint', 'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'],points: [{positions: [0.0, -1.57, 0.0, 0.0, 1.0, 0.0],time_from_start: {sec: 4}}]}"

echo "Waiting for arm"
sleep 10

echo "Step 3: Opening gripper"
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'],points: [{positions: [0.0, 0.0, 0.0, 0.0],time_from_start: {sec: 2}}]}"
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'],points: [{positions: [0.0, 0.0, 0.0, 0.0],time_from_start: {sec: 2}}]}"

sleep 5

echo "Step 4: Closing gripper..."
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'],points: [{positions: [0.7, 0.5, 0.7, 0.5],time_from_start: {sec: 2}}]}"
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'],points: [{positions: [0.7, 0.5, 0.7, 0.5],time_from_start: {sec: 2}}]}"

sleep 5

echo "Step 5: Opening gripper"
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'],points: [{positions: [0.0, 0.0, 0.0, 0.0],time_from_start: {sec: 2}}]}"
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2'],points: [{positions: [0.0, 0.0, 0.0, 0.0],time_from_start: {sec: 2}}]}"

sleep 5

echo "=== Sequence complete! ==="