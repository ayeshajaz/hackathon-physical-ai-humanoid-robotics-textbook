---
sidebar_position: 5
title: "Assessment Questions"
---

# Assessment Questions

## Chapter 1: Introduction to ROS 2 for Physical AI

### Multiple Choice Questions

1. What does DDS stand for in the context of ROS 2?
   a) Distributed Data System
   b) Data Distribution Service
   c) Distributed Development System
   d) Data Description Service

2. Which of the following is NOT a core communication pattern in ROS 2?
   a) Topics
   b) Services
   c) Actions
   d) Databases

3. Why is ROS 2 particularly valuable for humanoid robots?
   a) It provides distributed architecture for multiple processors
   b) It handles real-time requirements
   c) It enables modular development
   d) All of the above

### Short Answer Questions

4. Explain the concept of ROS 2 as the "nervous system" of a humanoid robot.

5. Describe the difference between ROS 2 and traditional software development approaches.

### Answers:
1. b) Data Distribution Service
2. d) Databases
3. d) All of the above
4. ROS 2 acts like a nervous system by allowing different parts of the robot to communicate and coordinate, similar to how the human nervous system coordinates different body parts.
5. ROS 2 uses distributed architecture with asynchronous communication patterns, unlike traditional monolithic applications.

## Chapter 2: ROS 2 Communication Model

### Multiple Choice Questions

1. What is the primary purpose of a ROS 2 node?
   a) To store data permanently
   b) To serve as the fundamental computational unit
   c) To manage hardware directly
   d) To replace operating systems

2. Which communication pattern is used for asynchronous, continuous data streams?
   a) Services
   b) Actions
   c) Topics
   d) Parameters

3. What does QoS stand for in ROS 2?
   a) Quality of Service
   b) Quick Operating System
   c) Quantitative Operating System
   d) Quality Operating Service

### Programming Questions

4. Write a simple publisher node that publishes a string message to the topic "robot_status" every second.

5. Create a service server that adds two integers and returns the result.

### Answers:
4. See the simple_publisher.py example in the tutorial-code/python-examples directory.
5. See the service_server.py example in the tutorial-code/python-examples directory.

## Chapter 3: Robot Structure with URDF

### Multiple Choice Questions

1. What does URDF stand for?
   a) Unified Robot Development Framework
   b) Universal Robot Description Format
   c) Unified Robot Description Format
   d) Universal Robot Development Framework

2. Which of the following is NOT a joint type in URDF?
   a) Revolute
   b) Prismatic
   c) Continuous
   d) Recursive

3. What is the purpose of the `<inertial>` tag in URDF?
   a) To define visual appearance
   b) To define collision properties
   c) To define mass and inertia properties
   d) To define joint limits

### Short Answer Questions

4. Explain the difference between `<visual> `and `<collision> `tags in URDF.

5. Describe the purpose of joint limits in a humanoid robot URDF.

### Answers:
1. c) Unified Robot Description Format
2. d) Recursive
3. c) To define mass and inertia properties
4. The `<visual> `tag defines how the robot appears in visualization tools, while `<collision>` defines the geometry used for physics simulation and collision detection.
5. Joint limits ensure that the robot's movements stay within safe and physically possible ranges.

## Comprehensive Questions

1. Design a simple ROS 2 system for a humanoid robot that includes:
   - A sensor node that publishes joint positions
   - A controller node that subscribes to joint positions and publishes motor commands
   - A UI node that provides a service to request robot poses

2. Create a URDF snippet for a simple robot with a base, one rotating joint, and an end effector.

3. Explain how you would integrate the communication patterns from Chapter 2 with the robot structure from Chapter 3 to create a functional humanoid robot system.

## Practical Exercises

1. Implement the publisher and subscriber nodes from the examples and run them together.
2. Modify the simple humanoid URDF to add a simple gripper to one of the arms.
3. Create a launch file that starts multiple nodes from the communication examples simultaneously.

These exercises will help reinforce the concepts learned in each chapter and provide hands-on experience with ROS 2 and URDF.

[← Previous: Summary](./summary) | [Glossary →](./glossary)