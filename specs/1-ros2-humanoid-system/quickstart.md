# Quickstart: ROS 2 for Physical AI Education

## Prerequisites

Before starting with this module, ensure you have:

1. **Basic Python Knowledge**: Familiarity with Python programming concepts
2. **Development Environment**: A computer capable of running ROS 2 (Ubuntu 22.04 recommended)
3. **Internet Access**: For downloading ROS 2 and dependencies

## Setting Up Your Environment

### ROS 2 Installation
1. Install ROS 2 Humble Hawksbill following the official installation guide
2. Set up your ROS 2 environment with `source /opt/ros/humble/setup.bash`
3. Create a workspace: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws`

### Docusaurus Documentation Setup
1. Install Node.js (v16 or higher)
2. Install Docusaurus: `npm init docusaurus@latest docs classic`
3. Navigate to your docs directory: `cd docs`

## Getting Started with the Module

### Chapter 1: Introduction to ROS 2
1. Read the introduction to understand what ROS 2 is
2. Learn about DDS concepts and ROS 2 architecture
3. Understand why ROS 2 matters for humanoid robots

### Chapter 2: Communication Model
1. Set up your Python environment with rclpy
2. Run the example code snippets to understand nodes, topics, and services
3. Practice creating simple publisher and subscriber nodes

### Chapter 3: URDF Structure
1. Examine sample URDF files provided in the tutorial
2. Learn to identify joints, links, and other components
3. Practice reading and interpreting URDF structure

## Running Code Examples

All code examples in this module are located in the `tutorial-code/python-examples/` directory. To run them:

1. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
2. Navigate to the example directory
3. Run the Python script: `python3 example_node.py`

## Troubleshooting Common Issues

- **ROS 2 Environment Not Found**: Ensure you've sourced the ROS 2 setup file
- **Python Import Errors**: Check that rclpy is properly installed in your environment
- **Permission Issues**: Run ROS 2 commands with appropriate permissions

## Next Steps

After completing this module, you should be able to:
- Explain ROS 2 architecture and communication flow
- Create basic ROS 2 nodes with publishers and subscribers
- Read and interpret humanoid URDF files
- Understand how Python agents interface with ROS controllers

## Accessing the Completed Module

The completed ROS 2 module is available in the Docusaurus documentation site. To access it:

1. Navigate to your Docusaurus project directory
2. Start the development server: `npm run start`
3. Access the ROS 2 module at the `/docs/ros2-module/intro-to-ros2` path
4. The module consists of three main chapters:
   - Introduction to ROS 2 for Physical AI
   - ROS 2 Communication Model
   - Robot Structure with URDF
5. Code examples are available in the `tutorial-code/python-examples/` directory