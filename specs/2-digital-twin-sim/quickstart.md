# Quickstart: Digital Twin Simulation for Physical AI

## Prerequisites

Before starting with this module, ensure you have:

1. **Basic ROS 2 Knowledge**: Familiarity with ROS 2 concepts from Module 1
2. **Development Environment**: A computer capable of running Gazebo and Unity (recommended: Ubuntu 22.04 or Windows with Unity Hub)
3. **Internet Access**: For downloading Gazebo, Unity, and dependencies

## Setting Up Your Environment

### Gazebo Installation
1. Install ROS 2 Humble Hawksbill with Gazebo Harmonic
2. Set up your ROS 2 environment: `source /opt/ros/humble/setup.bash`
3. Verify Gazebo installation: `gazebo --version`

### Unity Installation
1. Download and install Unity Hub from unity.com
2. Install Unity 2022.3 LTS or later
3. Install the ROS-TCP-Connector package for ROS integration

### Docusaurus Documentation Setup
1. Install Node.js (v16 or higher)
2. Navigate to your docs directory: `cd ai-frontend-book` (or your Docusaurus project)
3. Verify Docusaurus is working: `npm run start`

## Getting Started with the Module

### Chapter 1: Physics Simulation with Gazebo
1. Read about physics simulation principles
2. Set up a basic Gazebo environment
3. Configure robot models with proper physics properties
4. Run the example simulations to understand physics behavior

### Chapter 2: Unity Environment & HRI
1. Create a basic Unity scene
2. Import robot models and set up visualization
3. Implement human-robot interaction mechanisms
4. Connect Unity to ROS 2 for data exchange

### Chapter 3: Sensor Simulation
1. Configure LiDAR simulation in Gazebo
2. Set up depth camera simulation
3. Implement IMU simulation
4. Use simulated sensor data for perception tasks

## Running Simulation Examples

All simulation examples in this module are located in the `tutorial-code/simulation-examples/` directory. To run them:

1. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
2. Navigate to the example directory
3. Launch the simulation: `ros2 launch example_simulation.launch.py`

## Troubleshooting Common Issues

- **Gazebo Not Starting**: Ensure proper graphics drivers are installed
- **Unity Connection Issues**: Check ROS-TCP-Connector configuration
- **Performance Problems**: Adjust simulation quality settings based on hardware
- **Sensor Data Issues**: Verify sensor configuration parameters

## Next Steps

After completing this module, you should be able to:
- Create realistic physics simulations in Gazebo
- Build immersive Unity environments for human-robot interaction
- Configure and validate simulated sensors (LiDAR, depth cameras, IMU)
- Integrate Gazebo and Unity simulations through ROS 2