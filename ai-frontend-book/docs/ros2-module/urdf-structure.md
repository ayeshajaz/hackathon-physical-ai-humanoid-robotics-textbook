---
sidebar_position: 3
title: "Robot Structure with URDF"
---

# Robot Structure with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertia, visual appearance, and collision properties.

For humanoid robots, URDF is particularly important because it allows us to define complex multi-body systems with many degrees of freedom. This description is essential for simulation, visualization, motion planning, and control.

## URDF Fundamentals

### Links

Links represent rigid bodies in the robot. Each link has:
- A name
- Visual properties (shape, color, material)
- Collision properties
- Inertial properties (mass, center of mass, inertia matrix)

### Joints

Joints connect links and define the kinematic relationship between them. Joint types include:
- **Fixed**: No movement between links
- **Revolute**: Single axis of rotation with limits
- **Continuous**: Single axis of rotation without limits
- **Prismatic**: Single axis of translation with limits
- **Floating**: Six degrees of freedom
- **Planar**: Motion on a plane

### Materials and Visual Properties

URDF allows you to define how the robot appears visually, including colors, textures, and shapes.

## Basic URDF Structure

Here's a simple URDF example:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Child link connected by a fixed joint -->
  <link name="child_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting the links -->
  <joint name="base_to_child" type="fixed">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots have a more complex structure with multiple limbs. A typical humanoid URDF includes:

### Torso
- Base of the robot
- Usually contains main computer and power systems
- Connection point for head, arms, and legs

### Head
- Contains cameras and other sensors
- Often has pan and tilt joints for looking around

### Arms
- Shoulder, elbow, and wrist joints
- May include hands with finger joints
- Used for manipulation tasks

### Legs
- Hip, knee, and ankle joints
- Used for locomotion and balance
- May include feet with toe joints

### Joint Limitations in Humanoid Robots

Humanoid URDFs must carefully define joint limits to match human-like ranges of motion:
- Shoulders have wide range of motion
- Elbows are primarily revolute joints
- Wrists allow for pitch and yaw
- Hips have multiple degrees of freedom
- Knees are primarily single-axis joints
- Ankles allow for balance adjustments

## Reading and Interpreting URDF Files

When reading a URDF file, focus on:

1. **The kinematic chain**: How links are connected through joints
2. **Joint types and limits**: What movements are possible
3. **Reference frames**: How coordinate systems are defined
4. **Physical properties**: Mass, inertia, and collision properties

### Example Analysis

Let's analyze a portion of a humanoid URDF:

```xml
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0.0 0.1 0.0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

This defines a revolute joint for the left hip yaw:
- Connects the torso to the left thigh
- Located at position (0.0, 0.1, 0.0) relative to the torso
- Rotates around the Z-axis
- Limited to ±90 degrees (±1.57 radians)
- Maximum effort of 100 and velocity of 1

## URDF and Simulation

URDF is essential for robot simulation in tools like Gazebo:

1. **Physics simulation**: Collision and inertial properties define how the robot interacts with the environment
2. **Visualization**: Visual properties determine how the robot appears in simulation
3. **Sensor placement**: URDF defines where sensors are mounted on the robot
4. **Control interfaces**: Joint definitions provide the interface for controllers

## URDF Best Practices

### Naming Conventions
- Use consistent naming (e.g., left_arm_joint_1, left_arm_joint_2)
- Follow ROS conventions for frame names
- Use descriptive names that indicate function

### Organization
- Group related links and joints together
- Use consistent structure across similar components
- Include comments for complex sections

### Validation
- Check that the URDF is well-formed XML
- Verify that all joints have proper parent-child relationships
- Ensure joint limits are realistic
- Test the URDF in visualization tools

## Xacro for Complex URDFs

For complex robots, Xacro (XML Macros) is often used to simplify URDF creation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <xacro:macro name="simple_link" params="name xyz">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="0.1 0.1 0.1"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="0.1 0.1 0.1"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:simple_link name="link1" xyz="0 0 0"/>

</robot>
```

Xacro allows for:
- Parameterized macros
- Mathematical expressions
- Conditional statements
- File inclusion

## Simulation Readiness

To ensure your URDF is ready for simulation:

1. **Complete kinematic chain**: All parts of the robot should be connected
2. **Proper inertial properties**: Mass and inertia values for physics simulation
3. **Collision models**: Defined for all links that should interact with the environment
4. **Visual models**: Defined for rendering in simulation
5. **Joint limits**: Realistic values to prevent damage in simulation

## Basic Modeling Techniques

### Creating Simple Shapes
- Use basic geometries: box, cylinder, sphere
- Combine multiple visual/collision elements per link
- Use origin tags to position elements correctly

### Defining Joints
- Choose appropriate joint types
- Set realistic limits
- Define proper parent-child relationships
- Set correct axes of rotation/translation

### Adding Sensors
- Define mounting points for sensors
- Consider field of view and mounting orientation
- Include sensor parameters in the URDF

## Troubleshooting Common URDF Issues

### Parser Errors
- Check XML well-formedness
- Verify all tags are properly closed
- Ensure no special characters in names

### Kinematic Issues
- Check that all joints have proper parent-child relationships
- Verify that there's a clear path from base to all other links
- Ensure no disconnected components

### Simulation Issues
- Check that inertial properties are defined for all links
- Verify that collision models are appropriate
- Ensure joint limits are realistic

## URDF Tools

ROS provides several tools for working with URDF:

- **check_urdf**: Validates URDF and shows kinematic tree
- **urdf_to_graphiz**: Creates visual representation of kinematic tree
- **RViz**: Visualizes URDF models
- **Gazebo**: Simulates URDF-based robots

## Summary

URDF is a critical component of humanoid robot development, providing the foundation for simulation, visualization, and control. Understanding how to read, write, and interpret URDF files is essential for working with humanoid robots in ROS 2.

The key aspects to remember are:
- Links define the rigid bodies of the robot
- Joints define how links connect and move relative to each other
- Proper physical properties are essential for simulation
- Consistent naming and organization make URDFs easier to work with

[← Previous: ROS 2 Communication Model](./communication-model) | [Next: Summary →](./summary)