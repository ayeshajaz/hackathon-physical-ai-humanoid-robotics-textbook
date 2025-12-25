---
sidebar_position: 2
title: "ROS 2 Communication Model"
---

# ROS 2 Communication Model

## Introduction

The communication model in ROS 2 is fundamental to how different components of a robot system interact. Unlike traditional software applications that might use function calls or shared memory, ROS 2 uses a distributed communication model that allows components to run on different machines while maintaining a consistent interface.

This chapter will cover the three primary communication patterns in ROS 2: nodes, topics (publish-subscribe), and services (request-response). We'll also explore how these patterns apply to humanoid robots and provide practical examples using Python and the rclpy library.

## Nodes: The Foundation of ROS 2

Nodes are the fundamental building blocks of any ROS 2 system. A node is a process that performs computation, and it's the container for all other ROS 2 entities like publishers, subscribers, services, and parameters.

### Node Characteristics

- **Process-based**: Each node runs as a separate process
- **Communication hub**: Nodes contain publishers, subscribers, services, and other communication interfaces
- **Named**: Every node has a unique name within the ROS 2 domain
- **Managed**: Nodes can be started, stopped, and monitored independently

### Creating a Node in Python

```python
import rclpy
from rclpy.node import Node

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')
        self.get_logger().info('Communication Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and the Publish-Subscribe Pattern

Topics are named buses over which nodes exchange messages. The publish-subscribe pattern is asynchronous, meaning publishers don't wait for responses from subscribers, and subscribers don't need to know which publishers exist.

### Key Concepts

- **Topics**: Named channels for message exchange
- **Publishers**: Nodes that send messages to topics
- **Subscribers**: Nodes that receive messages from topics
- **Messages**: Data structures that are passed between nodes

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from humanoid robot {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services and the Request-Response Pattern

Services provide a synchronous request-response communication pattern. A service client sends a request to a service server, which processes the request and returns a response.

### Key Concepts

- **Services**: Named endpoints for request-response communication
- **Service Servers**: Nodes that provide service functionality
- **Service Clients**: Nodes that request service functionality
- **Service Types**: Define the request and response message structure

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()
    response = node.send_request(1, 2)
    node.get_logger().info(f'Result: {response.sum}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Patterns in Humanoid Robots

In humanoid robots, these communication patterns serve specific purposes:

### Publish-Subscribe Pattern
- **Sensor Data**: IMU readings, camera images, joint positions published to topics
- **Robot Status**: Battery level, temperature, operational status
- **Control Commands**: Joint position commands, walking gait parameters
- **Environment Perception**: Detected objects, obstacle locations, map updates

### Service Pattern
- **Action Requests**: "Walk to location", "Pick up object", "Change mode"
- **Configuration**: Setting parameters, calibrating sensors
- **Safety Operations**: Emergency stop, reset procedures
- **Data Queries**: Requesting specific information on demand

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service settings to configure how messages are delivered:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep last N messages vs. keep all messages
- **Depth**: Size of the message queue

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Example QoS for sensor data (real-time, best effort)
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)

# Example QoS for safety-critical data (reliable, persistent)
safety_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL
)
```

## Practical Example: Humanoid Robot Control

Let's look at how these patterns work together in a humanoid robot control scenario:

1. **Sensor Node**: Publishes joint positions and IMU data
2. **Perception Node**: Publishes detected objects and environment information
3. **Planning Node**: Subscribes to sensor and perception data, publishes control commands
4. **Control Node**: Subscribes to control commands, sends low-level commands to actuators
5. **UI Node**: Provides services for high-level commands from operators

This distributed approach allows each component to be developed and tested independently while maintaining the flexibility to modify individual components without affecting others.

## Best Practices for Communication

1. **Use appropriate QoS settings**: Match the QoS to your application's requirements
2. **Design clear message types**: Create messages that are self-explanatory
3. **Follow naming conventions**: Use consistent, descriptive names for topics and services
4. **Consider data rates**: Don't overload the system with unnecessary messages
5. **Implement error handling**: Plan for communication failures and timeouts
6. **Use appropriate message types**: Leverage standard message types when possible

## Summary

The ROS 2 communication model provides a flexible and robust foundation for robot software development. By understanding nodes, topics, and services, you can design systems that scale from simple robots to complex humanoid platforms. The publish-subscribe pattern enables asynchronous, decoupled communication, while services provide synchronous request-response interactions when needed.

[← Previous: Introduction to ROS 2](./intro-to-ros2) | [Next: Robot Structure with URDF →](./urdf-structure)