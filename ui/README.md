# ROS Bridge and RoslibJS Integration with Next.js

This guide outlines how to integrate a **Next.js** application with **ROS (Robot Operating System)** using **ROSBridge** and **roslibjs**. This setup enables web-based interaction with a ROS environment using JavaScript, opening up possibilities for creating powerful web interfaces for robotics applications.

## Features

- Real-time communication with ROS via ROSBridge
- Web-based interface using `Next.js` for interacting with ROS topics, services, and parameters
- Integration with `roslibjs` for easy access to ROS functionality in the frontend
- Scalable architecture for building complex robotics web applications

## Prerequisites

Before you begin, ensure you have the following installed:

1. **Node.js** and **npm** (Node Package Manager)
2. **ROS** (Robot Operating System)
3. ROSBridge, ROSAPI, and required ROS packages for WebSocket communication

## Getting Started

### 1. Project Setup

Clone the repository and install dependencies:

```bash
git clone https://github.com/your-username/ros-bridge-nextjs.git
cd ros-bridge-nextjs
npm install
```

### 2. Start the Next.js Development Server

Run the following command to start the Next.js application:

```bash
npm run dev
```

The app will be available at `http://localhost:3000`.

### 3. Set Up ROSBridge and ROSAPI

If not already installed, add the ROSBridge and ROSAPI packages:

```bash
sudo apt-get install ros-<your-ros-distro>-rosbridge-suite ros-<your-ros-distro>-rosapi
```

Replace `<your-ros-distro>` with your ROS distribution (e.g., noetic, melodic).

### 4. Launch ROSBridge and ROSAPI

Create a custom launch file to start both ROSBridge and ROSAPI nodes:

1. Navigate to your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   mkdir -p my_launch_files/launch
   touch my_launch_files/launch/rosbridge_with_rosapi.launch
   ```

2. Add the following to the `rosbridge_with_rosapi.launch` file:

   ```xml
   <launch>
     <!-- Start ROSBridge WebSocket server -->
     <node pkg="rosbridge_server" type="rosbridge_websocket" name="rosbridge_websocket" />

     <!-- Start ROSAPI -->
     <node pkg="rosapi" type="rosapi_node" name="rosapi" />

     <!-- Add any custom ROS nodes here -->
     <!-- Example of custom node -->
     <node pkg="my_custom_package" type="my_node" name="my_custom_node" output="screen" />
   </launch>
   ```

3. Launch the ROSBridge and ROSAPI nodes:

   ```bash
   roslaunch my_launch_files rosbridge_with_rosapi.launch
   ```

### 5. Connecting the Frontend to ROS

In your Next.js components, use `roslibjs` to establish a connection with ROS:

```javascript
import ROSLIB from 'roslib';

const RosComponent = () => {
  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // The ROSBridge WebSocket address
    });

    ros.on('connection', () => {
      console.log('Connected to ROSBridge');
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROSBridge:', error);
    });

    // Example of subscribing to a topic
    const listener = new ROSLIB.Topic({
      ros: ros,
      name: '/example_topic',
      messageType: 'std_msgs/String'
    });

    listener.subscribe((message) => {
      console.log('Received message on ' + listener.name + ': ' + message.data);
    });

    return () => {
      ros.close(); // Clean up on component unmount
    };
  }, []);

  // Rest of your component code...
};
```

### 6. Accessing ROSAPI

Use `roslibjs` to interact with ROS services provided by the `rosapi` node:

```javascript
// Example of listing ROS topics
ros.getTopics((result) => {
  console.log('Available topics:', result.topics);
});

// Example of calling a ROS service
const getParamClient = new ROSLIB.Service({
  ros: ros,
  name: '/rosapi/get_param',
  serviceType: 'rosapi/GetParam'
});

const request = new ROSLIB.ServiceRequest({
  name: '/robot_description'
});

getParamClient.callService(request, (result) => {
  console.log('Parameter value:', result.value);
});
```

## Advanced Topics

### Security Considerations

When deploying your application, consider the following security measures:

1. Implement authentication for the ROSBridge WebSocket connection.
2. Use HTTPS for your Next.js application and WSS (WebSocket Secure) for ROSBridge.
3. Implement proper access control for ROS topics and services.

### Performance Optimization

To ensure smooth performance:

1. Use WebSocket compression to reduce bandwidth usage.
2. Implement efficient state management in your React components to handle frequent updates from ROS topics.
3. Consider using Web Workers for computationally intensive tasks to keep the main thread responsive.

## Troubleshooting

Common issues and their solutions:

1. **Connection Refused**: Ensure ROSBridge is running and the WebSocket URL is correct.
2. **Topic Not Found**: Verify that the topic exists in your ROS system using `rostopic list`.
3. **CORS Issues**: Configure ROSBridge to allow cross-origin requests if your app is hosted on a different domain.

## Resources

- [ROSBridge Documentation](http://wiki.ros.org/rosbridge_suite)
- [roslibjs GitHub Repository](https://github.com/RobotWebTools/roslibjs)
- [Next.js Documentation](https://nextjs.org/docs)

By following this guide, you should have a solid foundation for integrating ROS with a Next.js application. This setup allows for the creation of sophisticated web interfaces for robotics applications, combining the power of ROS with the flexibility and performance of modern web technologies.
