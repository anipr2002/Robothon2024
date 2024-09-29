// @ts-nocheck
import * as ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({ encoding: 'ascii' , url: 'ws://localhost:9090'});

const tcp_pose = new ROSLIB.Topic({
    ros: ros,
    name: '/ur_hardware_interface/tcp_pose',
    messageType: 'geometry_msgs/TransformStamped',
    throttle_rate: 100
});

export default tcp_pose;
