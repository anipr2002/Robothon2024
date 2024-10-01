// @ts-nocheck

/**
 * @file topics.ts
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief ROS topics
 *
 * @copyright Copyright (c) 2024
 *
 */
import * as ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({ encoding: 'ascii' , url: 'ws://localhost:9090'});

const tcp_pose = new ROSLIB.Topic({
    ros: ros,
    name: '/ur_hardware_interface/tcp_pose',
    messageType: 'geometry_msgs/TransformStamped',
    throttle_rate: 100
});

export default tcp_pose;
