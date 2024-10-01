// @ts-nocheck
/**
 * @file services.ts
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief ROS services
 *
 * @copyright Copyright (c) 2024
 *
 */
import * as ROSLIB from 'roslib';
import tf2 from './tf2.json'


const ros = new ROSLIB.Ros({ encoding: 'ascii' , url: 'ws://localhost:9090'});

const board_detection = new ROSLIB.Service({
    ros: ros,
    name: '/board_detection',
    serviceType: 'msvc2024_setup/GetBoardLocationRequest'
    });

const board_detection_service = () => {
    board_detection.callService({}, (result) => {
        console.log("/board_detection called",result);
    }
)}

const touch_detection = new ROSLIB.Service({
    ros: ros,
    name: '/touch_detection',
    serviceType: 'msvc2024_setup/GetBoardLocationRequest'
    });


const touch_detection_service = () => {
    touch_detection.callService({}, (result) => {
        console.log("/touch_detection called",result);
    }
)}

const set_speed_slider = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/dashboard/set_speed_slider',
    serviceType: 'ur_ros_driver/SetSpeedSlider'
});


const set_speed_slider_service = ({slider_args} : {slider_args:number}) => {

    const args = new ROSLIB.ServiceRequest({
        slider: slider_args
    });

    console.log("sdfd",slider_args)

    set_speed_slider.callService(args, (result) => {
        console.log("/speed_slider called",result);
        // console.log("slider",);
    }
)}

const brake_release = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/dashboard/brake_release',
    serviceType: 'std_srvs/TriggerRequest'
});

const brake_release_service = () => {
    brake_release.callService({}, (result) => {
        console.log("/brake_release called",result);
    }
)}

const powerOff = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/dashboard/powerOff',
    serviceType: 'std_srvs/TriggerRequest'
});

const powerOff_service = () => {
    powerOff.callService({}, (result) => {
        console.log("/powerOff called",result);
    }
)}

const restart_safely = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/dashboard/restart_safely',
    serviceType: 'std_srvs/TriggerRequest'
});

const restart_safely_service = () => {
    restart_safely.callService({}, (result) => {
        console.log("/restart_safely called",result);
    }
)}

const unlock_protective_stop = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/dashboard/unlock_protective_stop',
    serviceType: 'std_srvs/TriggerRequest'
});

const unlock_protective_stop_service = () => {
    unlock_protective_stop.callService({}, (result) => {
        console.log("/unlock_protective_stop called",result);
    }
)}

const set_gripper = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/robotiq/set_gripper',
    serviceType: 'ur_ros_driver/SetGripperRequest'
});

const set_gripper_service = ({position_unit = 0, position = 0.0, speed=0.0, force=0.0, asynchronous=false} : {position_unit : number, position: number, speed:number, force:number, asynchronous: boolean}) => {

    const args = new ROSLIB.ServiceRequest({
        position_unit: position_unit,
        position: position,
        speed: speed,
        force: force,
        asynchronous: asynchronous
    });

    set_gripper.callService( args , (result) => {
        console.log("/set_gripper called",result);
    }
)}

const get_gripper_calibration = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/robotiq/get_gripper_calib',
    serviceType: 'ur_ros_driver/GetGripperCalibRequest'
});

const set_freedive = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/set_freedive',
    serviceType: 'ur_ros_driver/SetFreedriveRequest'
});

export type set_freedive_args = {
    IO : boolean,
    free_axes : number[],
    feature : number,
    custom_frame ?: {
        translation :{
            x : number,
            y : number,
            z : number
        },
        rotation : {
            x : number,
            y : number,
            z : number,
            w : number
        }
    }
}

const set_freedive_service = ({args}: {args:set_freedive_args}) => {
    console.log("args",args)
    set_freedive.callService(args, (result) => {
        console.log("/set_freedive called",result);
    }
)}

const set_cart_target = new ROSLIB.Service({
    ros: ros,
    name: '/ur_hardware_interface/set_cart_target',
    serviceType: 'ur_ros_driver/SetCartTarget'
});

export type set_cart_target_args = {
    mode : number,
    cartesian_goal : {
        translation : {
            x : number,
            y : number,
            z : number
        },
        rotation : {
            x : number,
            y : number,
            z : number,
            w : number
        }
    },
    speed : number,
    acceleration : number,
    asynchronous : boolean
}

const set_cart_target_service = ({args}: {args:set_cart_target_args}) => {
    set_cart_target.callService(args, (result) => {
        console.log("/set_cart_target called",result);
    }
)}

export {get_gripper_calibration,board_detection_service, touch_detection_service, set_speed_slider_service, brake_release_service, powerOff_service, restart_safely_service, unlock_protective_stop_service, set_gripper_service, set_freedive_service, set_cart_target_service, set_cart_target_args}
