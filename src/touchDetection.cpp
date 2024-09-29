/**
 * @file touchDetection.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de),
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program to manually localize the taskboard
 * @version 0.1
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>

#include <fstream>
#include <MSVC2024_Setup_2024/eigen_json.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <boost/array.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Line_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Aff_transformation_3.h>


#include <ur_ros_driver/SetFreedrive.h>
#include <ur_ros_driver/SetCartTarget.h>
#include <ur_ros_driver/SetGripper.h>
#include <MSVC2024_Setup_2024/AddTf2.h>
#include <MSVC2024_Setup_2024/GetBoardLocation.h>


using json = nlohmann::json;

typedef CGAL::Exact_predicates_exact_constructions_kernel K;

geometry_msgs::TransformStamped tcp_pose;

void tcpCallback(const geometry_msgs::TransformStamped& msg)
{
    tcp_pose = msg;
}

K::Aff_transformation_3 eigen2CGAL(Eigen::Matrix4d pose) {
    return K::Aff_transformation_3( pose.coeff(0,0), pose.coeff(0,1), pose.coeff(0,2), pose.coeff(0,3),
                                    pose.coeff(1,0), pose.coeff(1,1), pose.coeff(1,2), pose.coeff(1,3),
                                    pose.coeff(2,0), pose.coeff(2,1), pose.coeff(2,2), pose.coeff(2,3));
}

void calc_trans(std::string name, geometry_msgs::Transform trans)
{
    Eigen::Isometry3d iso;
    tf::transformMsgToEigen(trans,iso);

    Eigen::Quaterniond quat(iso.matrix().topLeftCorner<3, 3>());
    auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    std::cout << name << ": " << std::endl << "pos: " <<  trans.translation.x << ", " << trans.translation.y << ", " << trans.translation.z << std::endl;
    std::cout << "quat: "<<  trans.rotation.x << ", " << trans.rotation.y << ", " << trans.rotation.z << ", "<< trans.rotation.w << std::endl;
    // std::cout << "euler: " << euler[0] << ", " << euler[1] << ", " << euler[2]<< std::endl;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "touch_localizer_node");
    ros::NodeHandle n;
    ros::ServiceClient client_f = n.serviceClient<ur_ros_driver::SetFreedrive>("/ur_hardware_interface/set_freedive");
    ros::ServiceClient client_move = n.serviceClient<ur_ros_driver::SetCartTarget>("/ur_hardware_interface/set_cart_target");
    ros::ServiceClient client_store_tf = n.serviceClient<MSVC2024_Setup_2024::AddTf2>("/store_tf");
    ros::ServiceClient gripper_move = n.serviceClient<ur_ros_driver::SetGripper>("/ur_hardware_interface/robotiq/set_gripper");
    ros::ServiceClient taskboard_detection_ = n.serviceClient<MSVC2024_Setup_2024::GetBoardLocation>("/board_detection");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ur_ros_driver::SetGripper gripper_srv;
    gripper_srv.request.position = 100;
    gripper_srv.request.force = 1;
    gripper_srv.request.speed = 100;
    gripper_srv.request.asynchronous = true;

    ros::Subscriber sub = n.subscribe("/ur_hardware_interface/tcp_pose", 1000, tcpCallback);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    //Normal Gripper
    K::Plane_3 xzPlane(K::Point_3(0,-0.0105,1), K::Point_3(1,-0.0105,0), K::Point_3(-1,-0.0105,0)); //correction of -10,5mm because gripper is 21mm wide
    // Rubberized Gripper
    //K::Plane_3 xzPlane(K::Point_3(0,-0.01175,1), K::Point_3(1,-0.01175,0), K::Point_3(-1,-0.01175,0)); //correction of -10,5mm because gripper is 23.5mm wide
    K::Plane_3 xyPlane(K::Point_3(0,0,0), K::Point_3(1,0,0), K::Point_3(0,1,0));
    // wait for buffer to fill
    while(tcp_pose.header.stamp.isZero()) ros::Duration(0.1).sleep();
    //align tcp to z axis
    ur_ros_driver::SetCartTarget target_srv;
    target_srv.request.mode = 1;
    target_srv.request.speed = 0.5;
    target_srv.request.acceleration = 0.1;
    target_srv.request.asynchronous = false;
    target_srv.request.cartesian_goal.translation = tcp_pose.transform.translation;
    tf::quaternionEigenToMsg(Eigen::Quaterniond(0,1,0,0),target_srv.request.cartesian_goal.rotation);
    std::cout << target_srv.request.cartesian_goal << std::endl;
    client_move.call(target_srv);

    gripper_move.call(gripper_srv); // Open Gripper
    /*
    std::cout << "Detection with Cam" << std::endl;

    MSVC2024_Setup_2024::GetBoardLocation srv;
    if(!taskboard_detection_.call(srv))
    {
        ROS_INFO("Detection went wrong");
    }
    if(srv.response.success == false)
    {
        ROS_INFO("Detection went wrong");
    }

    ros::Duration(5).sleep();
    geometry_msgs::Transform transfrom_detection;
    transfrom_detection = tfBuffer.lookupTransform("base_link", "task_board",ros::Time(0)).transform;


    std::cout << "Press enter to start Touch detection" << std::endl;
    std::cin.ignore();
    */


    // Turn on freedrive
    ur_ros_driver::SetFreedrive freedrive_srv;
    freedrive_srv.request.IO = true;
    freedrive_srv.request.free_axes = {1, 1, 1, 0, 0, 1};
    client_f.call(freedrive_srv);
    std::cout << "This is TouchDetectioncpp" << std::endl;
    std::cout << "Move the robot to the long edge and press enter" << std::endl;
    std::cin.ignore(); //wait for enter press

    // get long edge plane
    Eigen::Isometry3d robotPose;
    tf::transformMsgToEigen(tcp_pose.transform, robotPose);
    Eigen::Matrix4d longPose = robotPose.matrix();
    auto longSidePlane = xzPlane.transform(eigen2CGAL(longPose)); //transform plane to tcp pose

    std::cout << "Move the robot to the short edge..." << std::endl;
    std::cin.ignore(); //wait for enter press

    // get short edge plane
    tf::transformMsgToEigen(tcp_pose.transform, robotPose);
    Eigen::Matrix4d sidePose = robotPose.matrix();
    auto shortSidePlane = xzPlane.transform(eigen2CGAL(sidePose)); //transform plane to tcp pose

    std::cout << "Move the robot to the top of the board..." << std::endl;
    std::cin.ignore(); //wait for enter press

    // get top plane
    tf::transformMsgToEigen(tcp_pose.transform, robotPose);
    Eigen::Matrix4d topPose = robotPose.matrix();
    auto topPlane = xyPlane.transform(eigen2CGAL(topPose)); //transform plane to tcp pose

    auto firstIntersect = CGAL::intersection(longSidePlane, shortSidePlane); //calculate intersection of sides --> returns a line in 3D space
    auto finalIntersect = CGAL::intersection(*boost::get<K::Line_3>(&*firstIntersect), topPlane); //calculate intersection of resulting line and top plane to get final point
    K::Point_3 * resPoint = boost::get<K::Point_3>(&*finalIntersect); //grab point out of boosted result
    std::cout << *resPoint << std::endl;

    // Take orientation of long edge and use our calculated position
    Eigen::Matrix4d resultPose = longPose;
    resultPose(0,3) = CGAL::to_double(resPoint->x());
    resultPose(1,3) = CGAL::to_double(resPoint->y());
    resultPose(2,3) = CGAL::to_double(resPoint->z());

    // Rotate result around X because Z of TCP points down
    resultPose.block<3,3>(0,0) = resultPose.block<3,3>(0,0) * Eigen::Quaterniond(0,1,0,0).matrix();

    std::string choice = "";
    std::string target_frame = "";
    std::string parent_frame = "base_adj";
    std::cout << "Choose target frame: 1 Taskboard    2 BYOD-board " << std::endl;
    std::cin  >> choice;
    if (choice == "1") {
        std::cout << "Taskboard Pose: " << resultPose << std::endl;
        target_frame = "task_board";
    }
    else if (choice == "2") {
        std::cout << "BYOD-board Pose: " << resultPose << std::endl;
        target_frame = "byod_board";
        parent_frame = "base";
    }
    else {
        std::cout << "Invalid choice" << std::endl;
        return false;
    }


    // Send TF to our server
    MSVC2024_Setup_2024::AddTf2 addTF_srv;
    tf::transformEigenToMsg(Eigen::Isometry3d(resultPose), addTF_srv.request.pose.transform);
    addTF_srv.request.pose.header.stamp = ros::Time::now();
    addTF_srv.request.pose.header.frame_id = parent_frame;
    addTF_srv.request.pose.child_frame_id = target_frame;
    addTF_srv.request.relative = false;
    client_store_tf.call(addTF_srv);

    std::cout << "Press enter to end the Freedrive..." << std::endl;
    std::cin.ignore(); //wait for enter press
    // Turn off freedrive
    freedrive_srv.request.IO = false;
    client_f.call(freedrive_srv);
    /*
    UNCOMMENT THIS FOR LOGGING
    ros::Duration(5).sleep();
    geometry_msgs::Transform transfrom_touch;
    transfrom_touch = tfBuffer.lookupTransform("base_link", "task_board",ros::Time(0)).transform;

    float orentierung;
    std::cout << "Geben sie die Orientierung das Taskboards in Grad an!" << std::endl;
    std::cout << "M5 zu Robot = 0 / rotation im Uhrzeigersin 45°" << std::endl;
    std::cin >> orentierung;

    std::ofstream myfile;
    myfile.open ("/home/robothon/Robothon/Detection_2023.csv", std::ios::app);
    myfile << orentierung << ",";
    myfile << transfrom_touch.translation.x << "," << transfrom_touch.translation.y << "," << transfrom_touch.translation.z << ",";
    myfile << transfrom_touch.rotation.x << "," << transfrom_touch.rotation.y << "," << transfrom_touch.rotation.z << "," << transfrom_touch.rotation.w << ",";
    myfile << transfrom_detection.translation.x << "," << transfrom_detection.translation.y << "," << transfrom_detection.translation.z << ",";
    myfile << transfrom_detection.rotation.x << "," << transfrom_detection.rotation.y << "," << transfrom_detection.rotation.z << "," << transfrom_detection.rotation.w << "\n";
    myfile.close();
    */

    return 0;
}
