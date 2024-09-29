/**
 * @file point_recording.cpp
 * @authors Adrian Müller (adrian.mueller@study.thws.de), 
 *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
 *          Usama Ali (usama.ali@study.thws.de)
 * @brief Program for recording points by using freedrive
 * @version 0.1
 * @date 2023-03-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <filesystem>

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <termios.h>

#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Joy.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/array.hpp>

#include <ur_ros_driver/SetFreedrive.h>
#include <ur_ros_driver/SetCartTarget.h>
#include <robothon2023/AddTf2.h>
#include <geometry_msgs/TransformStamped.h>
#include <ur_ros_driver/StartJog.h>
#include <ur_ros_driver/JogControl.h>
#include <ur_ros_driver/SetForceTarget.h>
#include <ur_ros_driver/SetGripper.h>
#include <tf2_ros/transform_listener.h>

//Terminal Styling with GUM
#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <sstream>
#include <cstdio>
#include <memory>

boost::array<long int, 6> free_axes = {1, 1, 1, 1, 1, 1};

geometry_msgs::TransformStamped tcp_pose;

int main_menu();
int free_drive_menu();
int con_menu();
void jog_menu(int frame, double speed, double step);

void tcpCallback(const geometry_msgs::TransformStamped& msg)
{
    tcp_pose = msg;
}



std::string command_exec(const char* cmd){
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    if(result.back() == '\n'){
        result.pop_back();
    }
    if(!result.empty()){
        return result;
    }
}


bool confirm_command_exec(const char* cmd){
    std::string command = std::string(cmd) + " && echo 0 || echo 1";
    std::string result = command_exec(command.c_str());

    try{
        int exit_status = std::stoi(result);
        return exit_status == 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid argument : Unable to convert the result to stoi" << std::endl;
        return false;
    }
}

bool check_app (const std::string& last_position, const std::string& current_position)
{

    std::cout<< "last pos : " << last_position <<" Current Pos : " <<current_position << std::endl;
    if(last_position.size() >= 4 && (last_position.find("_app") != std::string::npos)) {
        return true;
    }

    std::vector<std::string> ignore_pos = {"cable", "wind", "home", "hover", "_app"};

    for(const auto& word: ignore_pos){
        if(current_position.find(word) != std::string::npos){
            return true;
        }
    }

    return false;
}

std::string last_position = "home";

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "point_recorder");
    ros::NodeHandle n;

    ros::ServiceClient client_g = n.serviceClient<ur_ros_driver::SetGripper>("/ur_hardware_interface/robotiq/set_gripper");
    ros::ServiceClient client_f = n.serviceClient<ur_ros_driver::SetFreedrive>("/ur_hardware_interface/set_freedive");
    ros::ServiceClient client_fm = n.serviceClient<ur_ros_driver::SetForceTarget>("/ur_hardware_interface/set_force_mode");
    ros::ServiceClient client_t = n.serviceClient<ur_ros_driver::SetCartTarget>("/ur_hardware_interface/set_cart_target");
    ros::ServiceClient client_tf = n.serviceClient<robothon2023::AddTf2>("/store_tf");
    ros::ServiceClient jog_client = n.serviceClient<ur_ros_driver::StartJog>("/ur_hardware_interface/start_jog");
    
    ros::Subscriber sub = n.subscribe("/ur_hardware_interface/tcp_pose", 1000, tcpCallback);

    ros::Publisher jog_control_pub = n.advertise<ur_ros_driver::JogControl>("/jog_control",1000);
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    robothon2023::AddTf2 addTF_srv;
    ur_ros_driver::SetFreedrive freedrive_srv;
    freedrive_srv.request.IO = true;
    freedrive_srv.request.free_axes = free_axes;
    ur_ros_driver::SetCartTarget target_srv;
    target_srv.request.mode = 1;
    target_srv.request.speed = 0.25;
    target_srv.request.acceleration = 0.25;
    target_srv.request.asynchronous = false;
    ur_ros_driver::SetForceTarget force_srv;
    force_srv.request.type = 2;
    force_srv.request.limits = {1,1,1,1,1,1};
    ur_ros_driver::SetGripper gripper_srv;
    gripper_srv.request.force = 0;
    gripper_srv.request.asynchronous = false;
    gripper_srv.request.speed = 50;

    int main;
    bool relation;
    double offset;
    std::string point_ID;


    //track last poisition





    client_f.call(freedrive_srv);

    do{


        int menu_option = main_menu();

            switch (menu_option)
        {
        case 1:
            {
                // std::cout << "Punktname: ";
                std::string point_name_command = "gum input --placeholder \"Point Name\"";
                point_ID = command_exec(point_name_command.c_str());
                // std::cout << "Add z Offset ? [mm] ";
                // std::cin >> offset;
                std::string offset_command = "gum input --placeholder \"Add z Offset ? [mm]\"";
                offset = std::stod(command_exec(offset_command.c_str()));
                // std::cout << "Is this Pose in relation to the target ? [1/0] ";
                // std::cin >> relation;
                std::string relation_command = "gum input --placeholder \"Is this Pose in relation to the target ? [1/0]\"";
                relation = std::stoi(command_exec(relation_command.c_str()));
                addTF_srv.request.pose = tcp_pose;
                addTF_srv.request.pose.header.stamp = ros::Time::now();
                addTF_srv.request.pose.header.frame_id = "base";
                addTF_srv.request.pose.child_frame_id = point_ID;
                addTF_srv.request.relative = relation;
                if(relation)
                {
                    addTF_srv.request.pose.header.frame_id = "task_board";
                }
                client_tf.call(addTF_srv);
                if(offset != 0) {
                    addTF_srv.request.pose.transform = geometry_msgs::Transform();
                    addTF_srv.request.pose.transform.translation.z = -1 * offset / 1000;
                    addTF_srv.request.pose.transform.rotation.w = 1;
                    addTF_srv.request.pose.header.frame_id = point_ID;
                    addTF_srv.request.pose.child_frame_id = point_ID + "_app";
                    addTF_srv.request.relative = false;
                    client_tf.call(addTF_srv);
                }
                break;
            }
        case 2:
            {

                int axis;
                axis = free_drive_menu();
                // std::cin >> axis;

                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);

                if(axis < 4 && axis > 0)
                {
                    freedrive_srv.request.feature = axis-1;
                    if(axis == 3) freedrive_srv.request.custom_frame = tfBuffer.lookupTransform("base", "task_board",ros::Time(0)).transform;
                }

                freedrive_srv.request.IO = true;
                freedrive_srv.request.free_axes = free_axes;
                client_f.call(freedrive_srv);
                break;
            }
        case 3:
            {
                const char* placeholders[6] = {"x", "y", "z", "r", "p", "y"};
                int con;
                // std::cin >> con;
                con = con_menu();
                switch (con)
                {
                case 0:
                    break;
                case 1:
                    // std::cout << "Put in values for x,y,z,r,p,y (1/0)" << std::endl;
                    // for(int i=0;i<6;i++){std::cin >> free_axes[i];}
                    for (int i = 0; i < 6; i++) {
                        std::string command = std::string("gum input --placeholder \"") + placeholders[i] + "\"";
                        free_axes[i] = std::stoi(command_exec(command.c_str()));
                    }
                    break;
                case 2:
                    // std::cout << "Put in values for x,y,z (1/0)" << std::endl;
                    // for(int i=0;i<3;i++){std::cin >> free_axes[i];}
                    for (int i = 0; i < 3; i++) {
                        std::string command = std::string("gum input --placeholder \"") + placeholders[i] + "\"";
                        free_axes[i] = std::stoi(command_exec(command.c_str()));
                    }
                    break;
                case 3:
                    // std::cout << "Put in values for r,p,y (1/0)" << std::endl;
                    // for(int i=3;i<6;i++){std::cin >> free_axes[i];}
                    for (int i = 3; i < 6; i++) {
                        std::string command = std::string("gum input --placeholder \"") + placeholders[i] + "\"";
                        free_axes[i] = std::stoi(command_exec(command.c_str()));
                    }
                    break;

                default:
                    break;
                }
                //rtde->freedrive(0);
                //rtde->freedrive(1,free_axes);
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);
                freedrive_srv.request.IO = true;
                freedrive_srv.request.free_axes = free_axes;
                client_f.call(freedrive_srv);
                break;
            }
        case 4:
            {  
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);
                geometry_msgs::Transform taskboard = tfBuffer.lookupTransform("base", "task_board" ,ros::Time(0)).transform; 
                bool running = true;
                int feature = 1;
                //double step = 0.001;
                double step = 0.001;
                double stepIncrement = 0.001;
                int time = 1;

                //ur_ros_driver::JogControl msg;
                //ur_ros_driver::StartJog srv;
                //srv.request.IO = true;
                //jog_client.call(srv);

                int kfd = 0;
                struct termios cooked, raw;
                tcgetattr(kfd, &cooked);
                memcpy(&raw, &cooked, sizeof(struct termios));
                
                raw.c_lflag &=~ (ICANON | ECHO);
                // Setting a new line, then end of file                         
                raw.c_cc[VEOL] = 1;
                raw.c_cc[VEOF] = 2;
                tcsetattr(kfd, TCSANOW, &raw);

                //system("stty raw -echo");
                jog_menu(feature, step, stepIncrement);

                while(ros::ok() && running){
                    Eigen::Matrix4d deltaP = Eigen::Matrix4d::Identity();
                    
                    //char c = getc(stdin);
                    char c;
                    if(read(kfd, &c, 1) < 0)
                    {
                      c = '0';
                    }
                    ROS_INFO("value: 0x%02X\n", c);
                    //ros::Duration(2).sleep();
                    switch (c)
                    {
                    case '0':
                        //system("stty cooked echo");
                        tcsetattr(kfd, TCSANOW, &cooked);
                        running = false;
                        break;
                    case 'a':
                        deltaP(0,3) -= step;
                        break;
                    case 'd':
                        deltaP(0,3) += step;
                        break;
                    case 's':
                        deltaP(1,3) -= step;
                        break;
                    case 'w':
                        deltaP(1,3) += step;
                        break;
                    case 'e':
                        deltaP(2,3) -= step;
                        break;
                    case 'q':
                        deltaP(2,3) += step;
                        break;        
                    case 'j':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitX()).matrix();
                        break;
                    case 'l':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitX()).matrix();
                        break;
                    case 'k':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitY()).matrix();
                        break;
                    case 'i':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitY()).matrix();
                        break;
                    case 'o':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(-step * M_PI, Eigen::Vector3d::UnitZ()).matrix();
                        break;
                    case 'u':
                        deltaP.block<3,3>(0,0) = Eigen::AngleAxisd(step * M_PI, Eigen::Vector3d::UnitZ()).matrix();
                        break;
                    case 'n':
                        step += stepIncrement;
                        break;
                    case 'm':
                        step -= stepIncrement;
                        break;
                    case '1':
                        stepIncrement = 0.001;
                        // stepIncrement = 1;
                        // time = 0.1;
                        break;
                    case '2':
                        stepIncrement = 0.005;
                        // stepIncrement = 2.5;
                        // time = 0.2;
                        break;
                    case '3':
                        stepIncrement = 0.01;
                        // time = 0.3;
                        // stepIncrement = 5;
                        break;
                    case '4':
                        stepIncrement = 0.025;
                        // time = 0.5;
                        // stepIncrement = 7.5;
                        break;
                    case 'b':
                        feature = (feature+1) %2;
                        break;
                    default:
                        break;
                    }
                    //msg.stamp = ros::Time::now();
                    //msg.feature = feature;
                    //msg.speeds = speeds;
                    //jog_control_pub.publish(msg);
                    
                    //std::cout << "set Frame" << std::endl;
                    geometry_msgs::Transform null;
                    null.rotation.w = 0;
                    null.rotation.x = 0;
                    null.rotation.y = 0;
                    null.rotation.z = 0;
                    Eigen::Isometry3d robotPose_Iso;
                    tf::transformMsgToEigen(tcp_pose.transform, robotPose_Iso);
                    Eigen::Matrix4d robotPose_Mat = robotPose_Iso.matrix();
                    Eigen::Matrix4d resPose; 
                    if(feature == 0){
                        resPose = robotPose_Mat;
                        resPose(0,3) += deltaP(0, 3);
                        resPose(1,3) += deltaP(1, 3);
                        resPose(2,3) += deltaP(2, 3);
                        resPose.block<3,3>(0,0) = deltaP.block<3,3>(0,0) * resPose.block<3,3>(0,0);
                    }
                    else if(feature == 1){
                        resPose = robotPose_Mat * deltaP;
                    }
                    else if(feature == 2) {//TODO
                        Eigen::Isometry3d taskboard_Iso;
                        tf::transformMsgToEigen(taskboard, taskboard_Iso);
                        Eigen::Matrix4d taskboard_rot = Eigen::Matrix4d::Identity();
                        taskboard_rot.block<3,3>(0,0) = taskboard_Iso.matrix().block<3,3>(0,0);

                        deltaP = taskboard_rot*deltaP;


                        resPose = robotPose_Mat;
                        resPose(0,3) += deltaP(0, 3);
                        resPose(1,3) += deltaP(1, 3);
                        resPose(2,3) += deltaP(2, 3);
                        resPose.block<3,3>(0,0) = deltaP.block<3,3>(0,0) * resPose.block<3,3>(0,0);
                    }
                    
                    tf::transformEigenToMsg(Eigen::Isometry3d(resPose), target_srv.request.cartesian_goal);
                    client_t.call(target_srv);
                    
                    jog_menu(feature, step, stepIncrement);
                    //ros::Duration(0.05).sleep();
                }
                
                //srv.request.IO = false;
                //jog_client.call(srv);
                force_srv.request.IO = false;
                client_fm.call(force_srv);
                ros::Duration(1.0).sleep();
                freedrive_srv.request.IO = true;
                client_f.call(freedrive_srv);
                break;
            
            }
        case 5:
            {
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);

                system("clear");
                // std::cout << "------------------------" << std::endl;
                // std::cout << "Move to TF" << std::endl;
                // std::cout << "------------------------" << std::endl;
                    std::string heading_command = "gum style --border normal --padding \"0.5 1 \" --border-foreground \"#FFA500\" \"$(gum style --foreground \"#FFA500\" 'Move to TF')\"";
            std::string heading = command_exec(heading_command.c_str());
            std::cout << heading << std::endl;



                //improvised Menu - Shubham
                std::vector<std::string> frames;
                tfBuffer._getFrameStrings(frames);
                std::vector<std::pair<int, std::string>> frame_vector;
                std::vector<std::string> filter_strings = {"link", "tool", "flange", "tcp", "erklaerung", "cam", "wind"}; // Vector of substrings to filter out
                int id = 0;
                for (const auto& frame : frames) {
                    bool should_filter = false;
                    for (const auto& filter : filter_strings) {
                        if (frame.find(filter) != std::string::npos) {
                            should_filter = true;
                            break;
                        }
                    }
                    if (!should_filter && frame != "base") {
                        frame_vector.push_back(std::make_pair(id, frame));
                        //std::cout << "ID: " << id << " - Frame: " << frame << std::endl;
                        id++;
                    }
                }
                // Print the frames in a table-like format
                std::cout << "ID\tFrame" << std::endl;
                std::cout << "------------------------" << std::endl;
                for (const auto& frame_pair : frame_vector) {
                    std::cout << frame_pair.first << "\t" << frame_pair.second << std::endl;
                }

                // std::cout << "Please enter the ID of the target frame" << std::endl;
                // int tf_id;
                // std::cin >> tf_id;

                std::string command = "gum input --placeholder \"Please enter the ID of the target frame\"";
                int tf_id = std::stoi(command_exec(command.c_str()));

                if (tf_id >= 0 && tf_id < frame_vector.size()) {
                    std::string tf = frame_vector[tf_id].second;
                    try {
                        target_srv.request.cartesian_goal = tfBuffer.lookupTransform("base", tf, ros::Time(0)).transform;
                    } catch (tf2::TransformException &ex) {
                        ROS_WARN_STREAM(ex.what());
                        ros::Duration(2.0).sleep();
                        freedrive_srv.request.IO = true;
                        client_f.call(freedrive_srv);
                        break;
                    }
                } else {
                    std::cout << "Invalid ID entered." << std::endl;
                    freedrive_srv.request.IO = true;
                    client_f.call(freedrive_srv);
                    break;
                }
                bool execute = true;

                if(execute)
                    client_t.call(target_srv);

                freedrive_srv.request.IO = true;
                client_f.call(freedrive_srv);
                break;

            }

            case 6:
            {
                freedrive_srv.request.IO = false;
                client_f.call(freedrive_srv);

                system("clear");
                std::string heading_command = "gum style --border normal --padding \"0.5 1 \" --border-foreground \"#FFA500\" \"$(gum style --foreground \"#FFA500\" 'Move to TF')\"";
                std::string heading = command_exec(heading_command.c_str());
                std::cout << heading << std::endl;

                std::string command = "gum input --placeholder \"Please enter the Name of the target frame\"";
                std::string tf = command_exec(command.c_str());
                tf.erase(tf.find_last_not_of(" \n\r\t")+1);

                std::string current_position = tf;

                if(!check_app(last_position, current_position)){
                    if(!confirm_command_exec("gum confirm \"Are You sure you want to override\"")){
                        break;
                    }
                }

                last_position.clear();
                last_position = current_position;
                
                try{
                target_srv.request.cartesian_goal = tfBuffer.lookupTransform("base", tf ,ros::Time(0)).transform;
                }
                catch(tf2::TransformException &ex)
                {
                    ROS_WARN_STREAM(ex.what());
                    ros::Duration(2.0).sleep();
                    freedrive_srv.request.IO = true;
                    client_f.call(freedrive_srv);
                    break;
                }
                bool execute = true;
                //std::cin >> execute;
                if(execute)
                    client_t.call(target_srv);

                freedrive_srv.request.IO = true;
                client_f.call(freedrive_srv);
                break;
            }

            case 7:
            {
                system("clear");
                std::vector<std::string> options = {"1 - Gripper Open", "2 - Gripper Close", "0 - back"};
                std::string heading_command = "gum style --border normal --padding \"0.5 1 \" --border-foreground \"#FFA500\" \"$(gum style --foreground \"#FFA500\" 'Gripper')\"";
                std::string heading = command_exec(heading_command.c_str());
                std::cout << heading << std::endl;


                std::string command = "gum choose";
                for(const auto& option : options)
                {
                    command += " \"" + option + "\"";
                }

                std::string result = command_exec(command.c_str());
                result.erase(result.find_last_not_of(" \n\r\t")+1);
                int selection = std::stoi(result);

                switch (selection)
                {
                case 0:
                    break;
                case 1:
                    gripper_srv.request.position = 100;
                    gripper_srv.request.force = 50;
                    break;
                case 2:
                    gripper_srv.request.position = 0;
                    gripper_srv.request.force = 100;
                    break;

                default:
                    break;
                }

                client_g.call(gripper_srv);
            }
        default:
            break;
        }

    }while(ros::ok() && main != 0 );

    freedrive_srv.request.IO = false;
    client_f.call(freedrive_srv);

}


int main_menu()
{
    std::vector<std::string> options = {"1 - save point", "2 - set axis", "3 - set constrains", "4 - move remotely", "5 - move to tf (by ID)", "6 - move to tf (by name)", "7 - move Gripper", "0 - Quit"};

    system("clear");
    // std::cout << "------------------------" << std::endl;
    // std::cout << "Main Menu" << std::endl;
    // std::cout << "------------------------" << std::endl;
    // std::cout << "Please make your selection" << std::endl;

    std::string heading_command = "gum style --border normal --margin \"1\" --padding \"1 2\" --border-foreground \"#FFA500\" \"$(gum style --foreground \"#FFA500\" 'MAIN MENU')\"";
    std::string heading = command_exec(heading_command.c_str());

    std::cout << heading << std::endl;

    std::string subheading_command = "gum style --foreground \"#FFA500\" \"Please make your selection\"";
    std::string subheading = command_exec(subheading_command.c_str());
    std::cout << subheading << std::endl;

    std::string command = "gum choose";
    for(const auto& option : options)
    {
        command += " \"" + option + "\"";
    }

    std::string result = command_exec(command.c_str());
    result.erase(result.find_last_not_of(" \n\r\t")+1);
    int selection = std::stoi(result);
    return selection;

}

int free_drive_menu()
{
   std::vector<std::string> options = {"1 - Base", "2 - TCP", "3 - Taskboard", "0 - back"};

    system("clear");
    // std::cout << "------------------------" << std::endl;
    // std::cout << "Freedrive Frame" << std::endl;
    // std::cout << "------------------------" << std::endl;


    std::string heading_command = "gum style --border normal --padding \"0.5 1 \" --border-foreground \"#FFA500\" \"$(gum style --foreground \"#FFA500\" 'Freedrive Frame')\"";
    std::string heading = command_exec(heading_command.c_str());

    std::cout << heading << std::endl;

    std::string subheading_command = "gum style --foreground \"#FFA500\" \"Please make your selection\"";
    std::string subheading = command_exec(subheading_command.c_str());
    std::cout << subheading << std::endl;

    std::string command = "gum choose";
    for(const auto& option : options)
    {
        command += " \"" + option + "\"";
    }

    std::string result = command_exec(command.c_str());
    result.erase(result.find_last_not_of(" \n\r\t")+1);
    int selection = std::stoi(result);
    return selection;
}

int con_menu()
{
    std::vector<std::string> options = {"1 - all", "2 - position", "3 - orientaion", "0 - back"};

    system("clear");
    // std::cout << "------------------------" << std::endl;
    // std::cout << "Freedrive Constrains" << std::endl;
    // std::cout << "------------------------" << std::endl;
    // std::cout << "Please choose the constrains" << std::endl;

    std::string heading_command = "gum style --border normal --padding \"0.5 1 \" --border-foreground \"#FFA500\" \"$(gum style --foreground \"#FFA500\" 'Freedrive Contraints')\"";
    std::string heading = command_exec(heading_command.c_str());

    std::cout << heading << std::endl;

    std::string subheading_command = "gum style --foreground \"#FFA500\" \"Please choose the contraints\"";
    std::string subheading = command_exec(subheading_command.c_str());
    std::cout << subheading << std::endl;


    std::string command = "gum choose";
    for(const auto& option : options)
    {
        command += " \"" + option + "\"";
    }

    std::string result = command_exec(command.c_str());
    result.erase(result.find_last_not_of(" \n\r\t")+1);
    int selection = std::stoi(result);
    return selection;
}

void jog_menu(int feature, double speed, double step)
{
    std::vector<std::string> frames({"Base", "Tool", "Task Board"});
    system("clear");
    std::cout << "------------------------" << std::endl;
    std::cout << "\rRobot remote control" << std::endl << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "\rControls:" << std::endl;
    std::cout << "\r\tTranslation:" << std::endl;
    std::cout << "\r\t\tX: + a   - d" << std::endl;
    std::cout << "\r\t\tY: + w   - s" << std::endl;
    std::cout << "\r\t\tZ: + q   - e" << std::endl;
    std::cout << "\r\tRotation:" << std::endl;
    std::cout << "\r\t\tX: + j   - l" << std::endl;
    std::cout << "\r\t\tY: + i   - k" << std::endl;
    std::cout << "\r\t\tZ: + u   - o" << std::endl;
    std::cout << "\r\tSpeed: Up n, Down m" << std::endl;
    std::cout << "\r\tStep: 1mm:1, 5mm:2, 10mm:3, 25mm:4" << std::endl;
    std::cout << "\r\tSwitch Frame: b" << std::endl;
    std::cout << "\r\tQuit: 0" << std::endl;
    std::cout << "------------------------" << std::endl;
    //std::cout << std::endl << std::endl<< std::endl;
    std::cout << "\rX: " << tcp_pose.transform.translation.x << "\t Y: " << tcp_pose.transform.translation.y << "\t Z: " << tcp_pose.transform.translation.z << std::endl <<"\rRX: " << tcp_pose.transform.rotation.x << "\t RY: " << tcp_pose.transform.rotation.y << "\t RZ: "<< tcp_pose.transform.rotation.z << "\t RW: "<< tcp_pose.transform.rotation.w << std::endl;
    std::cout << "\rSpeed: " << speed << "\tStep: "<< step << std::endl;
    std::cout << "\rFrame: " << frames[feature] << std::endl;
    std::cout << "------------------------" << std::endl;
}

