// /**
//  * @file task_board_scheduler.cpp
//  * @authors Adrian Müller (adrian.mueller@study.thws.de), 
//  *          Maximilian Hornauer (maximilian.hornauer@study.thws.de),
//  *          Usama Ali (usama.ali@study.thws.de)
//  * @brief Program to schedule tasks
//  * @version 0.1
//  * @date 2023-04-11
//  * 
//  * @copyright Copyright (c) 2023
//  * 
//  */

// #include <ros/ros.h>
// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <string>
// #include <chrono>
// #include <robothon2023/task_board_tasks.hpp>

// #include <ur_ros_driver/ros_color_stream.hpp>

// const int btn_blue = 1;
// const int slider = 2;
// const int plug = 3;
// const int measure = 4;
// const int wind_cable = 5;
// const int btn_red = 6;
// const int home = 0;
// const int speed_test = 7;


// bool checkSequence(std::vector<int> sequence){
//     //measure
//     auto plug_it = std::find(sequence.begin(), sequence.end(), plug);
//     auto measure_it = std::find(sequence.begin(), sequence.end(), measure);
//     auto wind_it = std::find(sequence.begin(), sequence.end(), wind_cable);

//     if(wind_it < measure_it) return false;
//     if(measure_it < plug_it) return false;
//     return true;
// }

// // Argument handling added - Shubham
// // Function to split a comma-separated string argument and convert it to a vector
// // std::vector<int> parseTaskIDs(const std::string& task_ids_str) {
// //     std::vector<int> task_ids;
// //     std::stringstream ss(task_ids_str);
// //     std::string task_id;
    
// //     // Split the string by commas and convert each part to an integer
// //     while (getline(ss, task_id, ',')) {
// //         task_ids.push_back(std::stoi(task_id));  // Convert string to int and add to vector
// //     }
    
// //     return task_ids;
// // }



// int main(int argc, char* argv[])
// {
//     bool hook = false;
//     bool probe = false;
//     std::vector<double> time;

//     int ctn = 0;

//     ros::init(argc, argv, "taskboard_scheduler");
//     ros::NodeHandle n("~");
//     ros::AsyncSpinner spinner(2); 
//     spinner.start();

// // Argument handling added - Shubham
//     // std::vector<int> sequenz = {1};
//     // if (argc > 1) {
//     //     std::string task_ids_str = argv[1];
//     //     std::vector<int> sequenz = parseTaskIDs(task_ids_str);
//     // }
//     // else {
//     //     // default all tasks done
//     //     std::vector<int> sequenz = {1,2,3,4,5,6};
//     // }

//     std::vector<int> sequenz = {1,2,3,4,5,6};
//     //std::vector<int> sequenz = {3};

//     task_board_tasks task = task_board_tasks(n);

//     // uncomment to check the sequence in final run
//     // if(!checkSequence(sequenz)) {
//     //    ROS_ERROR("SEQUENCE INVALID!");
//     //    return 1;
//     // }

//     task.home();
//     ros::Duration(2).sleep();
    
    
//     int input;
//     std::cout << "Detection" << std::endl << " 0 - Cam" << std::endl << " 1 - Touch" << std::endl;
//     std::cin >> input;
    
//     if(input==1)
//     {
//         while(!task.call_touch_detection() && ros::ok());
//     }
//     else if(input == 0)
//     {
//         while(!task.call_board_detection() && ros::ok());
//     }
    
   
//     task.calculate_config();
//     geometry_msgs::Transform transfrom_detection;
//     transfrom_detection = task.tfBuffer_->lookupTransform("base_link", "task_board",ros::Time(0)).transform;
    
//     std::ofstream myfile1;
//     myfile1.open ("/home/robothon/Robothon/Detection_2023_new.csv", std::ios::app);
//     myfile1 << input << "," << task.config << ",";
//     myfile1 << transfrom_detection.translation.x << "," << transfrom_detection.translation.y << "," << transfrom_detection.translation.z << ",";
//     myfile1 << transfrom_detection.rotation.x << "," << transfrom_detection.rotation.y << "," << transfrom_detection.rotation.z << "," << transfrom_detection.rotation.w << "\n";
//     myfile1.close();
    
//     //std::cout << "Touch Taskboard before start" << std::endl;
    

//     //std::cout << "Touch Taskboard before start" << std::endl;
//     //std::cout << "!!!!!!!!!!!" << std::endl;
    

//     std::cout << "Press ENTER to start" << std::endl;
//     std::cin.ignore();
//     auto start = std::chrono::high_resolution_clock::now();
//     task.call_robot_time();
//     /** ENABLE THIS TO LOG DATA TO FILE **/
//     // task.call_log(true,"log_2");
    
//     // move to optimal start position
//     task.home();
    
//     int counter = 0;
//     sequenz.push_back(0);
//     do{
//         switch (sequenz[counter])
//         {
//         case btn_blue:
//             ROS_MAGENTA_STREAM("Task: btn blue");
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             task.press_button("blue");
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         case slider:
//             ROS_MAGENTA_STREAM("Task: slider");
//             task.center_slider();
//             task.move_slider();
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         case plug:
//             ROS_MAGENTA_STREAM("Task: plug");
//             task.move_plug("black","red");
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         case measure:
//             ROS_MAGENTA_STREAM("Task: grab probe");
//             if(task.grab_probe())
//             {
//                 probe = true;
//             }

//             ROS_MAGENTA_STREAM("Task: open door");
//             task.open_door();

//             ROS_MAGENTA_STREAM("Task: measure");
//             task.measure();
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());

//             ROS_MAGENTA_STREAM("Task: return probe");
//             task.call_robot_time();
//             if(task.return_probe())
//             {
//                 probe = false;
//             }
            
//             counter++;
//             break;
//         case wind_cable:
//             ROS_MAGENTA_STREAM("Task: get hook");
//             if(task.get_hook())
//             {
//                 hook = true;
//             }

//             ROS_MAGENTA_STREAM("Task: hook cable");
//             task.call_robot_time();
//             task.hook_cable();

//             ROS_MAGENTA_STREAM("Task: wind cable");
//             task.call_robot_time();
//             task.wind_cable();
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         case btn_red:
//             ROS_MAGENTA_STREAM("Task: btn red");
//             if(hook){
//                 task.press_button_hook();
//             } else {
//                 task.press_button("red");
//             }
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         case home:
//             if(hook)
//             {
//                 ROS_MAGENTA_STREAM("Task: return hook");
//                 task.get_hook(true);
//             }
//             if(probe)
//             {
//                 ROS_MAGENTA_STREAM("Task: return probe");
//                 task.return_probe();
//             }
//             task.config = 2;
//             ROS_MAGENTA_STREAM("Task: go home");
//             task.home();
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         case speed_test:
//             ROS_MAGENTA_STREAM("Task: speed test");
//             task.speed_test();
//             task.call_trajectory();
//             time.push_back(task.call_robot_time());
//             counter++;
//             break;
//         default:
//             break;
//         }

//     }while(counter <= sequenz.size()-1 &&ros::ok());

//     task.call_robot_time();
//     /** ENABLE THIS WHEN LOGGING TO FILE **/
//     // task.call_log(false," ");

//     auto ende  = std::chrono::high_resolution_clock::now();
//     std::cout << "----------------------------" << std::endl;
//     std::cout << std::endl << "Zeit: " << std::chrono::duration_cast<std::chrono::seconds>(ende-start).count() << " s" << std::endl;
//     std::cout << "----------------------------" << std::endl;

//     std::ofstream myfile;
//     myfile.open ("/home/robothon/Robothon/Task_times.csv", std::ios::app);
//     for(int i=1; i<time.size();i++)
//     {
//         myfile << "," << time[i]-time[i-1];
//     }

//     myfile << "\n";
//     myfile.close();
// }

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <robothon2023/task_board_tasks.hpp>

#include <ur_ros_driver/ros_color_stream.hpp>

#include "/home/robothon/Robothon/src/robothon2023/include/robothon2023/httplib.h"
#include <nlohmann/json.hpp>
#include <vector>
#include <algorithm>
#include <thread>
#include <sstream>

using namespace httplib;
using namespace std;
using json = nlohmann::json;

// Task identifiers
// const int btn_blue = 1;
// const int slider = 2;
// const int plug = 3;
// const int measure = 4;
// const int wind_cable = 5;
// const int btn_red = 6;
// const int home = 0;
const int M5 = 100;
const int btn_blue = 1;
const int slider = 2;
const int plug = 3;
const int measure = 4;
const int hook_grab = 8;
const int wind_cable = 5;
const int btn_red = 6;
const int home = 0;
const int speed_test = 7;

const int phone_charging = 10;
const int unscrew = 11;
const int suction = 12;
const int sphonegrab = 13;
const int hairdryer = 14;
const int switch_on = 15;
const int switch_off = 16;
const int prying = 17;
const int backcover = 18;
const int sort2 = 19;

bool checkSequence(const std::vector<int>& sequence) {
    auto plug_it = std::find(sequence.begin(), sequence.end(), plug);
    auto measure_it = std::find(sequence.begin(), sequence.end(), measure);
    auto wind_it = std::find(sequence.begin(), sequence.end(), wind_cable);

    if(wind_it < measure_it) return false;
    if(measure_it < plug_it) return false;
    return true;
}

std::vector<int> sequenz;

void startServer() {
    Server svr;

    svr.set_pre_routing_handler([](const Request &req, Response &res) {
    // Allow any origin
    res.set_header("Access-Control-Allow-Origin", "*");
    // Allow specific methods
    res.set_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    // Allow specific headers
    res.set_header("Access-Control-Allow-Headers", "Content-Type");

    // If it's an OPTIONS request, send a preflight response
    if (req.method == "OPTIONS") {
      res.status = 204;
      return Server::HandlerResponse::Handled;
    }

    return Server::HandlerResponse::Unhandled;
  });
    svr.Post("/post", [](const Request &req, Response &res) {
    try {
        json j = json::parse(req.body);
        sequenz.clear();

        // Iterate through the tasks array
        for (const auto& task : j["tasks"]) {
            if (task.is_string()) {
                // If the task is a string, convert it to an integer
                sequenz.push_back(std::stoi(task.get<std::string>()));
            } else if (task.is_number()) {
                // If the task is already a number, add it as is
                sequenz.push_back(task.get<int>());
            }
        }

        // Output the received sequence
        cout << "Received sequence: ";
        for (const int& task : sequenz) {
            cout << task << " ";
        }
        cout << endl;

        res.set_content("Data received successfully", "text/plain");
       
    } catch (const std::exception& e) {
        res.status = 400;
        res.set_content("Invalid data format", "text/plain");
    }
});

    ros::AsyncSpinner spinner(1); 
    spinner.start();
    svr.listen("0.0.0.0", 8080);

    cout << "Server started" << endl;
}

void runScheduler() {
    bool hook = false;
    bool probe = false;
    int ctn = 0;

    ros::NodeHandle n("~");
    task_board_tasks task = task_board_tasks(n);

    // if (!checkSequence(sequenz)) {
    //     ROS_ERROR("SEQUENCE INVALID!");
    //     return;
    // }

    task.home();


    // while(!task.call_board_detection() && ros::ok());
    // std::cout << "Press ENTER to start" << std::endl;
    // std::cin.ignore();
    auto start = std::chrono::high_resolution_clock::now();
    task.call_robot_time();
    task.call_log(true, "log_1");

    task.home();

    ROS_INFO_STREAM("Sequence size: " << sequenz.size());

    int counter = 0;
    sequenz.push_back(0);  // Add home task at the end
    do {
        switch (sequenz[counter]) {
            case M5:
                ROS_MAGENTA_STREAM("Task: start");
                task.call_robot_time();
                task.press_button("M5");
                counter++;
                break;
            case btn_blue:
                ROS_MAGENTA_STREAM("Task: btn blue");
                task.call_robot_time();
                task.press_button("blue");
                counter++;
                break;
            case slider:
                ROS_MAGENTA_STREAM("Task: slider");
                task.call_robot_time();
                // task.center_slider();
                task.move_slider();
                counter++;
                break;
            case plug:
                ROS_MAGENTA_STREAM("Task: plug");
                task.call_robot_time();
                task.move_plug("black", "red");
                counter++;
                break;
            case measure:
                ROS_MAGENTA_STREAM("Task: measure");
                task.call_robot_time();
                if (task.grab_probe()) {
                    probe = true;
                }
                task.open_door();
                task.measure();
                if (task.return_probe()) {
                    probe = false;
                }
                counter++;
                break;
            case wind_cable:
                ROS_MAGENTA_STREAM("Task: wind cable");
                task.call_robot_time();
                // if (task.get_hook_new()) {
                //     hook = true;
                // }
                if(task.get_hook())
                {
                    hook = true;
                }
                task.hook_cable();
                task.wind_cable();
                counter++;
                break;
            case btn_red:
                ROS_MAGENTA_STREAM("Task: btn red");
                task.call_robot_time();
                task.press_button("red");
                counter++;
                break;
            case home:
                // if (hook) {
                //     task.get_hook_new(true);
                // }
                // if (hook) {
                //     task.get_hook(true);
                // }   
                if (probe) {
                    task.return_probe();
                }
                task.config = 2;
                task.home();
                task.call_trajectory();
                counter++;
                break;
            case phone_charging:
                ROS_MAGENTA_STREAM("Task: phone charging");
                task.call_robot_time();
                task.charging_cable_insert("lightning");
                task.charging_cable_insert("usbc");
                counter++;
                break;

            case unscrew:
                ROS_MAGENTA_STREAM("Task: unscrew screw");
                task.call_robot_time();
                task.unscrew();
                counter++;
                break;


            case suction:
                ROS_MAGENTA_STREAM("Task: suction");
                task.call_robot_time();
                task.suction();
                counter++;
                break;

            case sphonegrab:
                ROS_MAGENTA_STREAM("Task: phone grab");
                task.call_robot_time();
                task.sphonegrab();
                counter++;
                break;

            case hairdryer:
                ROS_MAGENTA_STREAM("Task: hairdryer");
                task.call_robot_time();
                task.hairdryer();
                counter++;
                break;

            case switch_on:
                ROS_MAGENTA_STREAM("Task: switch on");
                task.call_robot_time();
                task.switch_on();
                counter++;
                break;

            case switch_off:
                ROS_MAGENTA_STREAM("Task: switch off");
                task.call_robot_time();
                task.switch_off();
                counter++;
                break;

            case backcover:
                ROS_MAGENTA_STREAM("Task: backcover");
                task.call_robot_time();
                task.backcover();
                counter++;
                break;

            case sort2:
                ROS_MAGENTA_STREAM("Task: sort2");
                task.call_robot_time();
                task.sort2();
                counter++;
                break;

            case prying:
                ROS_MAGENTA_STREAM("Task: prying");
                task.call_robot_time();
                task.prying();
                counter++;
                break;

            case hook_grab:
                ROS_MAGENTA_STREAM("Task: hook grab");
                task.call_robot_time();
                task.get_hook();
                counter++;
            default:
                break;
        }
    } while (counter <= sequenz.size() - 1 && ros::ok());

    task.call_robot_time();
    task.call_log(false, " ");

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "----------------------------" << std::endl;
    std::cout << std::endl << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " s" << std::endl;
    std::cout << "----------------------------" << std::endl;
}

int main(int argc, char* argv[]) {
    // Initialize ROS
    ros::init(argc, argv, "move_test");
    ros::NodeHandle n("~");    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <server_mode (0 or 1)>" << std::endl;
        return 1;
    }

    int server_mode = std::stoi(argv[1]);

    if (server_mode == 0) {
        // Manually set sequence
        sequenz = {5, 6}; // Set your manual sequence here
        runScheduler();
    } else if (server_mode == 1) {
        // Start server to receive sequence
        std::thread server_thread(startServer);
        server_thread.detach();

        // Wait for the task sequence to be set
        while (sequenz.empty() && ros::ok()) {
            ros::Duration(0.5).sleep();
        }

        runScheduler();
    } else {
        std::cerr << "Invalid server_mode. Use 0 for manual sequence or 1 for server mode." << std::endl;
        return 1;
    }

    return 0;
}