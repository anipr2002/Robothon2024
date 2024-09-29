// /**
//  * @file task_board_scheduler.cpp
//  * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
                Shubham Joshi(shubham.joshi@study.thws.de)\
                Medhansh Rath(medhansh.rath@study.thws.de)
//  * @brief Program to schedule tasks
//  * @version 0.1
//  * @date 2023-04-11
//  *
//  * @copyright Copyright (c) 2023
//  *
//  */

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <MSVC2024_Setup_2024/task_board_tasks.hpp>

#include <ur_ros_driver/ros_color_stream.hpp>

#include "/home/robothon/Robothon/src/MSVC2024_Setup_2024/include/MSVC2024_Setup_2024/httplib.h"
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
