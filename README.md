# MSVC2024_Setup_2024
# Team: RoboPig

## General Information of Setup
### Hardware Requirements:
- UR10e (Polyscope 5.11)
- Robotiq HandE without Rubber Tips
- 3D Printed Hook
- IDS U3-CP3800 Rev2.2 Camera (for detection of taskboard and certain components in BYOD)
- Intel Realsense D435i Camera with 3D Printed mount facing -y of TCP (for Slider Task)

Optional:
- Googly Eyes

### UR10e Settings
- TCP
    - X: 0mm
    - Y: 0mm
    - Z: 148mm
- Payload
    - With Realsense Camera and mount
        - Mass: 1.35kg
        - Center of Gravity: 
            - CX: 3mm
            - CY: 2mm
            - CZ: 50mm
    - Without Realsense Camera and mount
        - Mass: 1.06kg
        - Center of Gravity:
            - CX: 0mm
            - CY: 0mm
            - CZ: 59mm

## Installing Dependencies
### Json support
`sudo apt install nlohmann-json3-dev`
### cgal
`sudo apt install libcgal-dev`
### ur_rtde
https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#linux-ubuntu-and-macos
```
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git checkout v1.5.5
git submodule update --init --recursive
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j
sudo make install
```

## Starting the System
Run Launcher: `roslaunch MSVC2024_Setup_2024 taskboard_core.launch`
    - Note: Make sure that the robot IP in the launchfile is correct

To run the task scheduler:
`rosrun MSVC2024_Setup_2024 MSVC2024_Setup_2024_taskboard`
- !!WARNING!! robot will move immediately to home position

To run the UI Dashboard:
-

To run the point recording script:
`rosrun MSVC2024_Setup_2024 MSVC2024_Setup_2024_point_recording`

To call the taskboard camera detection service:
    - Run `rqt`
    - Select sift_board_detection service and call it

To run touch detect:
`rosrun MSVC2024_Setup_2024 MSVC2024_Setup_2024_taskboard`
    - !!Attention!! robot is immediately aligning to z
    - Touch so that +y of tcp looks away from taskboard.
    - Touch longside without cable holder
    - Touch shortside next to probe
    - Touch top of the board
    - Move robot away
    - Confirm position in rviz


## Robot Trajectory Logging
1. Uncomment 2x call_log() in task_board_scheduler.cpp
2. Change path in task_board_tasks.cpp
After each run change filename in first call_log() in task_board_scheduler.cpp
