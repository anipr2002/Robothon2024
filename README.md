# msvc2024_setup
# Team: RoboPig

## General Information of Setup
### Hardware Requirements
#### Robot Setup:
- UR10e (Polyscope 5.11)
- Robotiq HandE without Rubber Tips
- IDS U3-CP3800 Rev2.2 Camera (for detection of taskboard and certain components in BYOD)
- Intel Realsense D435i Camera with 3D Printed mount facing -y of TCP (for Slider Task)
#### Additional Requirements for Taskboard Operation
- 3D Printed Hook
#### Additional Requirements for BYOD Operation
- 3D Printed Mounts (tiles, tile fixtures, smartphone rack, hot-air blower holder with vents, suction pad holders, workstation, sorting boxes, 5 foam pads)
  !!Note:!! Environment friendly biodegradable PLA (3DJake ecoPLA) filament is used for 3D printing the mounts
- 2 Suction Pads: for moving and fixing a smartphone
- Prying Tool: to remove backcover if necessary
- Hot-air Blower: to heat up the back side of a smarthpone to melt the resin and ease the removal of backcover
  !!Note:!! An e-waste fully-functioning hair dryer is been reused here as an hot-air blower
- (Multimeter: as a future scope for taking certain measurements on the back-panel of a smartphone if the backcover is removed successfully)

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

## Software Dependencies
### ROS Noetic
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
### realsense-ros
```
git clone https://github.com/IntelRealSense/realsense-ros.git
```
### IDS-peak
```
https://en.ids-imaging.com/downloads.html
```
### OpenCV
```
sudo apt install libopencv-dev python3-opencv
```
### JSON support
```
sudo apt install nlohmann-json3-dev
```
### CGAL
```
sudo apt install libcgal-dev
```
### Eigen
```
eigen.tuxfamily.org
```

## Starting the System
Run Launcher: `roslaunch msvc2024_setup taskboard_core.launch`
    - Note: Make sure that the robot IP in the launchfile is correct

To run the task scheduler:
`rosrun msvc2024_setup msvc2024_setup_taskboard`
- !!WARNING!! robot will move immediately to home position

To run the point recording script:
`rosrun msvc2024_setup msvc2024_setup_point_recording`

To call the taskboard camera detection service:
    - Run `rqt`
    - Select sift_board_detection service and call it

To run touch detect service:
`rosrun msvc2024_setup msvc2024_setup_taskboard`
    - !!Attention!! robot is immediately aligning to z
    - Touch so that +y of tcp looks away from taskboard.
    - Touch longside without cable holder
    - Touch shortside next to probe
    - Touch top of the board
    - Move robot away
    - Confirm position in rviz

### Starting the UI Dashboard:
- [Follow the guide here](ui/README.md)

## Robot Trajectory Logging
1. Uncomment 2x call_log() in task_board_scheduler.cpp
2. Change path in tasks.cpp
After each run change filename in first call_log() in task_board_scheduler.cpp
