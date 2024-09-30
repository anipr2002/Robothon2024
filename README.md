# MSVC2024_Setup_2024 RoboPig

## General Information
### Used Hardware
Required:
- UR5e (Polyscope 5.13)
- Robotiq HandE with Rubber Tips
- 3D Printed Hook
- IDS Camera (for Board Detection)
- Intel Realsense D435i with 3D Printed Holder facing -y of TCP (for Slider Task)

Optional:
- Googly Eyes

### UR5e Settings
- TCP
    - X: 0mm
    - Y: 0mm
    - Z: 148mm
- Payload
    - With Realsense
        - Mass: 1.35kg
        - CX: 3mm
        - CY: 2mm
        - CZ: 50mm
    - Without Realsense
        -Mass: 1.06kg
        - CX: 0mm
        - CY: 0mm
        - CZ: 59mm

## How to install dependencies
### Json
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

## How to start
Run Launcher: `roslaunch MSVC2024_Setup_2024 taskboard_core.launch`
    - Make sure that the robot ip in the launchfile is correct

To run the taskboard scheduler:
`rosrun MSVC2024_Setup_2024 MSVC2024_Setup_2024_taskboard`
- !!WARNING!! robot will move immediately to home position

To run the point recording script:
`rosrun MSVC2024_Setup_2024 MSVC2024_Setup_2024_point_recording`

To call the taskboard detection service:
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

To perform hand-eye calibration of the IDS camera:
    - Run taskboard launcher
    - Run launcher `roslaunch camera_robot_calibration capture_points.launch`
    - `cd Robothon/src/extrinsic-calib-tools/camera_robot_calibration/scripts`
    - `python3 camera_robot_calibration.py` here you will see the output of the services
    - Go to Desktop and run `python3 cowCamInfoPub.py`
    - Open a new terminal in the same location and run `python3 diaMarkerDet.py`
    - !! If there are errors with opencv version, run this `pip3 install --upgrade opencv-contrib-python`
    - Attach the Charuco board to the gripper and move the tcp such that the board is visible in the camera
    - Run `rqt` in a terminal and open read_tf service
    - Move the board slightly and each time call the read_tf service. Do this at least 5 times
    - Call the `computer_frames` service to get the camera extrinsic matrix in terminal
    - It gives you the rotation matrix R and translation vector t, so convert this into a 4x4 transformation matrix [  R   transpose(t)],
                                                  [0 0 0       1     ]
    - Update the ids_cam transform in tf2.json file with this matrix


## How to Log data
1. Uncomment 2x call_log() in task_board_scheduler.cpp
2. Change path in task_board_tasks.cpp
After each run change filename in first call_log() in task_board_scheduler.cpp
