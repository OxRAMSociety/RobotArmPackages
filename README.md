# RobotArmPackages
ROS Packages related to the OxRAM Robot Arm project.

To use, `git clone --recursive https://github.com/OxRAMSociety/RobotArmPackages.git` into the `src` folder of your workspace.

## Installation:
### Computer Vision
Go to RobotArmPackages and run
```bash
./cv_setup.sh
```

## Running the code:
In separate terminals
1. Start the main scripts
  ```bash
  roslaunch rbx1_scripts rbx1_init.launch
  ```

2. Start the camera (select the appropriate video device and put it instead of
   video0)
  ```bash
  roslaunch rbx1_computer_vision camera_init.launch video_dev:=/dev/video0
  ```

3. Start YOLO
  ```bash
  roslaunch yolov5_ros yolov5.launch input_image_topic:=/camera/color/image_rect_color device:=cpu
  ```

