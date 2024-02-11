# Scripts
## Setup
### Computer vision stuff
- Make sure that you have camera passthrough into your VM (if you are using
   a VM)
- Make sure that you have installed the usb-cam package
  ```sh
  sudo apt-get install ros-noetic-usb-cam
  ```
- Make sure that the correct video device is selected in RobotArmPackages/rbx1_scripts/launch/camera_init.launch
  - You can check what the different video devices are by running
  ```sh
  ffplay /dev/video<number>
  ```
