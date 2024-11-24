This is the primitive state machine that was developed during 2023-2024 by Zhiyu Zheng, Jiayi Yang and Tianrui Liu under the supervision of Emir Kocak. It has only three states: Idle, Getpose and movearm. Idle state receives the start robot signal, GetPose gets the target for the next move and find the target's pose, MoveArm moves the arm to the Pose that is identified by GetPose. \
To use it simply run 
```
roslaunch rbx1_state_machine rbx1_init.launch
``` 
and then in another terminal run 
```
rosrun rbx1_state_machine rbx1_fsm.py
```
