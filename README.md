# turtlebot_Odometry_walk

#### Go to point work based on imu and wheel odometry of kobuki.

Controlled by logitech F710  joystick(literately compile with all other logitech joystick)

*must launch **minimal bringup** and **tele/logitech_joystick.launch**

`roslaunch turtle_walk walk_to_point.launch`to start .

1. press **RB** while sending command
2. press right joy_con down to allow auto drive mode (press again to set hand mode)
3. **led 2** is red while in auto mode, green while in joy_control mode
4. press **RB+Y** to set target according to launch file
5. press **RB+X** to emergency stop
6. press **RB+B** to reset odometry of turtlebot2