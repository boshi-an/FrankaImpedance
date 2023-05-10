# A panda impedance control python interface

To install, you need ros and franka_ros already installed.

replace the original `franka_example_controller` directory provided by franka_ros with this `controller/franka_example_controller` directory.

rebuild the source of franka_ros.

Feel free to run the controller. **Be careful with real robots!**

To use the impedance controller: `roslaunch franka_example_controllers cartesian_impedance_update_controller.launch robot_ip:=172.16.0.2` and use the python interface.