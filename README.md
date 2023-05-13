# A panda impedance control python interface

To install, you need ros and franka_ros already installed.

replace the original `franka_example_controller` directory provided by franka_ros with this `controller/franka_example_controller` directory.

rebuild the source of franka_ros.

Feel free to run the controller. **Be careful with real robots!**

To use the impedance controller directly use the python interface without starting any ros node. Be careful with the initial state of robot, the robot need to be in the blue mode to use this interface.