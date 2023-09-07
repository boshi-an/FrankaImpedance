import roslaunch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import rospy
import socket
import numpy as np
import time
import actionlib
import franka_gripper.msg
import sapien.core as sapien

class ImpedanceControl:

    def __init__ (self) :
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = ['franka_example_controllers', 'cartesian_impedance_update_controller.launch', 'robot_ip:=172.16.0.2']
        roslaunch_args = launch_file[2:]  # empty but doesn't affect the outcome
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_file)[0], roslaunch_args)]
        # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        parent.start()
        
        # try:
        #     parent.spin()
        # finally:
        #     parent.shutdown()

        self.default_translational_stiffness = 200.0
        self.default_rotational_stiffness = 10.0
        self.default_null_space_stiffness = 0.5
        self.current_translational_stiffness = self.default_translational_stiffness
        self.current_rotational_stiffness = self.default_rotational_stiffness
        self.current_null_space_stiffness = self.default_null_space_stiffness

        rospy.init_node("impedance_control_python_interface")
        self.eq_pose_publisher = rospy.Publisher("/cartesian_impedance_update_controller/equilibrium_pose", PoseStamped, queue_size=1000)
        self.param_publisher = rospy.Publisher("/cartesian_impedance_update_controller/compliance_param", Float64MultiArray, queue_size=1000)

        self.subscriber = rospy.Subscriber("/cartesian_impedance_update_controller/ee_pose", PoseStamped, self.callback)
        
        self.gripper_client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.gripper_client.wait_for_server()
        

        self.current_pose = PoseStamped()

        # while not rospy.is_shutdown() :
        #     message = PoseStamped()
        #     message.pose.position.x = 0.5
        #     message.pose.position.y = 0.5
        #     message.pose.position.z = 0.5
        #     message.pose.orientation.x = 0.0
        #     message.pose.orientation.y = 0.0
        #     message.pose.orientation.z = 0.0
        #     message.pose.orientation.w = 1.0
        #     self.publisher.publish(message)
        #     self.rate.sleep()
    
    def _process_pose(self, pose) :
        
        if isinstance(pose, list) or isinstance(pose, np.ndarray):
            return np.asarray(pose)
        elif isinstance(pose, sapien.Pose) :
            return np.concatenate([pose.p, pose.q])
        else :
            raise ValueError("Invalid pose type")
    
    def configure_stiffness(self, translational, rotational, nullspace) :

        if translational != None :
            self.current_translational_stiffness = translational
        if rotational != None :
            self.current_rotational_stiffness = rotational
        if nullspace != None :
            self.current_null_space_stiffness = nullspace
        
        data = [0, 0, 0]
        data[0] = self.current_translational_stiffness
        data[1] = self.current_rotational_stiffness
        data[2] = self.current_null_space_stiffness
        msg = Float64MultiArray(data=data)

        self.param_publisher.publish(msg)
    
    def callback(self, data) :
        self.current_pose = data
    
    def get_current_pose(self) :
        pose = self.current_pose
        nparray = np.zeros((7,))
        nparray[0] = self.current_pose.pose.position.x
        nparray[1] = self.current_pose.pose.position.y
        nparray[2] = self.current_pose.pose.position.z
        nparray[3] = self.current_pose.pose.orientation.w
        nparray[4] = self.current_pose.pose.orientation.x
        nparray[5] = self.current_pose.pose.orientation.y
        nparray[6] = self.current_pose.pose.orientation.z
        return nparray
    
    def move_to_pose(self, pose) :
        nparray = self._process_pose(pose)
        pose = PoseStamped()
        pose.pose.position.x = nparray[0]
        pose.pose.position.y = nparray[1]
        pose.pose.position.z = nparray[2]
        pose.pose.orientation.w = nparray[3]
        pose.pose.orientation.x = nparray[4]
        pose.pose.orientation.y = nparray[5]
        pose.pose.orientation.z = nparray[6]
        self.eq_pose_publisher.publish(pose)
    
    def close_gripper(self) :

        goal = franka_gripper.msg.GraspGoal()
        goal.width = 0.022
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.1
        goal.force = 5

        # Sends the goal to the action server.
        self.gripper_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.gripper_client.wait_for_result()

        # Prints out the result of executing the action
        return self.gripper_client.get_result()  # A GraspResult
    
    def open_gripper(self) :

        goal = franka_gripper.msg.MoveGoal()
        goal.width = 0.04
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.1
        goal.force = 5

        # Sends the goal to the action server.
        self.gripper_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.gripper_client.wait_for_result()

        # Prints out the result of executing the action
        return self.gripper_client.get_result()  # A GraspResult

if __name__ == "__main__" :

    impedance_control = ImpedanceControl()

    time.sleep(2)
    
    tmp = input("Type in anything to start. Be careful, the robot will move. ")

    impedance_control.configure_stiffness(130.0, 20.0, 0.2)

    time.sleep(2)

    # # impedance_control.open_gripper()
    # impedance_control.close_gripper()

    current_pose1 = impedance_control.get_current_pose()
    # print("init_pos", current_pose1)
    current_pose1[0] += 0.1
    # current_pose1[:7] = [0.30101646, -0.00,  0.49053053,   0,   0,   1.,   0. ]
    print("target_pos", current_pose1)
    # [0.33, 0, 0.4], [0.788205, 0, 0.615412, 0]

    print("move to next pose")
    impedance_control.move_to_pose(current_pose1)
    for i in range(20) :

        time.sleep(1)
        current_pose2 = impedance_control.get_current_pose()
        print("now_pos", current_pose2)


    # impedance_control.configure_stiffness(100.0, 10.0, 0.2)

    # time.sleep(1)

    # current_pose = impedance_control.get_current_pose()
    # current_pose[0] -= 0.1
    # impedance_control.move_to_pose(current_pose)

    # time.sleep(5)
    # print(current_pose)