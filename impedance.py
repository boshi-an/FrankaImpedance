import roslaunch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import rospy
import socket
import numpy as np
import time
import actionlib
import franka_gripper.msg

class ImpedanceControl:

    def __init__ (self) :

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
    
    def move_to_pose(self, nparray) :
        pose = PoseStamped()
        pose.pose.position.x = nparray[0]
        pose.pose.position.y = nparray[1]
        pose.pose.position.z = nparray[2]
        pose.pose.orientation.w = nparray[3]
        pose.pose.orientation.x = nparray[4]
        pose.pose.orientation.y = nparray[5]
        pose.pose.orientation.z = nparray[6]
        impedance_control.eq_pose_publisher.publish(pose)
    
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

        goal = franka_gripper.msg.GraspGoal()
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
    
    tmp = input("Type in anything to start. Be careful, the robot will move. ")


    impedance_control.configure_stiffness(200.0, 20.0, 0.5)

    time.sleep(1)

    impedance_control.open_gripper()
    impedance_control.close_gripper()

    # current_pose = impedance_control.get_current_pose()
    # print(current_pose)
    # current_pose[0] -= 0.1
    # impedance_control.move_to_pose(current_pose)

    # time.sleep(5)

    # impedance_control.configure_stiffness(100.0, 10.0, 0.2)

    # time.sleep(1)

    # current_pose = impedance_control.get_current_pose()
    # current_pose[0] -= 0.1
    # impedance_control.move_to_pose(current_pose)

    # time.sleep(5)
    # print(current_pose)