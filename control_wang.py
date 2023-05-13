import roslaunch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import rospy
import socket
import numpy as np
import time
import actionlib
import franka_gripper.msg
from impedance import ImpedanceControl

def intoplate_move(impedance_control,cur, des, num):
    cur = np.array(cur)
    des = np.array(des)
    delta = (des - cur ) / num
    for i in range(num):
        cur = delta + cur
        impedance_control.move_to_pose(cur.tolist())
        time.sleep(1)
    return


if __name__ == "__main__" :

    impedance_control = ImpedanceControl()
    # [ 3.06013153e-01 -3.36511240e-04  4.85137611e-01  4.98359979e-04
    # 9.99999578e-01 -9.04802283e-05 -7.66712466e-04]

#     [ 0.52041179 -0.04594893  0.13783044 -0.00406643  0.99949611 -0.00379083
#   0.03125104]
    home_pose = [ 3.06013153e-01, -3.36511240e-04,  4.85137611e-01,  4.98359979e-04,
    9.99999578e-01, -9.04802283e-05, -7.66712466e-04]
    des_pose  = [ 0.52041179, -0.04594893,  0.13783044, -0.00406643,  0.99949611, -0.00379083,
  0.03125104]

    impedance_control.configure_stiffness(500.0, 40.0, 10)

    # impedance_control.open_gripper()
    

    current_pose = impedance_control.get_current_pose()
    print(current_pose)
    impedance_control.move_to_pose(home_pose)
    time.sleep(2)
    # impedance_control.move_to_pose(des_pose)
    intoplate_move(impedance_control,home_pose, des_pose, 10)  
    time.sleep(5)
    impedance_control.close_gripper()
    impedance_control.open_gripper()
    impedance_control.move_to_pose(home_pose)
    print(current_pose)

