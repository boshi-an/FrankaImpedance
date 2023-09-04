import roslaunch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import rospy
import socket
import numpy as np
import time
import actionlib
import franka_gripper.msg
from franka_impedance import impedance

def get_distance(pose1, pose2):
    return (pose1[0]- pose2[0]) ** 2 + (pose1[1]- pose2[1]) ** 2 + (pose1[2]- pose2[2]) ** 2

def rotate_vector_3d(vector, axis, angle):
    # 将向量和轴转换为NumPy数组
    vector = np.array(vector)
    axis = np.array(axis)

    # 将轴向量归一化为单位向量
    axis = axis / np.linalg.norm(axis)

    # 创建旋转矩阵
    rotation_matrix = np.array([[np.cos(angle) + axis[0]**2 * (1 - np.cos(angle)),
                                 axis[0] * axis[1] * (1 - np.cos(angle)) - axis[2] * np.sin(angle),
                                 axis[0] * axis[2] * (1 - np.cos(angle)) + axis[1] * np.sin(angle)],
                                [axis[1] * axis[0] * (1 - np.cos(angle)) + axis[2] * np.sin(angle),
                                 np.cos(angle) + axis[1]**2 * (1 - np.cos(angle)),
                                 axis[1] * axis[2] * (1 - np.cos(angle)) - axis[0] * np.sin(angle)],
                                [axis[2] * axis[0] * (1 - np.cos(angle)) - axis[1] * np.sin(angle),
                                 axis[2] * axis[1] * (1 - np.cos(angle)) + axis[0] * np.sin(angle),
                                 np.cos(angle) + axis[2]**2 * (1 - np.cos(angle))]])

    # 应用旋转矩阵到向量上
    rotated_vector = np.dot(rotation_matrix, vector)

    return rotated_vector

def rotate_quaternion(quaternion, axis, angle):
    # 将四元数和轴转换为NumPy数组
    quaternion = np.array(quaternion)
    axis = np.array(axis)

    # 将轴向量归一化为单位向量
    axis = axis / np.linalg.norm(axis)

    # 计算旋转的一半角度
    half_angle = angle / 2

    # 创建表示旋转的四元数
    rotation_quaternion = np.array([np.cos(half_angle),
                                    axis[0] * np.sin(half_angle),
                                    axis[1] * np.sin(half_angle),
                                    axis[2] * np.sin(half_angle)])

    # 进行四元数乘法，得到旋转后的四元数
    rotated_quaternion = quaternion_multiply(rotation_quaternion, quaternion)

    return rotated_quaternion

# 四元数乘法函数
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


impedance_controler = impedance.ImpedanceControl()
tmp = input("Type in anything to start. Be careful, the robot will move. ")
impedance_controler.configure_stiffness(130.0, 20.0, 0.2) #the second parameter stands for rotational stiffness (should be large)

time.sleep(3)

# impedance_controler.open_gripper()
impedance_controler.close_gripper()
step = 0.08
iterations = 10
delta_pose = np.array([step, 0, 0, 0, 0, 0, 0])

time.sleep(1)

initial_pose = impedance_controler.get_current_pose()
print(initial_pose)

for i in range(iterations): 
    expected_pose = initial_pose - delta_pose
    impedance_controler.move_to_pose(expected_pose)
    time.sleep(1.3)
    actual_pose = impedance_controler.get_current_pose()
    delta_vec = actual_pose[:3] - initial_pose[:3]
    delta_s2 = get_distance(actual_pose, initial_pose)
    delta_l = np.dot(delta_vec, delta_pose[:3]) / np.linalg.norm(delta_pose[:3])
    delta_x = np.sqrt(delta_s2 - delta_l ** 2)
    delta_theta = np.arcsin(2 * delta_x * delta_l / delta_s2)
    axis = np.cross(delta_vec, delta_pose[:3])
    print("delta_theta: ", delta_theta)
    print("delta_s: ", np.sqrt(delta_s2))
    print("delta_x: ", delta_x)
    print("axis: ", axis)

    final_pose = np.concatenate((actual_pose[:3], rotate_quaternion(actual_pose[3:], axis, -delta_theta * 2)))
    
    print(i," : ",final_pose)
    impedance_controler.move_to_pose(final_pose)
    time.sleep(0.5)

    initial_pose = final_pose
    delta_pose = rotate_vector_3d(delta_pose[:3], axis, -delta_theta)
    delta_pose = np.concatenate((delta_pose, np.array([0, 0, 0, 0])))
