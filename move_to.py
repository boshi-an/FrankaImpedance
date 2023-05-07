import rospy
from panda_robot import PandaArm
import yaml
import numpy as np
import quaternion
import pyrealsense2 as rs
import cv2
import os

def load_calibration(path) :
    
    config = None
        
    with open("panda_rs_handeyecalibration_eye_on_hand.yaml", "r") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
        pass
    trans = config["transformation"]
    quat = quaternion.quaternion(trans["qx"], trans["qy"], trans["qz"], trans["qw"])
    pos = [trans["x"], trans["y"], trans["z"]]
    
    return pos, quat

def pose_multiplication(pos1, ori1, pos2, ori2) :
    
    pos2_rot = quaternion.rotate_vectors(ori1, pos2)
    
    return pos1+pos2_rot, ori1 * ori2

def process_pose(pos, ori) :

    return pos, np.asarray([ori.x, ori.y, ori.z, ori.w])

def move_to_camera_frame(arm, cal_pos, cal_ori, tar_pos) :
    
    arm.set_joint_position_speed(0.05)
    arm.move_to_neutral()

    arm.get_gripper().open()
    arm.get_gripper().close()
    
    cur_pos, cur_ori = arm.ee_pose()
    
    tar_to_base_pos = cur_pos + cal_pos + tar_pos
    tar_to_base_ori = cur_ori * cal_ori
    
    panda_arm.move_to_cartesian_pose(tar_to_base_pos, tar_to_base_ori)

def init_realsense() :
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)
        
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
    pipeline.start(config)
    return pipeline

def get_realsense_rgb(pipeline) :
    
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    
    color_image = np.asanyarray(color_frame.get_data())
    
    return color_image

if __name__ == "__main__" :
    
    root = "./real_data"
    handle_num = 3
    image_num = 4
    
    rs_pipeline = init_realsense()
    
    cal_pos, cal_quat = load_calibration("./panda_rs_handeyecalibration_eye_on_hand.yaml")
    rospy.init_node("panda_demo") # initialise ros node
    panda_arm = PandaArm() # create PandaArm instance
    
    # while True :
    #     rgb = get_realsense_rgb(rs_pipeline)
    #     cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    #     cv2.imshow('RealSense', rgb)
    #     cv2.waitKey(1)
    
    ee_pos, ee_ori = panda_arm.ee_pose()

    cam_pos, cam_ori = pose_multiplication(ee_pos, ee_ori, cal_pos, cal_quat)

    # panda_arm.move_to_cartesian_pose(cam_pos, cam_ori, use_moveit=False)
    panda_arm.get_gripper().open()
    panda_arm.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01])
    
    # move(panda_arm)

# r.get_gripper().home_joints()
# pos,ori = r.ee_pose()
# r.get_gripper().home_joints()
# r.get_gripper().open()
# r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01])
# r.move_to_cartesian_pose(pos,ori)
# r.get_gripper().grasp(0.02, 1000, epsilon_inner=0.1, epsilon_outer=0.1)