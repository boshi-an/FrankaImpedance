from segment_anything import SamPredictor, sam_model_registry
from package.impedance import ImpedanceControl
import roslaunch
import yaml
import time
import sapien.core as sapien
import numpy as np
from package.realworld_env import RealworldEnv

checkpt_path = "/home/hyperplane/ros-neotic-franka/segment-anything"
calibration_path = "panda_rs_handeyecalibration_eye_on_hand.yaml"

env = RealworldEnv(calibration_path)
# sam = sam_model_registry["default"](checkpoint=checkpt_path)
# predictor = SamPredictor(sam)
# predictor.set_image(<your_image>)
# masks, _, _ = predictor.predict(<input_prompts>)

print("Robot will move, type anything to execute:")
str = input()
cam_pose = env.get_cam_pose()
env.hand_move_to(cam_pose)
time.sleep(5.0)