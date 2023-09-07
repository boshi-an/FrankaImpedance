from .impedance import ImpedanceControl
from .realsense import RealsenseCamera
import roslaunch
import yaml
import time
import sapien.core as sapien
import numpy as np

class RealworldEnv :

    def __init__(self, calibration_path) :

        # node = roslaunch.core.Node("franka_example_controllers", "move_to_start.launch")

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = ['franka_example_controllers', 'move_to_start.launch', 'robot_ip:=172.16.0.2']
        roslaunch_args = launch_file[2:]  # empty but doesn't affect the outcome
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_file)[0], roslaunch_args)]
        # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        parent.start()

        try:
            parent.spin()
        finally:
            parent.shutdown()

        self.controller = ImpedanceControl()
        self.cam_to_hand = self._load_calibration(calibration_path)
        
        self.camera = RealsenseCamera()
        
    def _load_calibration(self, path) :

        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        
        t = data["transformation"]
        pose = sapien.Pose(p=[t["x"], t["y"], t["z"]],
                           q=[t["qw"], t["qx"], t["qy"], t["qz"]])
    
        return pose

    def _process_pose(self, pose) :
        
        if isinstance(pose, list) or isinstance(pose, np.ndarray):
            return sapien.Pose(p=pose[:3], q=pose[3:])
        elif isinstance(pose, sapien.Pose) :
            return pose
        else :
            raise ValueError("Invalid pose type")

    def cam_move_to(self, pose) :

        pose = self._process_pose(pose)
        
        hand_pose = pose * self.cam_to_hand.inv()
        
        self.hand_move_to(pose)
    
    def hand_move_to(self, pose) :
        
        pose = self._process_pose(pose)
        d_pose = sapien.Pose(q = [0, 0.707106781, 0, 0.707106781])
        self.controller.move_to_pose(pose * d_pose)

    def get_cam_pose(self) :
        
        hand_pose = self.get_hand_pose()
        return hand_pose * self.cam_to_hand
    
    def get_cam_rgb(self) :
        
        pass

    def get_hand_pose(self) :
        
        d_pose = sapien.Pose(q = [0, 0.707106781, 0, 0.707106781]).inv()
        return self._process_pose(self.controller.get_current_pose()) * d_pose
    
    def take_picture(self) :
        
        return self.camera.take_picture()
    
    def close_gripper(self) :

        self.controller.close_gripper()
    
    def open_gripper(self) :

        self.controller.open_gripper()