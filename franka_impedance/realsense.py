import pyrealsense2 as rs
import numpy as np

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

def get_realsense_rgb_depth(pipeline) :
    
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    
    color_image = np.asanyarray(color_frame.get_data())

    z_depth = np.zeros((640, 480))

    for x in range(640) :
        for y in range(480) :
            z_depth[x,y] = depth_frame.get_distance(int(x),int(y))
    
    return color_image, z_depth

class RealsenseCamera :
    
    def __init__(self) :
        
        self.pipeline = init_realsense()
    
    def take_picture(self) :
        
        return get_realsense_rgb_depth(self.pipeline)