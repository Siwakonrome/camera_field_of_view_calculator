import time
import numpy as np
import pyrealsense2 as rs



class PyRealsense2CamInfo:

    def __init__(self):
        self.color_frame, self.aligned_depth_frame = None, None
        self.pipeline_left, self.align_left, self.depth_scale_left, self.camera_internal_info_left = self.initialize_device()
        self.extract_camera_internal_info()

        cam_info = {
                "intrinsics": self.intrinsics,
                "coeffs": self.coeffs
        }
        print(cam_info)
        print("Pipeline stopped and camera resources released.")


    def extract_camera_internal_info(self):
        camera_internal_info = self.camera_internal_info_left
        self.coeffs = np.array([camera_internal_info[4]])
        self.intrinsics = np.array([[camera_internal_info[0], 0.0,     camera_internal_info[2]],
                                    [0.0,     camera_internal_info[1], camera_internal_info[3]],
                                    [0.0,     0.0,     1.0]])


    @staticmethod
    def initialize_device():
        pipeline = rs.pipeline()
        config = rs.config()
        # Get available devices
        devices = rs.context().query_devices()
        if len(devices) == 0: raise RuntimeError("No RealSense device found")
        serial_number = devices[0].get_info(rs.camera_info.serial_number)
        config.enable_device(serial_number)
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        print(f"Device info: {pipeline_profile.get_device()}")
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        profile = pipeline.start(config)
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        print(f"Color intrinsics: {color_intrinsics}")
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        align_to = rs.stream.color
        align = rs.align(align_to)
        fx = color_intrinsics.fx
        fy = color_intrinsics.fy
        ppx = color_intrinsics.ppx
        ppy = color_intrinsics.ppy
        coeffs = color_intrinsics.coeffs
        camera_internal_info = [fx, fy, ppx, ppy, coeffs]
        return pipeline, align, depth_scale, camera_internal_info
    

    def get_frame_of_color_and_depth(self):
        frames = self.pipeline_left.wait_for_frames()
        aligned_frames = self.align_left.process(frames)
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        self.color_frame = aligned_frames.get_color_frame()
