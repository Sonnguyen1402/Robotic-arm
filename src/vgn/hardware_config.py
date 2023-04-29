"""
Dobot CR5 & Kinect camera config
"""
T_gripper = {
      'rotation': [0.0, 0.0, 0.7071, 0.7071], 
      'translation': [-0.000, -0.000, -0.1545]
}
finger_depth = 0.05

T_robot_base = {
      'rotation': [0.0, 0.0, 0.0, 1.0], 
      'translation': [-0.25, 0.0, -0.07]
}

T_cam_pose = {
      'rotation': [0.6780, 0.6780, -0.2008, 0.2008], # [0.6799, 0.6760, -0.2073, 0.1943]
      'translation': [-0.106, -0.251, 1.025] #1.005
}
cam_frame_id = "camera_depth_optical_frame"
cam_topic_name = "/camera/depth/image_rect_raw"
cam_intrinsic = {
      'height': 480,
      'width': 640,
      'K': [580.265, 0.0, 310.39, 0.0, 580.26, 230.43, 0.0, 0.0, 1.0]
}
