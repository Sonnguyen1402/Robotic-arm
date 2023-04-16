#!/usr/bin/env python

"""
Real-time grasp detection.
"""

import argparse
from pathlib import Path
import time

import cv_bridge
import numpy as np
import rospy
import sensor_msgs.msg
import torch

from tcp_client import Control_Dobot_TCP 

from vgn import vis
from vgn.detection import *
from vgn.perception import *
from vgn.utils import ros_utils
from vgn.utils.transform import Rotation, Transform

T_base_tag2 = Transform(Rotation.identity(), [0.42, 0.02, 0.21])

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 

class GraspDetectionServer(object):
    def __init__(self, model_path):
        # create connection with robot
        # self.dobot = Control_Dobot_TCP(('127.0.0.1', 8001))
        # self.dobot.create_listen()
        
        
        # define frames
        self.task_frame_id = "task"
        self.cam_frame_id = "camera_depth_optical_frame"
        self.T_cam_task = Transform(
            Rotation.from_euler('xyz', [178.7, 12, 90], degrees=True), [-0.166, -0.151, 0.712]) # [-0.679, 0.726, -0.074, -0.081] 0.166, 0.101, 0.515
        
        
        # Rotation.from_quat([0.679, 0.726, -0.174, -0.021]), [0.166, 0.101, 0.74])
        # broadcast the tf tree (for visualization)
        self.tf_tree = ros_utils.TransformTree()
        self.tf_tree.broadcast_static(
            self.T_cam_task, self.cam_frame_id, self.task_frame_id
        )
        # self.define_workspace()

        # define camera parameters
        self.cam_topic_name = "/camera/depth/image_rect_raw"
        self.intrinsic = CameraIntrinsic(640, 480, 579.26, 579.26, 309.39, 242.43) # 640, 480, 383.265, 383.26, 319.39, 242.43
        # self.intrinsic = CameraIntrinsic(640, 480, 0, 0, 0, 0)
        # setup a CV bridge
        self.cv_bridge = cv_bridge.CvBridge()

        # construct the grasp planner object
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.net = load_network(model_path, self.device)

        # initialize the visualization
        vis.clear()
        vis.draw_workspace(0.3)

        # subscribe to the camera
        self.img = None
        rospy.Subscriber(self.cam_topic_name, sensor_msgs.msg.Image, self.sensor_cb)

        # setup cb to detect grasps
        rospy.Timer(rospy.Duration(0.1), self.detect_grasps)
        print("End Init")

    def sensor_cb(self, msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32) * 0.001
        
    def define_workspace(self):
        z_offset = -0.06
        t_tag_task = np.r_[[-0.5 * 0.3, -0.5 * 0.3, z_offset]]
        T_tag_task = Transform(Rotation.identity(), t_tag_task)
        self.T_base_task = T_base_tag2 * T_tag_task

        # self.tf_tree.broadcast_static(self.T_base_task, self.base_frame_id, "task")
        self.tf_tree.broadcast_static(self.T_base_task, "camera_depth_optical_frame", "task")
        # rospy.sleep(1.0)  # wait for the TF to be broadcasted
        
    def detect_grasps(self, _):
        if self.img is None:
            return

        tic = time.time()
        self.tsdf = TSDFVolume(0.3, 40)
        self.tsdf.integrate(self.img, self.intrinsic, self.T_cam_task)
        #print("Construct tsdf ", time.time() - tic)

        tic = time.time()
        tsdf_vol = self.tsdf.get_grid()
        voxel_size = self.tsdf.voxel_size
        #print("Extract grid  ", time.time() - tic)
        vis.draw_tsdf(tsdf_vol, voxel_size)
        tic = time.time()
        qual_vol, rot_vol, width_vol = predict(tsdf_vol, self.net, self.device)
        #print("Forward pass   ", time.time() - tic)

        tic = time.time()
        qual_vol, rot_vol, width_vol = process(tsdf_vol, qual_vol, rot_vol, width_vol)
        #print("Filter         ", time.time() - tic)

        vis.draw_quality(qual_vol, voxel_size, threshold=0.01)

        tic = time.time()
        grasps, scores = select(qual_vol, rot_vol, width_vol, 0.90, 1)
        
        num_grasps = len(grasps)
        if num_grasps > 0:
            idx = np.random.choice(num_grasps, size=min(1, num_grasps), replace=False)
            grasps, scores = np.array(grasps)[idx], np.array(scores)[idx]
            thegrasp = ros_utils.to_pose_msg(grasps[0].pose)
            newlist = grasps[0].pose.translation.tolist()
            newlist2 = ros_utils.to_euler(grasps[0].pose.rotation).tolist()
            newpose1 = newlist + newlist2
            newpose = [round(elem, 4) for elem in newpose1]
            print("Pose: ", newpose)
            #print("Position: ", grasps[0].pose.translation.tolist())
            # print("Rotation: ", ros_utils.to_euler(grasps[0].pose.rotation).tolist())
        #rint("Ori; ", thegrasp.orientation)
        #print("Rotation:" ,ros_utils.from_quat_msg(thegrasp))
        grasps = [from_voxel_coordinates(g, voxel_size) for g in grasps]
        # print("Select grasps  ", time.time() - tic)

        vis.clear_grasps()
        rospy.sleep(0.1)
        tic = time.time()
        vis.draw_grasps(grasps, scores, 0.05)
        # print("Visualize      ", time.time() - tic)

        self.img = None
        #print()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    rospy.init_node("panda_detection")
    GraspDetectionServer(args.model)
    rospy.spin()
