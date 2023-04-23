#!/usr/bin/env python

"""
Real-time grasp detection with fixed Kinect v1 camera.
"""

import argparse
from pathlib import Path

import cv_bridge
import numpy as np
import rospy
import sensor_msgs.msg

import vgn.hardware_config as config

from vgn import vis
from vgn.experiments.clutter_removal import State
from vgn.detection import VGN
from vgn.perception import *
from vgn.utils import ros_utils
from vgn.utils.transform import Rotation, Transform


# tag lies on the table in the center of the workspace
T_base_tag = Transform(Rotation.identity(), [0.50, 0.00, 0.14])
round_id = 0


class DobotGraspController(object):
    def __init__(self, args):
        self.robot_error = False
        self.T_gripper = Transform.from_dict(config.T_gripper)
        self.finger_depth =  config.finger_depth
        self.size = 6.0 * self.finger_depth

        self.tf_tree = ros_utils.TransformTree()
        self.define_workspace()
        self.tsdf_server = TSDFServer()
        self.plan_grasps = VGN(args.model, rviz=True)

        rospy.loginfo("Ready to take action")


    def define_workspace(self):
        # define robot pose
        T_tag_task = Transform.from_dict(config.T_robot_base)
        self.T_base_task = T_base_tag * T_tag_task
        rospy.loginfo("Raw position: %s", self.T_base_task.translation)       
        rospy.loginfo("Raw rotation: %s", ros_utils.to_euler(self.T_base_task.rotation))
        # define camera pose
        self.task_frame_id = "task"
        self.cam_frame_id = config.cam_frame_id
        self.T_cam_task = Transform.from_dict(config.T_cam_pose)
        self.tf_tree.broadcast_static(
            self.T_cam_task, self.cam_frame_id, self.task_frame_id
        )
        rospy.sleep(1.0)  # wait for the TF to be broadcasted

    def run(self):
        vis.clear()
        vis.draw_workspace(self.size)

        tsdf, pc = self.acquire_tsdf()
        vis.draw_tsdf(tsdf.get_grid().squeeze(), tsdf.voxel_size)
        vis.draw_points(np.asarray(pc.points))
        rospy.loginfo("Reconstructed scene")

        state = State(tsdf, pc)
        grasps, scores, planning_time = self.plan_grasps(state)
        vis.draw_grasps(grasps, scores, self.finger_depth)
        rospy.loginfo("Planned grasps")

        if len(grasps) == 0:
            rospy.loginfo("No grasps detected")
            return

        grasp, score = self.select_grasp(grasps, scores)
        vis.draw_grasp(grasp, score, self.finger_depth)
        rospy.loginfo("Selected grasp")
        self.print_grasp(grasp)
        
    def acquire_tsdf(self):
        self.tsdf_server.reset()
        self.tsdf_server.integrate = True
        rospy.sleep(0.5)

        self.tsdf_server.integrate = False
        tsdf = self.tsdf_server.low_res_tsdf
        pc = self.tsdf_server.high_res_tsdf.get_cloud()

        return tsdf, pc

    def select_grasp(self, grasps, scores):
        # select the highest grasp
        heights = np.empty(len(grasps))
        for i, grasp in enumerate(grasps):
            heights[i] = grasp.pose.translation[2]
        idx = np.argmax(heights)
        grasp, score = grasps[idx], scores[idx]

        # make sure camera is pointing forward
        rot = grasp.pose.rotation
        axis = rot.as_matrix()[:, 0]
        if axis[0] < 0:
            grasp.pose.rotation = rot * Rotation.from_euler("z", np.pi)

        return grasp, score

    def print_grasp(self, grasp):
        T_task_grasp = grasp.pose
        rospy.loginfo("Translation: %s", T_task_grasp.translation)       
        rospy.loginfo("Rotation: %s", ros_utils.to_euler(T_task_grasp.rotation))

class TSDFServer(object):
    def __init__(self):
        self.cam_frame_id = config.cam_frame_id
        self.cam_topic_name = config.cam_topic_name
        self.intrinsic = CameraIntrinsic.from_dict(config.cam_intrinsic)
        self.size = 6.0 * 0.05

        self.cv_bridge = cv_bridge.CvBridge()
        self.tf_tree = ros_utils.TransformTree()
        self.integrate = False
        rospy.Subscriber(self.cam_topic_name, sensor_msgs.msg.Image, self.sensor_cb)

    def reset(self):
        self.low_res_tsdf = TSDFVolume(self.size, 40)
        self.high_res_tsdf = TSDFVolume(self.size, 120)

    def sensor_cb(self, msg):
        if not self.integrate:
            return
        img = self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32) * 0.001
        T_cam_task = self.tf_tree.lookup(
            self.cam_frame_id, "task", msg.header.stamp, rospy.Duration(0.1)
        )

        self.low_res_tsdf.integrate(img, self.intrinsic, T_cam_task)
        self.high_res_tsdf.integrate(img, self.intrinsic, T_cam_task)


def main(args):
    rospy.init_node("dobot_grasp")
    dobot_grasp = DobotGraspController(args)
    dobot_grasp.run()
    
    while True:
        dobot_grasp.run()
        rospy.sleep(2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    main(args)