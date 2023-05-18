#!/usr/bin/env python

"""
Open-loop grasp execution using a Dobot CR5 and fixed position Kinect v1 camera.
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
from vgn.utils.dobot_control import Control_Dobot_TCP



class DobotGraspController(object):
    def __init__(self, args):
        self.T_tool0_tcp = Transform.from_dict(config.T_tool0_tcp)
        self.T_tcp_tool0 = self.T_tool0_tcp.inverse()
        
        T_ground_box = Transform(Rotation.identity(), [0.0, 0.0, 0.115])
        self.T_no_box = self.T_tcp_tool0 * T_ground_box
        
        self.finger_depth = config.finger_depth
        self.size = 6.0 * self.finger_depth
        self.dobot = Control_Dobot_TCP (('192.168.1.6', 6001)) # IP & Port of Dobot ('192.168.1.6', 6001)
        self.dobot.run_listener()
        
        self.tf_tree = ros_utils.TransformTree()
        self.define_workspace()
        self.tsdf_server = TSDFServer()
        self.plan_grasps = VGN(args.model, rviz=True)

        rospy.loginfo("Ready to take action")

    def define_workspace(self):
        self.task_frame_id = "task"
        # define robot to task transform
        self.T_base_task = Transform.from_dict(config.T_base_task)

        # define camera to task transform
        self.cam_frame_id = config.cam_frame_id
        self.T_cam_task = Transform.from_dict(config.T_cam_task)
        self.tf_tree.broadcast_static(
            self.T_cam_task, self.cam_frame_id, self.task_frame_id
        )
        rospy.sleep(1.0)  # wait for the TF to be broadcasted

    def run(self):
        vis.clear()
        vis.draw_workspace(self.size)
        self.dobot.move_home()
        self.dobot.open_gripper()

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

        self.execute_grasp(grasp)
        rospy.loginfo("Grasp execution")
        
    def acquire_tsdf(self):
        self.tsdf_server.reset()
        self.tsdf_server.integrate = True
        rospy.sleep(1)

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
    
    def to_pose_msg(self, pose):
        pose_list = np.r_[pose.translation, pose.rotation.as_euler('xyz', degrees=True)]
        pose_list[:3] = pose_list[:3] * 1000.0
        pose_msg = [round(elem, 4) for elem in pose_list]
        return pose_msg
    
    def execute_grasp(self, grasp):
        T_task_grasp = grasp.pose
        T_base_grasp = self.T_base_task * T_task_grasp
        
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, -0.05])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, -0.05])
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp
        T_base_retreat = T_base_grasp * T_grasp_retreat
        
        if not self.dobot.goto_pose(self.to_pose_msg(T_base_pregrasp * self.T_tcp_tool0)):
            return False

        if not self.approach_grasp(T_base_grasp):
            return False

        if not self.dobot.close_gripper():
            return False

        if not self.dobot.goto_pose(self.to_pose_msg(T_base_retreat * self.T_tcp_tool0)):
            return False
        
        # lift hand
        T_retreat_lift_base = Transform(Rotation.identity(), [0.0, 0.0, 0.16])
        T_base_lift = T_retreat_lift_base * T_base_retreat
        if not self.dobot.goto_pose(self.to_pose_msg(T_base_lift * self.T_tcp_tool0)):
            return False
        
        if self.dobot.status_gripper():
            rospy.loginfo("Object Grasped")
            self.drop(T_base_grasp)

    def approach_grasp(self, T_base_grasp):
        return self.dobot.goto_pose(self.to_pose_msg(T_base_grasp * self.T_tcp_tool0))

    def drop(self, T_base_grasp):
        z_drop = self.to_pose_msg(T_base_grasp * self.T_no_box)[2]
        if z_drop < 110.0:
            z_drop = 110.0
        self.dobot.goto_pose([550, -50, z_drop + 200.0, 179, -0.9, -90]) 
        self.dobot.goto_pose([550, -50, z_drop + 10.0, 179.0, -0.9, -90])
        self.dobot.open_gripper()
        self.dobot.goto_pose([550, -50, z_drop + 150.0, 179, -0.9, -90])

class TSDFServer(object):
    def __init__(self):
        self.cam_frame_id = config.cam_frame_id
        self.cam_topic_name = config.cam_topic_name
        self.intrinsic = CameraIntrinsic.from_dict(config.cam_intrinsic)
        self.size = 6.0 * config.finger_depth

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
    while True:
        dobot_grasp.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    main(args)