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

from vgn import vis
from vgn.experiments.clutter_removal import State
from vgn.detection import VGN
from vgn.perception import *
from vgn.utils import ros_utils
from vgn.utils.transform import Rotation, Transform
import vgn.hardware_config as config

# tag lies on the table in the center of the workspace
T_base_tag = Transform(Rotation.identity(), [0.50, 0.00, 0.14])
round_id = 0


class DobotGraspController(object):
    def __init__(self, args):
        self.robot_error = False
        self.T_gripper = config.T_gripper
        rospy.loginfo("T_gripper position: %s", self.T_gripper.translation)
        rospy.loginfo("T_gripper rotation: %s", ros_utils.to_euler(self.T_gripper.rotation))
        self.finger_depth =  config.finger_depth
        self.size = 6.0 * self.finger_depth

        self.tf_tree = ros_utils.TransformTree()
        self.define_workspace()
        self.tsdf_server = TSDFServer()
        self.plan_grasps = VGN(args.model, rviz=True)

        rospy.loginfo("Ready to take action")


    def define_workspace(self):
        z_offset = -0.06
        # t_tag_task = np.r_[[-0.5 * self.size, -0.5 * self.size, z_offset]]
        t_tag_task = np.r_[[-0.25, 0.0, z_offset]]
        T_tag_task = Transform(Rotation.identity(), t_tag_task)
        self.T_base_task = T_base_tag * T_tag_task
      
        # define camera pose
        self.task_frame_id = "task"
        self.cam_frame_id = "camera_depth_optical_frame"
        self.T_cam_task = Transform(
            Rotation.from_euler('xyz', [178.9, 14, 90], degrees=True), [0.006, -0.251, 0.77]) # [-0.679, 0.726, -0.074, -0.081] 0.166, 0.101, 0.515

        # broadcast the tf tree (for visualization)
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

        self.execute_grasp(grasp)
        rospy.loginfo("Grasp execution")

    def to_pose_msg(self, pose):
        tran_list = pose.translation.tolist()
        rot_list = ros_utils.to_euler(pose.rotation).tolist()
        pose_full = tran_list + rot_list
        rospy.loginfo("Pose raw: %s", pose_full)
        newpose = pose_full
        new_x = 1000.0 * newpose[1]
        new_y = (-1000.0 * newpose[0])
        new_z = (1000.0 * newpose[2])
        newpose[0] = new_x
        newpose[1] = new_y
        newpose[2] = new_z
        if newpose[5] < -90.0:
            newpose[5] = newpose[5] - 90.0 + 360.0
        else:
            newpose[5] = newpose[5] - 90.0
        pose_msg = [round(elem, 4) for elem in newpose]
        rospy.loginfo("Pose msg: %s", pose_msg)
        rospy.loginfo("------------------------------------")
        return pose_msg
        

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

    def execute_grasp(self, grasp):
        T_task_grasp = grasp.pose
        T_base_grasp = self.T_base_task * T_task_grasp

        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, -0.07])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, -0.07])
        T_base_pregrasp = T_base_grasp * T_grasp_pregrasp
        T_base_retreat = T_base_grasp * T_grasp_retreat
        
        rospy.loginfo("Raw position: %s", T_task_grasp.translation)       
        rospy.loginfo("Raw rotation: %s", ros_utils.to_euler(T_task_grasp.rotation))
        
        rospy.loginfo("Pregrasp position: %s", T_base_pregrasp.translation)
        self.to_pose_msg(T_base_pregrasp * self.T_gripper)

        rospy.loginfo("T_base_grasp: %s", T_base_grasp.translation)
        self.to_pose_msg(T_base_grasp * self.T_gripper)

        # lift hand
        T_retreat_lift_base = Transform(Rotation.identity(), [0.0, 0.0, 0.05])
        T_base_lift = T_retreat_lift_base * T_base_retreat
        rospy.loginfo("T_retreat_lift_grasp: %s", T_base_lift.translation)
        self.to_pose_msg(T_base_lift * self.T_gripper)

class TSDFServer(object):
    def __init__(self):
        self.cam_frame_id = "camera_depth_optical_frame"
        self.cam_topic_name = "/camera/depth/image_rect_raw"
        self.intrinsic = CameraIntrinsic(640, 480, 580.265, 580.26, 310.39, 230.43)
        # height: 480 CameraIntrinsic(640, 480, 525.26, 525.26, 319.39, 242.43) 235.43
        # width: 848
        # K: [423.189, 0.0, 423.336, 0.0, 423.18, 242.692, 0.0, 0.0, 1.0]
        self.size = 6.0 * 0.05 # 0.05

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
    
    # while True:
    #     dobot_grasp.run()
    #     rospy.sleep(2)
    # dobot_grasp.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    main(args)