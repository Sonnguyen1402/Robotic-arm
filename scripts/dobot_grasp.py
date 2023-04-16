#!/usr/bin/env python

"""
Open-loop grasp execution using a Panda arm and wrist-mounted RealSense camera.
"""

import argparse
from pathlib import Path

import cv_bridge
#import franka_msgs.msg
import geometry_msgs.msg
import numpy as np
import rospy
import sensor_msgs.msg

from vgn import vis
from vgn.experiments.clutter_removal import State
from vgn.detection import VGN
from vgn.perception import *
from vgn.utils import ros_utils
from vgn.utils.transform import Rotation, Transform
# from vgn.utils.panda_control import PandaCommander
import init_conf
from tcp_client import Control_Dobot_TCP

import sys
# tag lies on the table in the center of the workspace
T_base_tag = Transform(Rotation.identity(), [0.50, 0.00, 0.0])
round_id = 0


class PandaGraspController(object):
    def __init__(self, args):
        self.robot_error = False

        #self.base_frame_id = rospy.get_param("~base_frame_id") # panda_link0
        #self.tool0_frame_id = rospy.get_param("~tool0_frame_id") # panda_link8
        #self.T_tool0_tcp = Transform.from_dict(rospy.get_param("~T_tool0_tcp"))  # {rotation: [0.000, 0.000, -0.382, 0.924], translation: [0.000, 0.000, 0.065]}
        #self.T_tcp_tool0 = self.T_tool0_tcp.inverse() # 
        #self.finger_depth = rospy.get_param("~finger_depth") # 0.05
        #self.size = 6.0 * self.finger_depth # 0.3
        #self.scan_joints = rospy.get_param("~scan_joints") # 4 goc quet 
        self.T_tool0_tcp = Transform(Rotation.from_quat([0.000, 0.000, -0.382, 0.924]), [0.000, 0.000, 0.1345])
        self.T_tcp_tool0 = self.T_tool0_tcp.inverse()
        print("T_tcp_tool0: ", self.T_tool0_tcp.translation)
        self.finger_depth =  0.05
        self.size = 6.0 * self.finger_depth
        # self.dobot = Control_Dobot_TCP (('127.0.0.1', init_conf.PORT))
        # self.dobot.create_listen()

        # self.setup_panda_control()
        self.tf_tree = ros_utils.TransformTree()
        self.define_workspace()
        # self.create_planning_scene()
        self.tsdf_server = TSDFServer()
        self.plan_grasps = VGN(args.model, rviz=True)

        rospy.loginfo("Ready to take action")

    # def setup_panda_control(self):
    #     rospy.Subscriber(
    #         "/franka_state_controller/franka_states",
    #         franka_msgs.msg.FrankaState,
    #         self.robot_state_cb,
    #         queue_size=1,
    #     )
    #     rospy.Subscriber(
    #         "/joint_states", sensor_msgs.msg.JointState, self.joints_cb, queue_size=1
    #     )
    #     self.pc = PandaCommander()
    #     self.pc.move_group.set_end_effector_link(self.tool0_frame_id)

    def define_workspace(self):
        z_offset = -0.06
        # t_tag_task = np.r_[[-0.5 * self.size, -0.5 * self.size, z_offset]]
        t_tag_task = np.r_[[-0.25, 0.0, -0.06]]
        T_tag_task = Transform(Rotation.identity(), t_tag_task)
        self.T_base_task = T_base_tag * T_tag_task

        print("T_tag_task: ", T_tag_task.translation)
        print("T_base_task: ", self.T_base_task.translation)
        # self.tf_tree.broadcast_static(self.T_base_task, self.base_frame_id, "task")
        # self.tf_tree.broadcast_static(self.T_base_task, "camera_depth_optical_frame", "task")
                # define frames
        self.task_frame_id = "task"
        self.cam_frame_id = "camera_depth_optical_frame"
        self.T_cam_task = Transform(
            Rotation.from_euler('xyz', [178.7, 12, 90], degrees=True), [0.006, -0.251, 0.712]) # [-0.679, 0.726, -0.074, -0.081] 0.166, 0.101, 0.515

        # broadcast the tf tree (for visualization)
        # self.tf_tree = ros_utils.TransformTree()
        self.tf_tree.broadcast_static(
            self.T_cam_task, self.cam_frame_id, self.task_frame_id
        )
        rospy.sleep(1.0)  # wait for the TF to be broadcasted

    # def create_planning_scene(self):
    #     # collision box for table
    #     msg = geometry_msgs.msg.PoseStamped()
    #     msg.header.frame_id = self.base_frame_id
    #     msg.pose = ros_utils.to_pose_msg(T_base_tag)
    #     msg.pose.position.z -= 0.01
    #     self.pc.scene.add_box("table", msg, size=(0.6, 0.6, 0.02))

    #     rospy.sleep(1.0)  # wait for the scene to be updated

    # def robot_state_cb(self, msg):
    #     detected_error = False
    #     if np.any(msg.cartesian_collision):
    #         detected_error = True
    #     # for s in franka_msgs.msg.Errors.__slots__:
    #     #     if getattr(msg.current_errors, s):
    #     #         detected_error = True
    #     if not self.robot_error and detected_error:
    #         self.robot_error = True
    #         rospy.logwarn("Detected robot error")

    # def joints_cb(self, msg):
    #     self.gripper_width = msg.position[7] + msg.position[8]

    # def recover_robot(self):
    #     self.pc.recover()
    #     self.robot_error = False
    #     rospy.loginfo("Recovered from robot error")

    def run(self):
        vis.clear()
        vis.draw_workspace(self.size)
        # self.pc.move_gripper(0.08)
        # self.pc.home()
        # self.dobot.open_gripper()
        # self.dobot.move_home()

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

        # self.dobot.move_home()
        label = self.execute_grasp(grasp)
        rospy.loginfo("Grasp execution")

        # if self.robot_error:
        #     self.recover_robot()
        #     return

        # if label:
        #     self.drop()
        #     rospy.loginfo("Object Grasped")
        # self.dobot.move_home()
        
        # if self.dobot.exit_flag:
        #     sys.exit(0)

    def modify_pose(self, pose):
        tran_list = pose.translation.tolist()
        rot_list = ros_utils.to_euler(pose.rotation).tolist()
        pose_full = tran_list + rot_list
        # pose_4 = [round(elem, 4) for elem in pose_full]
        print("------------------------------------")
        print("Pose1: ", pose_full)
        # newpose2 = self.modify_pose(newpose)
        # print("Pose2: ", newpose2)
        newpose = pose_full
        new_x = 1000.0 * newpose[1]
        new_y = (-1000.0 * newpose[0]) # -290 + - 268.0 
        new_z = (1000.0 * newpose[2])  # 80 + + 190.0
        newpose[0] = new_x
        newpose[1] = new_y
        newpose[2] = new_z
        pose_fin = [round(elem, 4) for elem in newpose]
        print("------------------------------------")
        print("Pose2: ", pose_fin)
        print("------------------------------------")
        return pose_fin
        

    def acquire_tsdf(self):
        # self.pc.goto_joints(self.scan_joints[0])
        

        self.tsdf_server.reset()
        self.tsdf_server.integrate = True
        rospy.sleep(1)
        # self.dobot.open_gripper()
        # for joint_target in self.scan_joints[1:]:
        #     self.pc.goto_joints(joint_target)

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
        
        print("T_task_grasp: ", T_task_grasp.translation )       
        print("T_base_grasp: ", self.T_tcp_tool0.translation)
        print("T_base_pregrasp: ", T_base_pregrasp.translation)
        #print("T_base_retreat : ", T_base_retreat.translation)
        self.modify_pose(T_base_pregrasp * self.T_tcp_tool0)
        self.modify_pose(T_base_grasp * self.T_tcp_tool0)
        # comfirm_move = self.dobot.goto_pose(self.modify_pose(T_base_pregrasp * self.T_tcp_tool0))
        # comfirm_move = self.approach_grasp(T_base_grasp)
        # comfirm_move = self.dobot.goto_pose(self.modify_pose(T_base_pregrasp))
        # comfirm_move = self.dobot.goto_pose(self.modify_pose(T_task_grasp))
        # if not comfirm_move:
        #     return False
        
        # # if self.robot_error:
        # #     return False

        # # self.pc.grasp(width=0.0, force=20.0)
        # comfirm_grasp = self.dobot.grasp()
        
        # comfirm_move = self.dobot.goto_pose(self.modify_pose(T_base_retreat))
        # if not comfirm_grasp:
        #     return False
        # else:
        #     return True

        # self.dobot.goto_pose(self.modify_pose(T_base_retreat * self.T_tcp_tool0))

        # lift hand
        T_retreat_lift_base = Transform(Rotation.identity(), [0.0, 0.0, 0.05])
        T_base_lift = T_retreat_lift_base * T_base_retreat
        self.modify_pose(T_base_lift * self.T_tcp_tool0)
        # self.dobot.goto_pose(self.modify_pose(T_base_lift * self.T_tcp_tool0))
        
        # if not comfirm_grasp:
        #     return False
        # else:
        #     return True
        
        # if self.gripper_width > 0.004:
        #     return True
        # else:
        #     return False

    # def approach_grasp(self, T_base_grasp):
    #     return self.dobot.goto_pose(self.modify_pose(T_base_grasp * self.T_tcp_tool0))

    # def drop(self):
    #     # self.pc.goto_joints(
    #     #     [0.678, 0.097, 0.237, -1.63, -0.031, 1.756, 0.931], 0.2, 0.2
    #     # )
    #     # self.pc.move_gripper(0.08)
    #     self.dobot.goto_pose([550, -50, 450, 179, -0.9, 179])
    #     self.dobot.goto_pose([550, -50, 300, 179, -0.9, 179])
    #     self.dobot.open_gripper()


class TSDFServer(object):
    def __init__(self):
        # self.cam_frame_id = rospy.get_param("~cam/frame_id") # camera_depth_optical_frame
        # self.cam_topic_name = rospy.get_param("~cam/topic_name") # /camera/depth/image_rect_raw
        # self.intrinsic = CameraIntrinsic.from_dict(rospy.get_param("~cam/intrinsic")) 
        # # height: 480
        # # width: 848
        # # K: [423.189, 0.0, 423.336, 0.0, 423.18, 242.692, 0.0, 0.0, 1.0]
        # self.size = 6.0 * rospy.get_param("~finger_depth") # 0.05
        
        self.cam_frame_id = "camera_depth_optical_frame"
        self.cam_topic_name = "/camera/depth/image_rect_raw" # /camera/depth/image_rect_raw
        self.intrinsic = CameraIntrinsic(640, 480, 560.265, 560.26, 310.39, 230.43)
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
        # print("Sensor_cb")
        if not self.integrate:
            return
        
        img = self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32) * 0.001
        T_cam_task = self.tf_tree.lookup(
            self.cam_frame_id, "task", msg.header.stamp, rospy.Duration(0.1)
        )

        self.low_res_tsdf.integrate(img, self.intrinsic, T_cam_task)
        self.high_res_tsdf.integrate(img, self.intrinsic, T_cam_task)


def main(args):
    rospy.init_node("panda_grasp")
    panda_grasp = PandaGraspController(args)

    while True:
        panda_grasp.run()
        rospy.sleep(2)
    # panda_grasp.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    main(args)
