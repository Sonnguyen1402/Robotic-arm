from vgn.utils.transform import Rotation, Transform

T_tool0_tcp = Transform(Rotation.from_quat([0.000, 0.000, 0.929, 0.370]), [0.000, 0.000, 0.1345])
T_gripper = Transform(Rotation.from_quat([0.0, 0.0, 0.7071, 0.7071]), [-0.000, -0.000, -0.1345]) # + 90
finger_depth = 0.05