#!/usr/bin/env3 python

import numpy as np
import helper_functions as hf
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    pose_list = np.load("Update with path of file")
    count = 0
    quaternion_pose = []
    for pose in pose_list:
        quaternion_pose.append(hf.ur2ros(pose))
        count += 1
    np.save("Update with desired path", quaternion_pose)