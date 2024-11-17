import pinocchio as pin
import rospkg
import os
import math
import numpy as np
import rospy
import time
from pinocchio.robot_wrapper import RobotWrapper

rospack = rospkg.RosPack()

package_name = "g1_description"

file_name = "robot.urdf"

urdf_path = os.path.join(rospack.get_path(package_name), "urdf", file_name)


robot = RobotWrapper.BuildFromURDF(urdf_path, [""], pin.JointModelFreeFlyer())

model = robot.model

