import time
import os
import rospkg
import numpy as np
import rospy
import pinocchio
from pinocchio import difference
from pinocchio.robot_wrapper import RobotWrapper
from std_msgs.msg import Float32MultiArray,Bool,String
from geometry_msgs.msg import Pose,Twist
from nao_controller.msg import MPCxuk

rospack = rospkg.RosPack()

class controller_node:
    def __init__(self, package_name, file_name):
        self.urdf_path = os.path.join(rospack.get_path(package_name), "urdf", file_name)
        self.robot = RobotWrapper.BuildFromURDF(self.urdf_path, [""], pinocchio.JointModelFreeFlyer())
        self.model = self.robot.model
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nu = 23
        self.joint_state = np.zeros(2 * self.nu)
        self.body_pose = np.zeros(7)
        self.body_twist = np.zeros(6)
        self.x0 = np.zeros(self.nq + self.nv)
        self.torque = np.zeros(self.nu)
        self.torquetime = time.time()
        self.lqrTime = time.time()
        self.body_pose_sub = rospy.Subscriber('/bodyPose', Pose, self.body_pose_callback)
        self.body_twist_sub = rospy.Subscriber('/bodyTwist', Twist, self.body_twist_callback)
        self.joint_state_sub = rospy.Subscriber('/jointsPosVel', Float32MultiArray, self.joint_state_callback)
        self.mpc_xuk_sub = rospy.Subscriber('/mpc_xuk', MPCxuk, self.mpcxuk_callback)
        self.lqr_xuk_sub = rospy.Subscriber('/lqr_xuk', MPCxuk, self.lqrxuk_callback)
        self.torque_pub = rospy.Publisher('/jointsTorque', Float32MultiArray, queue_size=10)
        self.ctrl_mode_sub = rospy.Subscriber('/ctrlMode', String, self.ctrl_mode_callback)

        self.lqr_xs = [np.zeros(self.nq+self.nv)] * 2
        self.lqr_us = [np.zeros(self.nu)] * 2
        self.lqr_Ks = [np.zeros((self.nu, self.nv*2))] * 2

        self.mpc_xs = [np.zeros(self.nq+self.nv)] * 2
        self.mpc_us = [np.zeros(self.nu)] * 2
        self.mpc_Ks = [np.zeros((self.nu, self.nv*2))] * 2

        self.ctrl_mode = '0'
        self.lqr = False

    def ctrl_mode_callback(self, msg):
        self.ctrl_mode = msg.data

    def mpcxuk_callback(self, msg):
        # self.mpcTime = msg.stamp.to_sec()
        # print("mpctime",self.mpcTime)
        # 重构二维数组 us 和 xs
        self.mpc_us = np.array(msg.us).reshape((msg.ncontrol, self.nu))
        self.mpc_xs = np.array(msg.xs).reshape((msg.ncontrol, self.nq + self.nv))
        # 重构三维数组 Ks
        self.mpc_Ks = np.array(msg.Ks).reshape((msg.ncontrol, self.nu, 2 * self.nv))

    def lqrxuk_callback(self, msg):
        # self.mpcTime = msg.stamp.to_sec()
        # print("mpctime",self.mpcTime)
        # 重构二维数组 us 和 xs
        self.lqr_us = np.array(msg.us).reshape((msg.ncontrol, self.nu))
        self.lqr_xs = np.array(msg.xs).reshape((msg.ncontrol, self.nq + self.nv))
        # 重构三维数组 Ks
        self.lqr_Ks = np.array(msg.Ks).reshape((msg.ncontrol, self.nu, 2 * self.nv))
        self.lqrTime = time.time()
        self.lqr = True


    def joint_state_callback(self, msg):
        self.joint_state = np.array(msg.data)

    def body_pose_callback(self, msg):
        # self.state_rsv = True
        self.body_pose[0] = msg.position.x
        self.body_pose[1] = msg.position.y
        self.body_pose[2] = msg.position.z
        self.body_pose[3] = msg.orientation.x
        self.body_pose[4] = msg.orientation.y
        self.body_pose[5] = msg.orientation.z
        self.body_pose[6] = msg.orientation.w
        # rospy.loginfo("Received robot state")

    def body_twist_callback(self, msg):
        self.body_twist[0] = msg.linear.x
        self.body_twist[1] = msg.linear.y
        self.body_twist[2] = msg.linear.z
        self.body_twist[3] = msg.angular.x
        self.body_twist[4] = msg.angular.y
        self.body_twist[5] = msg.angular.z

    def updateX0(self):
        self.x0[0:7] = self.body_pose
        self.x0[7:self.nq] = self.joint_state[0:self.nu]
        self.x0[self.nq:self.nq+6] = self.body_twist
        self.x0[self.nq+6:self.nq+self.nv] = self.joint_state[self.nu:2 * self.nu]

    def lqr_torque(self):
        self.updateX0()
        dxoutTemp = np.zeros(self.nv + self.nv)
        index = int((time.time() - self.lqrTime) * 100)
        print(index)
        if index >= len(self.lqr_xs)-1:
            self.lqr = False
        dxoutTemp[:self.nv] = difference(self.model, self.lqr_xs[index][:self.nq], self.x0[:self.nq])
        dxoutTemp[self.nv:] = self.x0[self.nq:] - self.lqr_xs[index][self.nq:]
        print(np.linalg.norm(dxoutTemp))
        self.torque = np.array(self.lqr_us[index]) - self.lqr_Ks[index].dot(dxoutTemp)
        if self.ctrl_mode == '2':
            self.torque = np.array(self.lqr_us[index])

    def mpc_torque(self):
        self.updateX0()
        dxoutTemp = np.zeros(self.nv + self.nv)
        dxoutTemp[:self.nv] = difference(self.model, self.mpc_xs[0][:self.nq], self.x0[:self.nq])
        dxoutTemp[self.nv:] = self.x0[self.nq:] - self.mpc_xs[0][self.nq:]
        self.torque = np.array(self.mpc_us[0]) - self.mpc_Ks[0].dot(dxoutTemp)

    def torque_publish(self):
        print("torque pub time", time.time() - self.torquetime)
        self.torquetime = time.time()
        jointsTorque = Float32MultiArray()
        jointsTorque.data = self.torque
        self.torque_pub.publish(jointsTorque)

    def compute_torque(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if not self.lqr:
                self.mpc_torque()
            else:
                self.lqr_torque()
            self.torque_publish()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mpc_node', anonymous=True)
    package_name = "g1_description"
    file_name = "robot.urdf"
    g1_controller = controller_node(package_name, file_name)
    g1_controller.compute_torque()