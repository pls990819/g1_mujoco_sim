import time
import os
import rospkg
import numpy as np
import crocoddyl
import rospy
import pinocchio
from pinocchio import difference
import math
from pinocchio.robot_wrapper import RobotWrapper
from g1_walking_problem import G1GaitProblem
from std_msgs.msg import Float32MultiArray,Bool,String
from geometry_msgs.msg import Pose,Twist
from nao_controller.msg import MPCxuk

rospack = rospkg.RosPack()

class mpc_node:
    def __init__(self, rightFoot, leftFoot, package_name, file_name):
        self.urdf_path = os.path.join(rospack.get_path(package_name), "urdf", file_name)
        self.robot = RobotWrapper.BuildFromURDF(self.urdf_path, [""], pinocchio.JointModelFreeFlyer())
        self.model = self.robot.model
        self.rightFoot = rightFoot
        self.leftFoot = leftFoot
        self.timeStep = 0.01
        self.supportKnots = 4
        self.ncontrol = 3
        self.lqr_timeStep = 0.01
        self.lqr_supportKnots = 1001
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nu = 23
        self.joint_state = np.zeros(2 * self.nu)
        self.body_pose = np.zeros(7)
        self.body_twist = np.zeros(6)
        self.x0 = np.zeros(self.model.nq + self.model.nv)
        self.pubtime = time.time()

        self.gait = G1GaitProblem(self.model, rightFoot, leftFoot, True)
        self.body_pose_sub = rospy.Subscriber('/bodyPose', Pose, self.body_pose_callback)
        self.body_twist_sub = rospy.Subscriber('/bodyTwist', Twist, self.body_twist_callback)
        self.joint_state_sub = rospy.Subscriber('/jointsPosVel', Float32MultiArray, self.joint_state_callback)
        self.MPC_xuk_pub = rospy.Publisher('/mpc_xuk', MPCxuk, queue_size=10)
        self.LQR_xuk_pub = rospy.Publisher('/lqr_xuk', MPCxuk, queue_size=10)
        self.ctrl_mode_sub = rospy.Subscriber('/ctrlMode', String, self.ctrl_mode_callback)

        self.last_xs = None
        self.lqr = False
        self.mpc = True
        self.mpc_us = np.zeros((self.ncontrol,self.nu))
        self.mpc_xs = np.zeros((self.ncontrol,self.nq + self.nv))
        self.mpc_Ks = np.zeros((self.ncontrol,self.nu,2 * self.nv))
        self.lqr_us = [np.zeros(self.nu)] * 2
        self.lqr_xs = [np.zeros(self.nq + self.nv)] * 2
        self.lqr_Ks = [np.zeros((self.nu,2 * self.nv))] * 2

        self.ctrl_mode = '0'
        self.stepKnotsWalking = 35
        self.supportKnotsWalking = 10

    def ctrl_mode_callback(self, msg):
        self.ctrl_mode = msg.data
        self.mpc = False

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

    def joint_state_callback(self, msg):
        self.joint_state = np.array(msg.data)

    def updateX0(self):
        self.x0[0:7] = self.body_pose
        self.x0[7:self.nq] = self.joint_state[0:self.nu]
        self.x0[self.nq:self.nq+6] = self.body_twist
        self.x0[self.nq+6:self.nq+self.nv] = self.joint_state[self.nu:2 * self.nu]

    def solve_stand(self, refState):
        # self.mpcTime = rospy.Time.now()
        locoModel = self.gait.createStandingProblem(self.x0, self.timeStep, self.supportKnots, refState)
        problemStand = crocoddyl.ShootingProblem(self.x0, locoModel[:-1], locoModel[-1])
        solverStand = crocoddyl.SolverFDDP(problemStand)
        solverStand.th_stop = 1e-7
        problemStand.x0 = self.x0
        if self.last_xs is None:
            xs = [self.x0] * (solverStand.problem.T + 1)
        else:
            xs = self.last_xs
        us = solverStand.problem.quasiStatic([self.x0] * solverStand.problem.T)
        solverStand.solve(xs, us, 6, False)
        self.last_xs = solverStand.xs
        # print(len(self.torque))
        self.mpc_xs = solverStand.xs[0:self.ncontrol]
        self.mpc_us = solverStand.us[0:self.ncontrol]
        self.mpc_Ks = solverStand.K[0:self.ncontrol]

    def solve_squat(self, refState):
        # self.mpcTime = rospy.Time.now()
        locoModel = self.gait.createSquatDownProblem(self.x0, self.lqr_timeStep, self.lqr_supportKnots, refState)
        problemStand = crocoddyl.ShootingProblem(self.x0, locoModel[:-1], locoModel[-1])
        solverStand = crocoddyl.SolverFDDP(problemStand)
        solverStand.th_stop = 1e-7
        problemStand.x0 = self.x0
        xs = [self.x0] * (solverStand.problem.T + 1)
        us = solverStand.problem.quasiStatic([self.x0] * solverStand.problem.T)
        solverStand.solve(xs, us, 100, False)
        # print(len(self.torque))
        self.lqr_xs = solverStand.xs[1:]
        self.lqr_us = solverStand.us
        self.lqr_Ks = solverStand.K
        self.mpc = True

    def solve_walking(self):
        x0 = self.x0
        self.lqr_xs = []
        self.lqr_us = []
        self.lqr_Ks = []
        GAITPHASES = [
            {
                "walking": {
                    "stepLength": 0.2,
                    "stepHeight": 0.1,
                    "timeStep": 0.01,
                    "stepKnots": self.stepKnotsWalking,
                    "supportKnots": self.supportKnotsWalking,
                }
            },
            {
                "walking": {
                    "stepLength": 0.2,
                    "stepHeight": 0.1,
                    "timeStep": 0.01,
                    "stepKnots": self.stepKnotsWalking,
                    "supportKnots": self.supportKnotsWalking,
                }
            },
            {
                "walking": {
                    "stepLength": 0.2,
                    "stepHeight": 0.1,
                    "timeStep": 0.01,
                    "stepKnots": self.stepKnotsWalking,
                    "supportKnots": self.supportKnotsWalking,
                }
            },
            {
                "walking": {
                    "stepLength": 0.2,
                    "stepHeight": 0.1,
                    "timeStep": 0.01,
                    "stepKnots": self.stepKnotsWalking,
                    "supportKnots": self.supportKnotsWalking,
                }
            },
        ]
        solver = [None] * len(GAITPHASES)
        for i, phase in enumerate(GAITPHASES):
            for key, value in phase.items():
                if key == "walking":
                    # Creating a walking problem
                    solver[i] = crocoddyl.SolverBoxFDDP(
                        self.gait.createWalkingProblem(
                            x0,
                            value["stepLength"],
                            value["stepHeight"],
                            value["timeStep"],
                            value["stepKnots"],
                            value["supportKnots"],
                        )
                    )
                    solver[i].th_stop = 1e-9

            # Added the callback functions
            # print("*** SOLVE " + key + " ***")
            # solver[i].setCallbacks(
            #     [
            #         crocoddyl.CallbackVerbose(),
            #         crocoddyl.CallbackLogger(),
            #     ]
            # )
            #   solver[i].setCallbacks([crocoddyl.CallbackVerbose()])

            # Solving the problem with the DDP solver
            xs = [x0] * (solver[i].problem.T + 1)
            us = solver[i].problem.quasiStatic([x0] * solver[i].problem.T)
            solver[i].solve(xs, us, 100, False)
            # display = crocoddyl.MeshcatDisplay(self.robot, frameNames=[self.rightFoot, self.leftFoot])
            # display.displayFromSolver(solver[i])
            # Defining the final state as initial one for the next phase
            x0 = solver[i].xs[-1]
            # self.lqr_us.append(solver[i].us)
            # # print(self.lqr_us)
            # self.lqr_xs.append(solver[i].xs[1:])
            # self.lqr_Ks.append(solver[i].K)
            print(len(list(solver[i].us)))
            print(len(list(solver[i].xs)))
            print(len(list(solver[i].K)))
            self.lqr_us += list(solver[i].us[0:self.stepKnotsWalking+self.supportKnotsWalking]) + list(solver[i].us[self.stepKnotsWalking+self.supportKnotsWalking+1:])
            self.lqr_xs += list(solver[i].xs[1:self.stepKnotsWalking+self.supportKnotsWalking+1]) + list(solver[i].xs[self.stepKnotsWalking+self.supportKnotsWalking+2:])
            self.lqr_Ks += list(solver[i].K[0:self.stepKnotsWalking+self.supportKnotsWalking]) + list(solver[i].K[self.stepKnotsWalking+self.supportKnotsWalking+1:])


    def lqr_xuk_pub(self):
        print("pub lqr")
        print("mpc pub time", time.time() - self.pubtime)
        self.pubtime = time.time()
        msg = MPCxuk()
        # msg.stamp = self.mpcTime
        # msg.ncontrol = self.lqr_supportKnots-1
        msg.ncontrol = len(self.lqr_us)
        msg.us = [item for sublist in self.lqr_us for item in sublist]
        msg.xs = [item for sublist in self.lqr_xs for item in sublist]
        msg.Ks = [item for submatrix in self.lqr_Ks for sublist in submatrix for item in sublist]
        print(len(self.lqr_us), "msg.us", len(self.lqr_us) * self.nu, len(msg.us))
        print(len(self.lqr_Ks), "msg.Ks", len(self.lqr_Ks) * self.nu * (self.nv + self.nv), len(msg.Ks))
        print(len(self.lqr_xs), "msg.xs", len(self.lqr_xs) * (self.nq + self.nv), len(msg.xs))
        self.LQR_xuk_pub.publish(msg)

    def mpc_xuk_pub(self):
        print("mpc pub time", time.time() - self.pubtime, "self.ncontrol", self.ncontrol)
        self.pubtime = time.time()
        msg = MPCxuk()
        # msg.stamp = self.mpcTime
        msg.ncontrol = self.ncontrol
        msg.us = [item for sublist in self.mpc_us for item in sublist]
        msg.xs = [item for sublist in self.mpc_xs for item in sublist]
        msg.Ks = [item for submatrix in self.mpc_Ks for sublist in submatrix for item in sublist]
        self.MPC_xuk_pub.publish(msg)

    def compute_mpc(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.mpc:
                if self.ctrl_mode == '0':
                    self.updateX0()
                    self.solve_stand(self.gait.standState)
                    self.mpc_xuk_pub()
                elif self.ctrl_mode == '1':
                    self.updateX0()
                    self.solve_stand(self.gait.defaultState)
                    self.mpc_xuk_pub()
                # if self.ctrl_mode == '2':
                #     self.updateX0()
                #     self.solve_stand(self.gait.standState)
                #     self.mpc_xuk_pub()
            else:
                print(self.ctrl_mode)
                if self.ctrl_mode == '1':
                    self.updateX0()
                    self.solve_squat(self.gait.defaultState)
                    self.lqr_xuk_pub()
                elif self.ctrl_mode == '0':
                    self.updateX0()
                    self.solve_squat(self.gait.standState)
                    self.lqr_xuk_pub()
                else:
                    self.updateX0()
                    self.solve_walking()
                    self.lqr_xuk_pub()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mpc_node', anonymous=True)
    package_name = "g1_description"
    file_name = "robot.urdf"
    rightFoot = "right_ankle_roll_link"
    leftFoot = "left_ankle_roll_link"
    nao_controller = mpc_node(rightFoot, leftFoot, package_name, file_name)
    nao_controller.compute_mpc()