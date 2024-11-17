import numpy as np
import pinocchio
import math
import crocoddyl

class GaitParameter:
    def __int__(self):
        # mode 1 stand
        # mode 2 squat
        # mode 3 walk
        self.mode = 1
        self.comTrackWeight = 10
        self.wrenchConeWeight = 0.01
        self.footTrackWeight = 0.2
        self.statePoseWeight = 10
        self.stateJointPosWeight = 10
        self.stateTwistWeight = 10
        self.stateJointVelWeight = 0
        self.stateWeight = 2.5
        self.ctrlWeight = 0.01
        self.impulseFootTrackWeight = 1e8
        self.impulseStateJointPosWeight = 0.1
        self.impulseStateJointVelWeight = 0.1
        self.impulseStateWeight = 1e1

class G1GaitProblem:
    def __init__(
            self,
            rmodel,
            rightFoot,
            leftFoot,
            # gaitmode,
            isfirststep=True,
            integrator="euler",
            control="zero",
            fwddyn=True,
    ):
        # self.mode = gaitmode # 默认站立
        self.rmodel = rmodel
        self.rdata = rmodel.createData()
        self.state = crocoddyl.StateMultibody(self.rmodel)
        self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)
        # Getting the frame id for all the legs
        self.rfId = self.rmodel.getFrameId(rightFoot)
        self.lfId = self.rmodel.getFrameId(leftFoot)
        self._integrator = integrator
        self._control = control
        self._fwddyn = fwddyn
        # Defining default state
        self.rmodel.defaultState = self.standstate()
        self.defaultState = self.defaultstate()
        self.standState = self.standstate()
        self.standcomref = pinocchio.centerOfMass(self.rmodel, self.rdata, self.standState[:self.rmodel.nq])[2]
        self.defaultcomref = pinocchio.centerOfMass(self.rmodel, self.rdata, self.defaultState[:self.rmodel.nq])[2]
        self.firstStep = isfirststep
        # Defining the friction coefficient and normal
        self.mu = 0.7
        self.Rsurf = np.eye(3)


        # 初始化一些步态参数

        self.standMode = GaitParameter()
        self.squatMode = GaitParameter()
        self.walkingMode = GaitParameter()
        # print("self.walkingMode.mode",self.walkingMode.mode)

        self.standMode.comTrackWeight = 10
        self.standMode.wrenchConeWeight = 0.01
        self.standMode.footTrackWeight = 0.2
        self.standMode.statePoseWeight = 100
        self.standMode.stateJointPosWeight = 10
        self.standMode.stateTwistWeight = 100
        self.standMode.stateJointVelWeight = 0
        self.standMode.stateWeight = 4.5
        self.standMode.ctrlWeight = 0.01
        self.standMode.impulseFootTrackWeight = 30
        self.standMode.impulseStateJointPosWeight = 10
        self.standMode.impulseStateJointVelWeight = 0
        self.standMode.impulseStateWeight = 1

        self.squatMode.comTrackWeight = 10
        self.squatMode.wrenchConeWeight = 0.01
        self.squatMode.footTrackWeight = 0.2
        self.squatMode.statePoseWeight = 10
        self.squatMode.stateJointPosWeight = 10
        self.squatMode.stateTwistWeight = 10
        self.squatMode.stateJointVelWeight = 0
        self.squatMode.stateWeight = 3
        self.squatMode.ctrlWeight = 0.1
        self.squatMode.impulseFootTrackWeight = 20
        self.squatMode.impulseStateJointPosWeight = 10
        self.squatMode.impulseStateJointVelWeight = 0
        self.squatMode.impulseStateWeight = 1

        # self.squatMode.comTrackWeight = 10
        # self.squatMode.wrenchConeWeight = 0.1
        # self.squatMode.footTrackWeight = 0.2
        # self.squatMode.statePoseWeight = 10
        # self.squatMode.stateJointPosWeight = 10
        # self.squatMode.stateTwistWeight = 10
        # self.squatMode.stateJointVelWeight = 10
        # self.squatMode.stateWeight = 3
        # self.squatMode.ctrlWeight = 0.1
        # self.squatMode.impulseFootTrackWeight = 20
        # self.squatMode.impulseStateJointPosWeight = 10
        # self.squatMode.impulseStateJointVelWeight = 10
        # self.squatMode.impulseStateWeight = 1

# 正常
        self.walkingMode.comTrackWeight = 10
        # self.walkingMode.comTrackWeight = 1e6
        self.walkingMode.wrenchConeWeight = 1000
        self.walkingMode.footTrackWeight = 1000000
        self.walkingMode.statePoseWeight = 100
        self.walkingMode.stateJointPosWeight = 10
        self.walkingMode.stateTwistWeight = 10
        self.walkingMode.stateJointVelWeight = 10
        self.walkingMode.stateWeight = 3
        self.walkingMode.ctrlWeight = 0.1
        self.walkingMode.impulseFootTrackWeight = 1e2
        self.walkingMode.impulseStateJointPosWeight = 1
        self.walkingMode.impulseStateJointVelWeight = 10
        self.walkingMode.impulseStateWeight = 1e1


        # 慢
        # self.walkingMode.comTrackWeight = 10
        # self.walkingMode.wrenchConeWeight = 100
        # self.walkingMode.footTrackWeight = 100
        # self.walkingMode.statePoseWeight = 0
        # self.walkingMode.stateJointPosWeight = 100
        # self.walkingMode.stateTwistWeight = 100
        # self.walkingMode.stateJointVelWeight = 100
        # self.walkingMode.stateWeight = 3
        # self.walkingMode.ctrlWeight = 0.1
        # self.walkingMode.impulseFootTrackWeight = 100
        # self.walkingMode.impulseStateJointPosWeight = 100
        # self.walkingMode.impulseStateJointVelWeight = 10
        # self.walkingMode.impulseStateWeight = 10

        # self.walkingMode.comTrackWeight = 1
        # self.walkingMode.wrenchConeWeight = 0.01
        # self.walkingMode.footTrackWeight = 5
        # self.walkingMode.statePoseWeight = 0
        # self.walkingMode.stateJointPosWeight = 10
        # self.walkingMode.stateTwistWeight = 10
        # self.walkingMode.stateJointVelWeight = 10
        # self.walkingMode.stateWeight = 10
        # self.walkingMode.ctrlWeight = 0.1
        # self.walkingMode.impulseFootTrackWeight = 20
        # self.walkingMode.impulseStateJointPosWeight = 10
        # self.walkingMode.impulseStateJointVelWeight = 0
        # self.walkingMode.impulseStateWeight = 1


    def defaultstate(self):
        q0 = pinocchio.utils.zero(self.rmodel.nq)
        q0[6] = 1
        q0[2] = 0.62
        q0[7] = -math.pi / 8
        q0[10] = math.pi / 4
        q0[11] = -math.pi / 8
        q0[13] = -math.pi / 8
        q0[16] = math.pi / 4
        q0[17] = -math.pi / 8
        return np.concatenate([q0, np.zeros(self.rmodel.nv)])

    def standstate(self):
        q0 = pinocchio.utils.zero(self.rmodel.nq)
        q0[6] = 1
        q0[2] = 0.75
        return np.concatenate([q0, np.zeros(self.rmodel.nv)])


    def createSquatDownProblem(
            self, x0, timeStep, supportKnots, refStste
    ):
        """Create a shooting problem for a simple walking gait.

        :param x0: initial state
        :param timeStep: step time for each knot
        :param supportKnots: number of knots for double support phases
        :return shooting problem
        """
        # Compute the current foot positions
        # self.rmodel.defaultState = self.defaultstate()

        self.rmodel.defaultState = refStste
        q0 = x0[: self.state.nq]
        pinocchio.forwardKinematics(self.rmodel, self.rdata, q0)
        pinocchio.updateFramePlacements(self.rmodel, self.rdata)
        rfPos0 = self.rdata.oMf[self.rfId].translation
        lfPos0 = self.rdata.oMf[self.lfId].translation
        comRef = (rfPos0 + lfPos0) / 2
        curcom = pinocchio.centerOfMass(self.rmodel, self.rdata, q0)[2]
        # Defining the action models along the time instances
        loco3dModel = []
        for k in range(supportKnots):
            if curcom - 0.004 * k > self.defaultcomref:
                comRef[2] = curcom - 0.004 * k
            else:
                comRef[2] = self.defaultcomref
            loco3dModel += [
                self.createSwingFootModel(timeStep, [self.rfId, self.lfId], self.squatMode, comRef)
            ]

        return loco3dModel

    def createStandingProblem(
            self, x0,  timeStep,   supportKnots, refState
    ):
        """Create a shooting problem for a simple walking gait.

        :param x0: initial state
        :param timeStep: step time for each knot
        :param supportKnots: number of knots for double support phases
        :return shooting problem
        """
        # Compute the current foot positions
        self.rmodel.defaultState = refState
        q0 = x0[: self.state.nq]
        pinocchio.forwardKinematics(self.rmodel, self.rdata, q0)
        pinocchio.updateFramePlacements(self.rmodel, self.rdata)
        rfPos0 = self.rdata.oMf[self.rfId].translation
        lfPos0 = self.rdata.oMf[self.lfId].translation
        comRef = (rfPos0 + lfPos0) / 2
        # comRef[2] = pinocchio.centerOfMass(self.rmodel, self.rdata, q0)[2]
        comRef[2] = self.standcomref
        # Defining the action models along the time instances
        loco3dModel = []
        for k in range(supportKnots):
            loco3dModel += [
                self.createSwingFootModel(timeStep, [self.rfId, self.lfId], self.standMode, comRef)
            ]

        return loco3dModel

    def createWalkingProblem(
        self,
        x0,
        stepLength,
        stepHeight,
        timeStep,
        stepKnots,
        supportKnots
    ):
        """Create a shooting problem for a simple walking gait.

        :param x0: initial state
        :param stepLength: step length
        :param stepHeight: step height
        :param timeStep: step time for each knot
        :param stepKnots: number of knots for step phases
        :param supportKnots: number of knots for double support phases
        :return shooting problem
        """
        # Compute the current foot positions

        self.rmodel.defaultState = self.defaultstate()

        q0 = x0[: self.state.nq]
        pinocchio.forwardKinematics(self.rmodel, self.rdata, q0)
        pinocchio.updateFramePlacements(self.rmodel, self.rdata)
        rfPos0 = self.rdata.oMf[self.rfId].translation
        lfPos0 = self.rdata.oMf[self.lfId].translation
        comRef = (rfPos0 + lfPos0) / 2
        # comRef[2] = pinocchio.centerOfMass(self.rmodel, self.rdata, q0)[2]
        comRef[2] = self.defaultcomref
        # Defining the action models along the time instances
        loco3dModel = []
        doubleSupport = [
            self.createSwingFootModel(timeStep, [self.rfId, self.lfId], self.standMode)
            for k in range(supportKnots)
        ]

        # Creating the action models for three steps
        if self.firstStep is True:
            rStep = self.createFootstepModels(
                comRef,
                [rfPos0],
                0.5 * stepLength,
                stepHeight,
                timeStep,
                stepKnots,
                [self.lfId],
                [self.rfId],
            )
            self.firstStep = False
        else:
            rStep = self.createFootstepModels(
                comRef,
                [rfPos0],
                stepLength,
                stepHeight,
                timeStep,
                stepKnots,
                [self.lfId],
                [self.rfId],
            )
        lStep = self.createFootstepModels(
            comRef,
            [lfPos0],
            stepLength,
            stepHeight,
            timeStep,
            stepKnots,
            [self.rfId],
            [self.lfId],
        )

        # We defined the problem as:
        loco3dModel += doubleSupport + rStep
        loco3dModel += doubleSupport + lStep

        problem = crocoddyl.ShootingProblem(x0, loco3dModel[:-1], loco3dModel[-1])
        return problem

    def createFootstepModels(
        self,
        comPos0,
        feetPos0,
        stepLength,
        stepHeight,
        timeStep,
        numKnots,
        supportFootIds,
        swingFootIds,
    ):
        """Action models for a footstep phase.

        :param comPos0, initial CoM position
        :param feetPos0: initial position of the swinging feet
        :param stepLength: step length
        :param stepHeight: step height
        :param timeStep: time step
        :param numKnots: number of knots for the footstep phase
        :param supportFootIds: Ids of the supporting feet
        :param swingFootIds: Ids of the swinging foot
        :return footstep action models
        """
        numLegs = len(supportFootIds) + len(swingFootIds)
        # comPercentage = float(len(swingFootIds)) / numLegs
        comPercentage = 0.5

        # Action models for the foot swing
        footSwingModel = []
        for k in range(numKnots):
            swingFootTask = []
            for i, p in zip(swingFootIds, feetPos0):
                # Defining a foot swing task given the step length. The swing task
                # is decomposed on two phases: swing-up and swing-down. We decide
                # deliveratively to allocated the same number of nodes (i.e. phKnots)
                # in each phase. With this, we define a proper z-component for the
                # swing-leg motion.
                phKnots = numKnots / 2
                if k < phKnots:
                    dp = np.array(
                        [stepLength * (k + 1) / numKnots, 0.0, stepHeight * k / phKnots]
                    )
                elif k == phKnots:
                    dp = np.array([stepLength * (k + 1) / numKnots, 0.0, stepHeight])
                else:
                    dp = np.array(
                        [
                            stepLength * (k + 1) / numKnots,
                            0.0,
                            stepHeight * (1 - float(k - phKnots) / phKnots),
                        ]
                    )
                tref = p + dp

                swingFootTask += [[i, pinocchio.SE3(np.eye(3), tref)]]

            comTask = (
                np.array([stepLength * (k + 1) / numKnots, 0.0, 0.0]) * comPercentage
                + comPos0
            )
            footSwingModel += [
                self.createSwingFootModel(
                    timeStep,
                    supportFootIds,
                    self.walkingMode,
                    comTask=comTask,
                    swingFootTask=swingFootTask,
                )
            ]

        # Action model for the foot switch
        footSwitchModel = self.createFootSwitchModel(supportFootIds, swingFootTask, self.walkingMode)

        # Updating the current foot position for next step
        comPos0 += [stepLength * comPercentage, 0.0, 0.0]
        for p in feetPos0:
            p += [stepLength, 0.0, 0.0]
        print("len(footSwingModel)", len(list(footSwingModel)))
        print("len([*footSwingModel, footSwitchModel])", len(list([*footSwingModel, footSwitchModel])))
        return [*footSwingModel, footSwitchModel]

    def createSwingFootModel(
        self,
        timeStep,
        supportFootIds,
        gaitPara,
        comTask=None,
        swingFootTask=None
    ):
        """Action model for a swing foot phase.

        :param timeStep: step duration of the action model
        :param supportFootIds: Ids of the constrained feet
        :param comTask: CoM task
        :param swingFootTask: swinging foot task
        :return action model for a swing foot phase
        """
        # Creating a 6D multi-contact model, and then including the supporting
        # foot
        if self._fwddyn:
            nu = self.actuation.nu
        else:
            nu = self.state.nv + 6 * len(supportFootIds)
        contactModel = crocoddyl.ContactModelMultiple(self.state, nu)
        for i in supportFootIds:
            supportContactModel = crocoddyl.ContactModel6D(
                self.state,
                i,
                pinocchio.SE3.Identity(),
                pinocchio.LOCAL_WORLD_ALIGNED,
                nu,
                np.array([0.0, 30.0]),
            )
            contactModel.addContact(
                self.rmodel.frames[i].name + "_contact", supportContactModel
            )

        # Creating the cost model for a contact phase
        costModel = crocoddyl.CostModelSum(self.state, nu)
        if isinstance(comTask, np.ndarray):
            comResidual = crocoddyl.ResidualModelCoMPosition(self.state, comTask, nu)
            comTrack = crocoddyl.CostModelResidual(self.state, comResidual)
            costModel.addCost("comTrack", comTrack, gaitPara.comTrackWeight)
        for i in supportFootIds:
            cone = crocoddyl.WrenchCone(self.Rsurf, self.mu, np.array([0.2, 0.05]))
            wrenchResidual = crocoddyl.ResidualModelContactWrenchCone(
                self.state, i, cone, nu, self._fwddyn
            )
            wrenchActivation = crocoddyl.ActivationModelQuadraticBarrier(
                crocoddyl.ActivationBounds(cone.lb, cone.ub)
            )
            wrenchCone = crocoddyl.CostModelResidual(
                self.state, wrenchActivation, wrenchResidual
            )
            costModel.addCost(
                self.rmodel.frames[i].name + "_wrenchCone", wrenchCone, gaitPara.wrenchConeWeight
            )

        if swingFootTask is not None:
            for i in swingFootTask:
                framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
                    self.state, i[0], i[1], nu
                )
                footTrack = crocoddyl.CostModelResidual(
                    self.state, framePlacementResidual
                )
                costModel.addCost(
                    self.rmodel.frames[i[0]].name + "_footTrack", footTrack, gaitPara.footTrackWeight
                )

        stateWeights = np.array(
            [0] * 3 + [gaitPara.statePoseWeight] * 3 +
            [gaitPara.stateJointPosWeight] * (self.state.nv - 6) +
            [0] * 3 + [gaitPara.stateTwistWeight] * 3 +
            [gaitPara.stateJointVelWeight] * (self.state.nv - 6)
        )
        stateResidual = crocoddyl.ResidualModelState(
            self.state, self.rmodel.defaultState, nu
        )
        stateActivation = crocoddyl.ActivationModelWeightedQuad(stateWeights**2)
        stateReg = crocoddyl.CostModelResidual(
            self.state, stateActivation, stateResidual
        )
        if self._fwddyn:
            ctrlResidual = crocoddyl.ResidualModelControl(self.state, nu)
        else:
            ctrlResidual = crocoddyl.ResidualModelJointEffort(
                self.state, self.actuation, nu
            )
        ctrlReg = crocoddyl.CostModelResidual(self.state, ctrlResidual)
        costModel.addCost("stateReg", stateReg, gaitPara.stateWeight)
        costModel.addCost("ctrlReg", ctrlReg, gaitPara.ctrlWeight)

        # Creating the action model for the KKT dynamics with simpletic Euler
        # integration scheme
        if self._fwddyn:
            dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(
                self.state, self.actuation, contactModel, costModel, 0.0, True
            )
        else:
            dmodel = crocoddyl.DifferentialActionModelContactInvDynamics(
                self.state, self.actuation, contactModel, costModel
            )
        if self._control == "one":
            control = crocoddyl.ControlParametrizationModelPolyOne(nu)
        elif self._control == "rk4":
            control = crocoddyl.ControlParametrizationModelPolyTwoRK(
                nu, crocoddyl.RKType.four
            )
        elif self._control == "rk3":
            control = crocoddyl.ControlParametrizationModelPolyTwoRK(
                nu, crocoddyl.RKType.three
            )
        else:
            control = crocoddyl.ControlParametrizationModelPolyZero(nu)
        if self._integrator == "euler":
            model = crocoddyl.IntegratedActionModelEuler(dmodel, control, timeStep)
        elif self._integrator == "rk4":
            model = crocoddyl.IntegratedActionModelRK4(dmodel, control, timeStep)
        return model

    def createFootSwitchModel(self, supportFootIds, swingFootTask, gaitPara):
        """Action model for a foot switch phase.

        :param supportFootIds: Ids of the constrained feet
        :param swingFootTask: swinging foot task
        :param pseudoImpulse: true for pseudo-impulse models, otherwise it uses the
            impulse model
        :return action model for a foot switch phase
        """
        # if pseudoImpulse:
        #     return self.createPseudoImpulseModel(supportFootIds, swingFootTask)
        # else:
        return self.createImpulseModel(supportFootIds, swingFootTask, gaitPara)

    def createImpulseModel(self, supportFootIds, swingFootTask, gaitPara):
        """Action model for impulse models.

        An impulse model consists of describing the impulse dynamics against a set of
        contacts.
        :param supportFootIds: Ids of the constrained feet
        :param swingFootTask: swinging foot task
        :return impulse action model
        """
        # Creating a 6D multi-contact model, and then including the supporting foot
        impulseModel = crocoddyl.ImpulseModelMultiple(self.state)
        for i in supportFootIds:
            supportContactModel = crocoddyl.ImpulseModel6D(
                self.state, i, pinocchio.LOCAL_WORLD_ALIGNED
            )
            impulseModel.addImpulse(
                self.rmodel.frames[i].name + "_impulse", supportContactModel
            )

        # Creating the cost model for a contact phase
        costModel = crocoddyl.CostModelSum(self.state, 0)
        if swingFootTask is not None:
            for i in swingFootTask:
                frameTranslationResidual = crocoddyl.ResidualModelFrameTranslation(
                    self.state, i[0], i[1].translation, 0
                )
                footTrack = crocoddyl.CostModelResidual(
                    self.state, frameTranslationResidual
                )
                costModel.addCost(
                    self.rmodel.frames[i[0]].name + "_footTrack", footTrack, 1e8
                )

        stateWeights = np.array(
            [0.0] * 6 +
            [gaitPara.impulseStateJointPosWeight] * (self.rmodel.nv - 6) +
            [0.0] * 6 +
            [gaitPara.impulseStateJointVelWeight] * (self.rmodel.nv - 6)
        )
        stateResidual = crocoddyl.ResidualModelState(
            self.state, self.rmodel.defaultState, 0
        )
        stateActivation = crocoddyl.ActivationModelWeightedQuad(stateWeights**2)
        stateReg = crocoddyl.CostModelResidual(
            self.state, stateActivation, stateResidual
        )
        costModel.addCost("stateReg", stateReg, gaitPara.impulseStateWeight)

        # Creating the action model for the KKT dynamics with simpletic Euler
        # integration scheme
        model = crocoddyl.ActionModelImpulseFwdDynamics(
            self.state, impulseModel, costModel
        )
        return model


def plotSolution(solver, bounds=True, figIndex=1, figTitle="", show=True):
    import matplotlib.pyplot as plt

    xs, us, cs = [], [], []
    if bounds:
        us_lb, us_ub = [], []
        xs_lb, xs_ub = [], []

    def updateTrajectories(solver):
        xs.extend(solver.xs[:-1])
        for m, d in zip(solver.problem.runningModels, solver.problem.runningDatas):
            if hasattr(m, "differential"):
                cs.append(d.differential.multibody.pinocchio.com[0])
                us.append(d.differential.multibody.joint.tau)
                if bounds and isinstance(
                    m.differential, crocoddyl.DifferentialActionModelContactFwdDynamics
                ):
                    us_lb.extend([m.u_lb])
                    us_ub.extend([m.u_ub])
            else:
                cs.append(d.multibody.pinocchio.com[0])
                us.append(np.zeros(nu))
                if bounds:
                    us_lb.append(np.nan * np.ones(nu))
                    us_ub.append(np.nan * np.ones(nu))
            if bounds:
                xs_lb.extend([m.state.lb])
                xs_ub.extend([m.state.ub])

    if isinstance(solver, list):
        for s in solver:
            rmodel = solver[0].problem.runningModels[0].state.pinocchio
            nq, nv, nu = (
                rmodel.nq,
                rmodel.nv,
                solver[0].problem.runningModels[0].differential.actuation.nu,
            )
            updateTrajectories(s)
    else:
        rmodel = solver.problem.runningModels[0].state.pinocchio
        nq, nv, nu = (
            rmodel.nq,
            rmodel.nv,
            solver.problem.runningModels[0].differential.actuation.nu,
        )
        updateTrajectories(solver)

    # Getting the state and control trajectories
    nx = nq + nv
    X = [0.0] * nx
    U = [0.0] * nu
    if bounds:
        U_LB = [0.0] * nu
        U_UB = [0.0] * nu
        X_LB = [0.0] * nx
        X_UB = [0.0] * nx
    for i in range(nx):
        X[i] = [x[i] for x in xs]
        if bounds:
            X_LB[i] = [x[i] for x in xs_lb]
            X_UB[i] = [x[i] for x in xs_ub]
    for i in range(nu):
        U[i] = [u[i] for u in us]
        if bounds:
            U_LB[i] = [u[i] for u in us_lb]
            U_UB[i] = [u[i] for u in us_ub]

    # Plotting the joint positions, velocities and torques
    plt.figure(figIndex)
    plt.suptitle(figTitle)
    legJointNames = ["1", "2", "3", "4", "5", "6"]
    # left foot
    plt.subplot(2, 3, 1)
    plt.title("joint position [rad]")
    [plt.plot(X[k], label=legJointNames[i]) for i, k in enumerate(range(7, 13))]
    if bounds:
        [plt.plot(X_LB[k], "--r") for i, k in enumerate(range(7, 13))]
        [plt.plot(X_UB[k], "--r") for i, k in enumerate(range(7, 13))]
    plt.ylabel("LF")
    plt.legend()
    plt.subplot(2, 3, 2)
    plt.title("joint velocity [rad/s]")
    [
        plt.plot(X[k], label=legJointNames[i])
        for i, k in enumerate(range(nq + 6, nq + 12))
    ]
    if bounds:
        [plt.plot(X_LB[k], "--r") for i, k in enumerate(range(nq + 6, nq + 12))]
        [plt.plot(X_UB[k], "--r") for i, k in enumerate(range(nq + 6, nq + 12))]
    plt.ylabel("LF")
    plt.legend()
    plt.subplot(2, 3, 3)
    plt.title("joint torque [Nm]")
    [plt.plot(U[k], label=legJointNames[i]) for i, k in enumerate(range(0, 6))]
    if bounds:
        [plt.plot(U_LB[k], "--r") for i, k in enumerate(range(0, 6))]
        [plt.plot(U_UB[k], "--r") for i, k in enumerate(range(0, 6))]
    plt.ylabel("LF")
    plt.legend()

    # right foot
    plt.subplot(2, 3, 4)
    [plt.plot(X[k], label=legJointNames[i]) for i, k in enumerate(range(13, 19))]
    if bounds:
        [plt.plot(X_LB[k], "--r") for i, k in enumerate(range(13, 19))]
        [plt.plot(X_UB[k], "--r") for i, k in enumerate(range(13, 19))]
    plt.ylabel("RF")
    plt.xlabel("knots")
    plt.legend()
    plt.subplot(2, 3, 5)
    [
        plt.plot(X[k], label=legJointNames[i])
        for i, k in enumerate(range(nq + 12, nq + 18))
    ]
    if bounds:
        [plt.plot(X_LB[k], "--r") for i, k in enumerate(range(nq + 12, nq + 18))]
        [plt.plot(X_UB[k], "--r") for i, k in enumerate(range(nq + 12, nq + 18))]
    plt.ylabel("RF")
    plt.xlabel("knots")
    plt.legend()
    plt.subplot(2, 3, 6)
    [plt.plot(U[k], label=legJointNames[i]) for i, k in enumerate(range(6, 12))]
    if bounds:
        [plt.plot(U_LB[k], "--r") for i, k in enumerate(range(6, 12))]
        [plt.plot(U_UB[k], "--r") for i, k in enumerate(range(6, 12))]
    plt.ylabel("RF")
    plt.xlabel("knots")
    plt.legend()

    plt.figure(figIndex + 1)
    rdata = rmodel.createData()
    Cx = []
    Cy = []
    for x in xs:
        q = x[: rmodel.nq]
        c = pinocchio.centerOfMass(rmodel, rdata, q)
        Cx.append(c[0])
        Cy.append(c[1])
    plt.plot(Cx, Cy)
    plt.title("CoM position")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True)
    if show:
        plt.show()
