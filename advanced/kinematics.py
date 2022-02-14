import numpy as np
import math

from numpy import sin, cos

if __name__ == '__main__':
    robot_arm = RobotArm3dof(l=ARMS_LENGTHS, arduino_control=None, )
    F_end = np.zeros(1)  # Force that move the endpoint .Implement this using PID control or something else
    p, q, dq = robot_arm.move_endpoint(F_end)
    # q: angles
    print(q)


from intermediate.constants import CONTROL_DT, ARMS_LENGTHS


class RobotArm3dof:

    def __init__(self, l, reset_q=None, arduino_control=None, dt=CONTROL_DT):
        self.l = l  # link length
        if reset_q is not None:
            self.reset_q = reset_q.copy()
        else:
            self.reset_q = np.array([0.])
        self.q = self.reset_q.copy()  # joint position
        self.lambda_coeff = 0.01  # coefficient for robustness of singularity positions

        self.arduino_control = arduino_control
        self.dt = dt
        self.end_p = np.zeros(2)  # end effector position (x,z)
        self.reset()
        # For visualization
        self.arm_regions = []

    # forward kinematics (until the second elbow, i.e., secondary endpoint)
    def FK2(self):
        p = np.zeros([2])  # endpoint position
        l = self.l
        q = self.q
        p[0] = l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1])
        p[1] = l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1])
        return p

    # Jacobian matrix (until the second elbow, i.e., secondary endpoint)
    def Jacobian1(self):
        l = self.l
        q = self.q
        J = np.array([-l[0] * sin(q[0]),
                      l[0] * cos(q[0])])
        return J

    # Jacobian matrix (until the second elbow, i.e., secondary endpoint)
    def Jacobian2(self):
        l = self.l
        q = self.q
        J = np.array([[-l[0] * sin(q[0]) - l[1] * sin(q[0] + q[1]), -l[1] * sin(q[0] + q[1])],
                      [l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]), l[1] * cos(q[0] + q[1])]])
        return J

    def Jacobian3(self):
        # Jacobian for 3 angles. Very similar as Jacobian2
        l = self.l
        q = self.q
        dxq1 = -l[2] * sin(q[0] + q[1])
        dyq1 = l[2] * cos(q[0] + q[1])
        dxq2 = -l[2] * sin(q[0] + q[1] + q[2])
        dyq2 = l[2] * cos(q[0] + q[1] + q[2])
        J = np.array(
            [[-l[0] * sin(q[0]) + dxq1 + dxq2, dxq1 + dxq2, dxq2],
             [l[0] * cos(q[0]) + dyq1 + dyq2, dyq1 + dyq2, dyq2]])
        return J

    # forward kinematics (until the end of the chain, i.e., primary endpoint)
    def FK_all_points(self, other_q=None):
        l = self.l
        q = self.q
        if other_q is not None:
            q = other_q
        p1 = np.array([l[0] * cos(q[0]), l[0] * sin(q[0])])
        # p2 = p1 + l[1] * np.array([cos(q[0] + q[1]), sin(q[0] + q[1])])
        # p3 = p2 + l[2] * np.array([cos(q[0] + q[1] + q[2]), sin(q[0] + q[1] + q[2])])
        # return np.array([p1, p2, p3])
        return np.array([p1])

    def FK_end_p(self):
        return self.FK_all_points()[-1]

    # inverse kinematics (until joint 2)
    def IK2(self, p):
        q = np.zeros([2])
        r = np.sqrt(p[0] ** 2 + p[1] ** 2)
        q[1] = np.pi - math.acos((self.l[0] ** 2 + self.l[1] ** 2 - r ** 2) / (2 * self.l[0] * self.l[1]))
        q[0] = math.atan2(p[1], p[0]) - math.acos((self.l[0] ** 2 - self.l[1] ** 2 + r ** 2) / (2 * self.l[0] * r))

        return q

    # state change
    def _set_state(self, q, dq):
        self.q = q
        # self.dq = dq

    def get_dq(self, F):
        """"
        F: float[2] the endpoint movement
        """
        # KINEMATIC CONTROL
        J_end = self.Jacobian1()

        def J_robust(_J):
            _Jt = _J.transpose()
            damp_identity = self.lambda_coeff * np.identity(len(_J))
            return _Jt @ np.linalg.inv(_J @ _Jt + damp_identity)

        # if len(J_end.shape) > 1:
        J_end_robust = J_robust(J_end)

        dq = J_end_robust @ F
        # else:
        #     dq = J_end * F
        return dq

    def move_endpoint(self, F):
        """"
        F: float[2] the endpoint movement (x,z)
        """
        dq = self.get_dq(F)

        # Could add constraints to your robot arm
        # dq = self.constraint(dq)
        self.q += dq * self.dt
        self.end_p = self.FK_end_p()

        if self.arduino_control is not None:
            self.arduino_control.sent_action(self.q)

        return self.end_p, self.q, dq

    def reset(self, joint_angles=None):
        if joint_angles is None:
            self.q = self.reset_q.copy()
        else:
            self.q = joint_angles.copy()

        self.end_p = self.FK_end_p()

        if self.arduino_control is not None:
            self.arduino_control.sent_action(self.q)

    # def constraint(self, dq):
    #     # Could add constraints to your robot arm
    #     global_pos_constraint_lb = [-10, -0.1]  # lower bound global constraint
    #     p = np.zeros(2)
    #     self.arm_regions = []
    #
    #     def create_obstacles(joint_pos_new, q):
    #         obstacles = []
    #         for i in range(len(dq)):
    #             l = ARMS_LENGTHS[i]
    #             if i > 0:
    #                 p = joint_pos_new[i - 1]
    #             else:
    #                 p = np.zeros(2)
    #             obstacles.append(arm_to_polygon(*p, np.sum(q[:i + 1]), l, ARM_WIDTH))
    #         return obstacles
    #
    #     new_q = self.q + dq * self.dt
    #     # Check for base arm to not hit the base
    #     if new_q[0] < 0.5 * np.pi:
    #         dq[0] = 0
    #     # Other checks on all arms
    #     for i in range(len(dq)):
    #         new_q = self.q + dq * self.dt
    #         joint_positions_new = self.FK_all_points(new_q)
    #
    #         # Global constraint check
    #         if np.any(joint_positions_new < global_pos_constraint_lb):
    #             dq[i] = 0
    #         # Check collision with itself
    #         if i > 0:
    #             p = joint_positions_new[i - 1].copy()
    #         if not check_collision(create_obstacles(joint_positions_new, new_q)):
    #             dq[i] = 0
    #
    #         # for visualization
    #         l = ARMS_LENGTHS[i]
    #         pol = arm_to_polygon(*p, np.sum(self.q[:i + 1]), l, ARM_WIDTH)
    #         self.arm_regions.append(pol)
    #
    #     return dq
