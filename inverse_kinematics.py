import numpy as np
import math

# inverse kinematics of anthropomorphic/articulated robot arm with 3 DOF

class InverseKinematics:

    def __init__(self):

        # thetas in radians
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0

        self.identity_matrix = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]

        # self.x_rotation = [
        #     [1, 0, 0],
        #     [0, math.cos(self.theta1), -math.sin(self.theta1)],
        #     [0, math.sin(self.theta1), math.cos(self.theta1)]
        # ]
        #
        # self.y_rotation = [
        #     [math.cos(self.theta2), 0, math.sin(self.theta2)],
        #     [0, 1, 0],
        #     [-math.sin(self.theta2), 0, math.cos(self.theta2)]
        # ]
        #
        # self.z_rotation = [
        #     [math.cos(self.theta3), -math.sin(self.theta3), 0],
        #     [math.sin(self.theta3), math.cos(self.theta3), 0],
        #     [0, 0, 1]
        # ]

    def x_rotation(self, theta):
        theta = math.radians(theta)
        return [
            [1, 0, 0],
            [0, math.cos(theta), -math.sin(theta)],
            [0, math.sin(theta), math.cos(theta)]
        ]

    def y_rotation(self, theta):
        theta = math.radians(theta)
        return [
            [math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0],
            [-math.sin(theta), 0, math.cos(theta)]
        ]

    def z_rotation(self, theta):
        theta = math.radians(theta)
        return [
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ]

    def get_03_rotation_matrix(self, theta1, theta2, theta3):

        # projection of axis_1 on axis_1 when theta1 is 0
        projection_of_1_on_0 = [
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ]

        # R_01
        R_01 = np.dot(projection_of_1_on_0, self.y_rotation(theta1))

        # R_12
        R_12 = np.dot(self.identity_matrix, self.z_rotation(theta2))

        # R_23
        R_23 = np.dot(self.identity_matrix, self.z_rotation(theta3))

        # multiply together
        return np.dot(np.dot(R_01, R_12), R_23) # same as R_01 @ R_12 @ R_23 (np.dot only takes 2 parameters)

    def build_displacement_vector(self, base_height, link1 ,link2, theta2, theta3):

        d_01 = [[0], [0], [base_height]]

        d_12 = [[link1 * math.cos(theta2)], [link1 * math.sin(theta2)], [0]]

        d_23 = [[link2 * math.cos(theta3)], [link2 * math.sin(theta3)], [0]]


    # get homogenous transformation matrix
    def get_htm(self, rotation_matrix, displacement_vector):

        htm = np.concatenate((rotation_matrix, displacement_vector), 1)

        htm = np.concatenate((htm, [[0, 0, 0, 1]]), 0)

        return htm