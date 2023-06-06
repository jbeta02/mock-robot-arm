import math
import numpy as np

class Kinematics:

    # bounds:

    # angle1 = [0, 180]
    # angle2 = (-180, 180)
    # angle3 = (-180, 180)

    def __init__(self):
        self.identity_matrix = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]


    # see kinetic diagram for angle definitions (angle 1 at base, angle 2 at second joint, angle 3 at third joint)
    # (input angles are in degrees, so need to convert to radians)
    # links measured in cm
    # implementation of forward kinematics
    def get_end_effector_position(self, angle1, angle2, angle3, link_len1, link_len2, base_height):
        # convert to radians
        angle1 = math.radians(angle1)
        angle2 = math.radians(angle2)
        angle3 = math.radians(angle3)

        # projection of axis_1 on axis_1 when angle1 is 0
        projection_of_1_on_0 = [
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ]

        # rotation matrices
        R_01 = np.dot(projection_of_1_on_0, self._y_rotation(angle1))
        R_12 = np.dot(self.identity_matrix, self._z_rotation(angle2))
        R_23 = np.dot(self.identity_matrix, self._z_rotation(angle3))

        # multiply together
        R_03 = np.dot(np.dot(R_01, R_12), R_23)  # same as R_01 @ R_12 @ R_23 (np.dot only takes 2 parameters)


        # displacement vectors
        d_01 = [[0], [0], [base_height]]
        d_12 = [[link_len1 * math.cos(angle2)], [link_len1 * math.sin(angle2)], [0]]
        d_23 = [[link_len2 * math.cos(angle3)], [link_len2 * math.sin(angle3)], [0]]


        # homogenous transformation matrices
        htm_01 = self._get_htm(R_01, d_01)
        htm_12 = self._get_htm(R_12, d_12)
        htm_23 = self._get_htm(R_23, d_23)

        # print("htm matrix 01\n", np.matrix(htm_01))
        # print("htm matrix 12\n", np.matrix(htm_12))
        # print("htm matrix 23\n", np.matrix(htm_23))

        htm_03 = np.dot(np.dot(htm_01, htm_12), htm_23)

        return htm_03


    # see kinetic diagram for angle definitions (angle 1 at base, angle 2 at second joint, angle 3 at third joint)
    # (input angles are in degrees, so need to convert to radians)
    # links measured in cm
    # implementation of forward kinematics using Denavit Hartenberg method
    def get_end_effector_position_alternate(self, angle1, angle2, angle3, link_len1, link_len2, base_height):
        # convert to radians
        angle1 = math.radians(angle1)
        angle2 = math.radians(angle2)
        angle3 = math.radians(angle3)

        # Denavit Hartenberg parameter table
        parameter_table = [
            [angle1, math.radians(90), 0, base_height],
            [angle2, 0, link_len1, 0],
            [angle3, 0, link_len2, 0]
        ]

        htms = []

        for i in range(0, 3):

            # htm for Denavit Hartenberg method
            htm_DH_i_ii = [
                [math.cos(parameter_table[i][0]), -math.sin(parameter_table[i][0]) * math.cos(parameter_table[i][1]), math.sin(parameter_table[i][0]) * math.sin(parameter_table[i][1]), parameter_table[i][2] * math.cos(parameter_table[i][0])],
                [math.sin(parameter_table[i][0]), math.cos(parameter_table[i][0]) * math.cos(parameter_table[i][1]), -math.cos(parameter_table[i][0]) * math.sin(parameter_table[i][1]), parameter_table[i][2] * math.sin(parameter_table[i][0])],
                [0, math.sin(parameter_table[i][1]), math.cos(parameter_table[i][1]), parameter_table[i][3]],
                [0, 0, 0, 1]
            ]

            htms.append(htm_DH_i_ii)

        # handle any calculating dot product of any number of frame count
        for i in range(0, len(htms)-1):

            htms[i+1] = np.dot(htms[i], htms[i+1])

        return htms[len(htms)-1]


    # implement inverse kinematics using analytical approach by graphical method and trig
    def get_joint_angles(self, link_len1, link_len2, base_height, end_x, end_y, end_z):

        # using kinematic diagram then splitting diagram into 2 2d diagrams (top view and side view)
        # then use trig to find angles based on given end effector position

        # get angle1

        # deal with x position at angle1 at 90 degrees
        if end_x == 0:
            angle1 = math.radians(90)

        # deal with x position being negative
        elif end_x != abs(end_x) and end_x != 0 and end_y != 0:
            angle1 = math.radians(180) + math.atan(end_y / end_x)

        else:
            angle1 = math.atan(end_y / end_x)


        # if andle2 is positive (elbow down)

        # get angle2
        big_tri_base = math.sqrt(end_x**2 + end_y**2)
        big_tri_height = end_z - base_height
        phi2 = math.atan(big_tri_height / big_tri_base) # left angle of big_tri

        hypot = math.sqrt(big_tri_base**2 + big_tri_height**2)
        phi1 = math.acos((link_len2**2 - link_len1**2 - hypot**2) / (-2 * link_len1 * hypot))

        angle2 = phi2 + phi1


        # get angle3
        phi3 = math.acos((hypot**2 - link_len1**2 - link_len2**2) / (-2 * link_len1 * link_len2))
        angle3 = -(math.radians(180) - phi3) # neg for elbow up (remove neg for elbow down)


        # convert angles to degrees
        angle1 = math.degrees(angle1)
        angle2 = math.degrees(angle2)
        angle3 = math.degrees(angle3)

        # print("phi1", math.degrees(phi1))
        # print("phi2", math.degrees(phi2))
        # print("phi3", math.degrees(phi3))
        # print("r1", big_tri_base)
        # print("r2", big_tri_height)
        # print("r3", hypot)
        # print("\n")

        return [angle1, angle2, angle3]


    # get homogenous transformation matrix
    def _get_htm(self, rotation_matrix, displacement_vector):
        htm = np.concatenate((rotation_matrix, displacement_vector), 1)

        htm = np.concatenate((htm, [[0, 0, 0, 1]]), 0)

        return htm


    # perform rotation on given axis
    def _x_rotation(self, theta):
        return [
            [1, 0, 0],
            [0, math.cos(theta), -math.sin(theta)],
            [0, math.sin(theta), math.cos(theta)]
        ]


    # perform rotation on given axis
    def _y_rotation(self, theta):
        return [
            [math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0],
            [-math.sin(theta), 0, math.cos(theta)]
        ]


    # perform rotation on given axis
    def _z_rotation(self, theta):
        return [
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ]