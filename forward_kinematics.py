import math

"""

Purpose: calculate position of arm end-effector using forward/direct kinematics

arm constants: 

angle 1: [0, 180]
angle 2: (0, 180)
angle 3: (0, 180]

links > 0

"""

#TODO might need to change coordinate system when I finish implementing inverse kinematics software (see kinematic diagrams)

class ForwardKinematics:

    def __init__(self):
        pass


    # angles parameters are in degrees
    def get_position(self,
                    angle1, # angle top view (first joint)
                    angle2, # angle second joint
                    angle3, # angle third joint
                    link_len1,
                    link_len2,
                    base_height):

        # if (angle1 < 0 or angle1 > 180) or (angle2 <= 0 or angle2 >= 180) or (angle3 <= 0 or angle3 > 180):
        #     return None

        ### first triangle ###

        # print("angle 1:", angle1)
        # print("angle 2:", angle2)
        # print("angle 3:", angle3)
        # print("link 1:", link_len1)
        # print("link 2:", link_len2)
        # print("base height:", base_height)

        extension_of_link1 = link_len1 * math.cos(math.radians(angle2))
        height_of_link1 = link_len1 * math.sin(math.radians(angle2))
        angle_of_first_triangle = 90 - angle2 # created from second joint end point

        # extreme case
        if angle3 == 180:
            angle_of_first_triangle = 90

        # print("angle_of_first_triangle", angle_of_first_triangle)


        ### second triangle ###

        # second triangle case 1 (angle 3 < 180)
        if angle3 < 180:

            angle_of_second_triangle = angle3 - angle_of_first_triangle  # created from second joint end point

            # print("angle_of_second_triangle", angle_of_second_triangle)

            extension_of_link2 = link_len2 * math.sin(math.radians(angle_of_second_triangle))

            # treat as triangle where its base is created where the endpoint ends
            height_of_link2 = link_len2 * math.cos(math.radians(angle_of_second_triangle))


        if angle3 == 180:
            angle_of_second_triangle = angle3 - angle_of_first_triangle  # created from second joint end point

            # print("angle_of_second_triangle", angle_of_second_triangle)

            extension_of_link2 = extension_of_link1

            # treat as triangle where its base is created where the endpoint ends
            height_of_link2 = -height_of_link1

        # following case doesn't follow robot constraints
        # second triangle case 2 (angle 3 > 180)

        if angle3 > 180:
            angle_of_second_triangle = angle3 - angle_of_first_triangle  # created from second joint end point

            extension_of_link2 = link_len2 * math.sin(math.radians(angle_of_second_triangle))

            # treat as triangle where its base is created where the endpoint ends
            height_of_link2 = link_len2 * math.cos(math.radians(angle_of_second_triangle))


        z = height_of_link1 + base_height - height_of_link2

        # how far endpoint reaches out
        robot_arm_extension = extension_of_link1 + extension_of_link2

        # print("\nd3", height_of_link1)
        # print("\nd6", height_of_link2)
        # print("\nd4", extension_of_link1)
        # print("\nd5", extension_of_link2)
        # print("\nd1", robot_arm_extension)

        x = robot_arm_extension * math.cos(math.radians(angle1))
        y = robot_arm_extension * math.sin(math.radians(angle1))

        return [x, y, z]
