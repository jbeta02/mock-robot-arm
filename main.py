from forward_kinematics import ForwardKinematics
from inverse_kinematics import InverseKinematics

from kinematics import Kinematics

def print_position(position):
    if position == None:
        print("\ninput values are out of bounds check robot arm constraints and re-enter values")

    else:
        x = position[0]
        y = position[1]
        z = position[2]

        print("position (x, y, z): {:.2f} {:.2f} {:.2f}".format(x, y, z) + "\n")

def print_rotation_matrix(matrix):

    print("rotation matrix: ")
    for row in matrix:
        print("{:.2f}  {:.2f}  {:.2f}".format(row[0], row[1], row[2]))
    print("\n")

def print_htm_matrix(matrix):

    print("\nhomogenous transformation matrix: ")
    for row in matrix:
        print("{:.2f}  {:.2f}  {:.2f}  {:.2f}".format(row[0], row[1], row[2], row[3]))

    print("\n")


#########################################################################################################


# angles based on kinematic diagram
angle1 = 0 # 20 30
angle2 = 90 # 68 92
angle3 = 0 # 160 181

link1 = 1
link2 = 1
base_height = 0

# homogenous transformation matrix kinematic implementation
kinematics = Kinematics()

end_position = kinematics.get_end_effector_position(angle1=angle1, angle2=angle2, angle3=angle3, link_len1=link1, link_len2=link2, base_height=base_height)

print_htm_matrix(end_position)

end_position_a = kinematics.get_end_effector_position_alternate(angle1=angle1, angle2=angle2, angle3=angle3, link_len1=link1, link_len2=link2, base_height=base_height)

# Denavit Hartenberg method for forward kinematics
# print_htm_matrix(end_position_a)


# forward kinematics trig implementation
angle1 = angle1
angle2 = angle2
angle3 = angle3 + 180

f_kinematics = ForwardKinematics()

# on a different coordinate system than matrix implementation
position = f_kinematics.get_position(angle1=angle1, angle2=angle2, angle3=angle3, link_len1=1, link_len2=1, base_height=0)

print_position(position)


# rotation matrix stuff
i_kinematics = InverseKinematics()

rotation_matrix = i_kinematics.get_03_rotation_matrix(theta1=0, theta2=90, theta3=0)

#print_rotation_matrix(rotation_matrix)


#########################################################################################################

# inverse kinematics implementation

angle_list = kinematics.get_joint_angles(link_len1=link1, link_len2=link2, base_height=base_height, end_x=position[0], end_y=position[1], end_z=position[2])

print("inverse kinematics\n")
print_position(angle_list)
