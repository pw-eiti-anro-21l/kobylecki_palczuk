import csv
import math
import os
import random

"""
WARNING:
xyz -> zxy (in visual), rpy -> ypr (in xml)
theta -> alpha, alpha -> theta (in .csv)
"""

# theta_sum = 0

# takes filename and path of csv file and outputs DH matrix as table
def read_from_csv(filename):
    dh=[]
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            dh.append(row)
    return dh
# a, d, theta, alpha

# writes to a given location a urdf.xml file with robot data
# WARNING: xyz -> zxy (in visual), rpy -> ypr (in xml)
def create_urdf(filename_out, rob_name, filename_in):
    dh = read_from_csv(filename_in)
    dh = dh[1:]
    orix = 0
    oriy = 0
    oriz = 0
    matr = [[1,0,0,0,"base"]]
    for line in dh:
        matr.append(line)
    dh = matr
    with open(filename_out, 'w+') as file:
        file.write(f'<robot name="{str(rob_name)}">\n\n')
        for i in range(len(dh)):
            name = dh[i][4]

            # link
            file.write(f'<link name="{name}">\n')

            
            # inertia

            # file.write(f'  <inertial>\n')
            # file.write(f'    <mass value="1"/>\n')
            # file.write(f'    <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100" />\n')
            # file.write(f'    <origin/>\n')
            # file.write(f'  </inertial>\n\n')

            # visual


            # length = math.sqrt(math.pow(float(dh[i][0]), 2) + math.pow(float(dh[i][1]), 2))
            is_newA = False
            for index in range(len(str(dh[i][0]))):
                if str(dh[i][0])[index] == ':':
                    prismatic_lower_limit = float(dh[i][0][:index])
                    prismatic_upper_limit = float(dh[i][0][index+1:])
                    is_newA = True
            if is_newA:
                newA = prismatic_lower_limit
            else:
                newA = float(dh[i][0])

            file.write('  <visual>\n')
            file.write(f'    <origin xyz="{calculate_link_origin(newA, dh[i][1], dh[i][2], dh[i][3])}" rpy="0 0 0"/>\n') # {float(dh[i][0])/2} {float(dh[i][1])/2} # TRZEBA ZMIENIĆ RPY
            file.write('    <geometry>\n')
            file.write(f'      <box size="0.5 0.5 {newA}" />\n') # random.random()
            file.write('    </geometry>\n')
            materials = [("green","0 1 0 0.5"), ("magenta", "1 0 1 0.5")]
            file.write(f'    <material name="{random.random()}">\n') #materials[i%2][0]}">\n')
            file.write(f'      <color rgba="{random.random()} {random.random()} {random.random()} 0.5" />\n') #{materials[i%2][1]}" />\n')
            file.write('    </material>\n')
            file.write('  </visual>\n')

            
            # collision

            # file.write('  <collision>\n')
            # file.write(f'    <origin xyz="{0} 0 {0}" rpy="0 {1.57} {0}" />\n')
            # file.write('    <geometry>\n')
            # file.write(f'      <cylinder radius="0.01" length="{length}" />\n')
            # file.write('    </geometry>\n')
            # file.write('    <contact_coefficients mu="0" kp="1000.0" kd="1.0"/>\n')
            # file.write('  </collision>\n')

            file.write('</link>\n\n')

            # joint
            if i >= 1:
                prev_name = dh[i-1][4]
                joint_name = prev_name + "_to_" + name
                # types = {'0': "fixed", "var": "revolute"}

                is_prev_newA = False
                for index in range(len(str(dh[i-1][0]))):
                    if str(dh[i-1][0])[index] == ':':
                        prismatic_lower_limit = float(dh[i-1][0][:index])
                        prismatic_upper_limit = float(dh[i-1][0][index+1:])
                        is_prev_newA = True
                if is_prev_newA:
                    prev_newA = prismatic_lower_limit
                else:
                    prev_newA = float(dh[i-1][0])

                # joint_type
                # one joint cant move and change angle!
                is_fixed = True
                if dh[i][3] == "var":
                    joint_type = "revolute"
                    is_fixed = False
                    
                for index in range(len(str(dh[i][0]))):
                    if dh[i][0][index] == ':':
                        joint_type = "prismatic"
                        is_fixed = False
                        prismatic_lower_limit = float(dh[i][0][:index])
                        prismatic_upper_limit = float(dh[i][0][index+1:])

                if is_fixed:
                    joint_type = "fixed"

                file.write(f'<joint name="{joint_name}" type="{joint_type}">\n') #{types[str(dh[i][3])]}">\n')
                file.write(f'  <origin xyz="{calculate_joint_origin(prev_newA, dh[i-1][1], dh[i-1][2], dh[i-1][3])}" rpy="{float(dh[i][3]) if dh[i][3] != "var" else "0"} 0 {float(dh[i][2])}"/>\n') # {calculate_theta_joint_origin(dh[i-1]) if i>0 else "0 0 0"}"/>\n')#  {orix - float(dh[i][1])} 0 {oriz - float(dh[i][0])}" />\n') # CHYBA ŹLE? ALE NWM, dla przypadku statycznego to chyba nawet ok
                file.write(f'  <parent link="{prev_name}"/>\n')
                file.write(f'  <child link="{name}"/>\n')
                
                if joint_type != "fixed":
                    if joint_type != "prismatic":
                        prismatic_upper_limit = 0
                        prismatic_lower_limit = 0
                    axises = {"revolute": "0 1 0", "prismatic": "0 0 1"}
                    upper_limits = {"revolute": "1.57075", "prismatic": str(prismatic_upper_limit)}
                    lower_limits = {"revolute": "-1.57075", "prismatic": str(prismatic_lower_limit)}
                    file.write(f'  <axis xyz="{axises[joint_type]}" />\n')
                    file.write(f'  <limit upper="{upper_limits[joint_type]}" lower="{lower_limits[joint_type]}" effort="10" velocity="10" />\n')

                file.write('</joint>\n\n')

            # sum_theta(dh[i])
            # orix = float(dh[i][1])
            # oriz = 0 #float(dh[i][0])

        file.write('</robot>\n')
        print("URDF file generated successfully.")

def calculate_joint_origin(a, d, theta, alpha): # all from previous row
    orix = 0 #(float(a)*math.sin(float(theta)))
    oriy = float(d) # cos trzeba zmienic
    oriz = float(a) #*math.cos(float(theta)))
    return str(orix) + " " + str(oriy) + " " + str(oriz)

def calculate_link_origin(a, d, theta, alpha):
    orix = 0 #(float(a)*math.sin(float(theta)))/2
    oriy = float(d)/2 # cos trzeba zmienic
    oriz = float(a)/2 #(float(a)*math.cos(float(theta)))/2
    return str(orix) + " " + str(oriy) + " " + str(oriz)

# def calculate_theta_link_origin(row): # data from previous row
#     [a, d, theta, alpha] = row[:4]
#     return "0 " + str(theta) + " 0"
    # return "0 " + str(row.split()[2]) + "0"

# def calculate_theta_joint_origin(prev_row): # data from previous row
    # global theta_sum
    # [a, d, theta, alpha] = prev_row[:4]
    # return "0 " + str(theta) + " 0"
    # return "0 " + str(prev_row.split()[2]) + "0"

# def sum_theta(row):
#     global theta_sum
#     [a, d, theta, alpha] = row[:4]
#     theta_sum += float(theta)
#     theta_sum = normalize_angle(theta_sum)

def normalize_angle(angle):
    while angle > math.pi * 2 or angle < -math.pi * 2:
        if angle > math.pi * 2:
            angle -= math.pi * 2
        elif angle < -math.pi * 2:
            angle += math.pi * 2
    return angle

def main():
    create_urdf("urdf/bogson.urdf.xml", "bogson", "config/DH.csv")


if __name__ == '__main__':
    main()
