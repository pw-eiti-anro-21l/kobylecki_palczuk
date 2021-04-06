import csv
import math
import os
import random

alpha_sum = 0

# takes filename and path of csv file and outputs DH matrix as table
def read_from_csv(filename):
    dh=[]
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            dh.append(row)
    return dh
# a, d, alpha, theta

# writes to a given location a urdf.xml file with robot data
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

            file.write('  <visual>\n')
            file.write(f'    <origin xyz="{calculate_link_origin(dh[i][0], dh[i][1], dh[i][2], dh[i][3])}" rpy="{claculate_alpha_link_origin(dh[i])}"/>\n') # {float(dh[i][0])/2} {float(dh[i][1])/2} # TRZEBA ZMIENIĆ RPY
            file.write('    <geometry>\n')
            file.write(f'      <cylinder radius="{0.5}" length="{float(dh[i][0])}" />\n') # random.random()
            file.write('    </geometry>\n')
            materials = [("green","0 1 0 0.5"), ("magenta", "1 0 1 0.5")]
            file.write(f'    <material name="{materials[i%2][0]}">\n')
            file.write(f'      <color rgba="{materials[i%2][1]}" />\n')
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
                types = {'0': "fixed", "var": "revolute"}
                file.write(f'<joint name="{joint_name}" type="{types[str(dh[i][3])]}">\n')
                file.write(f'  <origin xyz="{calculate_joint_origin(dh[i-1][0], dh[i-1][1], dh[i-1][2], dh[i-1][3])}" rpy="{claculate_alpha_joint_origin(dh[i-1]) if i>0 else "0 0 0"}"/>\n')#  {orix - float(dh[i][1])} 0 {oriz - float(dh[i][0])}" />\n') # CHYBA ŹLE? ALE NWM, dla przypadku statycznego to chyba nawet ok
                file.write(f'  <parent link="{prev_name}"/>\n')
                file.write(f'  <child link="{name}"/>\n')
                if types[dh[i][3]]=="revolute": # TO PONIŻEJ CHYBA NIE POWINNY BYĆ STAŁE WARTOŚCI?
                    file.write(f'    <axis xyz="0 1 0" />\n')
                    file.write(f'    <limit upper="0" lower="-0.5" effort="10" velocity="10" />\n')
                file.write('</joint>\n\n')

            sum_alpha(dh[i])
            # orix = float(dh[i][1])
            # oriz = 0 #float(dh[i][0])

        file.write('</robot>\n')
        print("URDF file generated successfully.")

def calculate_joint_origin(a, d, alpha, theta): # all from previous row
    orix = (float(a)*math.sin(float(alpha)))
    oriy = float(d) # cos trzeba zmienic
    oriz = (float(a)*math.cos(float(alpha)))
    return str(orix) + " " + str(oriy) + " " + str(oriz)

def calculate_link_origin(a, d, alpha, theta):
    orix = (float(a)*math.sin(float(alpha)))/2
    oriy = float(d) # cos trzeba zmienic
    oriz = (float(a)*math.cos(float(alpha)))/2
    return str(orix) + " " + str(oriy) + " " + str(oriz)

def claculate_alpha_link_origin(row): # data from previous row
    [a, d, alpha, theta] = row[:4]
    return "0 " + str(alpha) + " 0"

def claculate_alpha_joint_origin(prev_row): # data from previous row
    global alpha_sum
    [a, d, alpha, theta] = prev_row[:4]
    return "0 " + str(alpha) + " 0"

def sum_alpha(row):
    global alpha_sum
    [a, d, alpha, theta] = row[:4]
    alpha_sum += float(alpha)
    alpha_sum = normalize_angle(alpha_sum)

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
