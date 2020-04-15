import numpy as np
import matplotlib.pyplot as plt
from vpython import*
import random
import math

scene = canvas()

class Robot:
    def __init__(self, p2, l1, p3, l2, p4, l3, p5):
        self.p2 = p2
        self.l1 = l1
        self.p3 = p3
        self.l2 = l2
        self.p4 = p4
        self.l3 = l3
        self.p5 = p5
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0

def rads(deg):
    return np.radians(deg)

def cos(deg):
    return np.cos(deg)

def sin(deg):
    return np.sin(deg)

def get_transformation(angle, dx, dy):
    return np.array([[cos(angle), -1*sin(angle), dx], 
                    [sin(angle), cos(angle), dy],
                    [0, 0, 1]])

def rotate_frame(robot, deg, deg2, deg3):
    # angle = rads(-deg)
    print('deg: %d, deg1: %d, deg2: %d' % (deg, deg2, deg3))
    deg = -1*deg
    axis_y = robot.p2.pos.y
    axis_x = robot.p2.pos.x
    start = robot.theta1
    print("Theta 1: %d" % start)
    if deg < start:
        step = -1
    else:
        step = 1
    angle1 = rads(deg-1)

    for i in range(start, deg, step):
        rate(100)
        robot.l1.rotate(angle=rads(step),axis=vector(0,0,1), origin=vector(axis_x, axis_y,0))
        robot.theta1 = deg
        angle1 = rads(i)
        T0_1 = get_transformation(angle1, 0, robot.p2.pos.y)
        # angle2 = rads(0)
        angle2 = rads(robot.theta2)
        T1_2 = get_transformation(angle2, 0, robot.l1.height+(2*robot.p3.radius))
        # angle3 = rads(0)
        angle3 = rads(robot.theta3)
        T2_3 = get_transformation(angle3, 0, robot.l2.height+(2*robot.p4.radius)) #Really doesn't make a difference p3 or p4 or p5
        # angle4 = rads(0)
        T3_4 = get_transformation(rads(0), 0, robot.l3.height+(2*robot.p5.radius))
        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3)
        T0_4 = np.dot(T0_3, T3_4)

        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        p4_pos = np.dot(T0_3, np.array([0,0,1]))
        p5_pos = np.dot(T0_4, np.array([0,0,1]))

        # print('deg: %f -> end effector x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
        robot.p3.pos.x = p3_pos[0]
        robot.p3.pos.y = p3_pos[1]

        Tp3_l2 = get_transformation(rads(i), 0,(robot.l2.height/2)+(robot.p4.radius))
        T0_l2 = np.dot(T0_2, Tp3_l2)
        l2_pos = np.dot(T0_l2, np.array([0,0,1]))
        robot.l2.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l2.pos.x = l2_pos[0]
        robot.l2.pos.y = l2_pos[1]

        robot.p4.pos.x = p4_pos[0]
        robot.p4.pos.y = p4_pos[1]

        Tp4_l3 = get_transformation(0, 0,(robot.l3.height/2)+(robot.p5.radius))
        T0_l3 = np.dot(T0_3, Tp4_l3)
        l3_pos = np.dot(T0_l3, np.array([0,0,1]))
        robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l3.pos.x = l3_pos[0]
        robot.l3.pos.y = l3_pos[1]

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]
        print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
    print("### SECOND LOOP ###")
    deg = -1*deg2
    start = robot.theta2
    if deg < start:
        step = -1
    else:
        step = 1
    angle2 = rads(deg-1)
    axis_y = robot.p3.pos.y
    axis_x = robot.p3.pos.x
    # print("start: %d, deg: %d" % (start, deg))
    for i in range(start, deg, step):
        rate(100)
        robot.l2.rotate(angle=rads(step),axis=vector(0,0,1), origin=vector(axis_x, axis_y,0))
        robot.theta2 = deg
        # angle1 = rads(0)
        T0_1 = get_transformation(angle1, 0, robot.p2.pos.y)
        angle2 = rads(i)
        T1_2 = get_transformation(angle2, 0, robot.l1.height+(2*robot.p3.radius))
        # angle3 = rads(0)
        angle3 = rads(robot.theta3)
        T2_3 = get_transformation(angle3, 0, robot.l2.height+(2*robot.p4.radius)) #Really doesn't make a difference p3 or p4 or p5
        # angle4 = rads(0)
        T3_4 = get_transformation(rads(0), 0, robot.l3.height+(2*robot.p5.radius))
        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3)
        T0_4 = np.dot(T0_3, T3_4)

        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        p4_pos = np.dot(T0_3, np.array([0,0,1]))
        p5_pos = np.dot(T0_4, np.array([0,0,1]))

        robot.p4.pos.x = p4_pos[0]
        robot.p4.pos.y = p4_pos[1]

              ######## Animating link 3 using transformation matrix
        Tp4_l3 = get_transformation(0, 0,(robot.l3.height/2)+(robot.p5.radius))
        T0_l3 = np.dot(T0_3, Tp4_l3)
        l3_pos = np.dot(T0_l3, np.array([0,0,1]))
        robot.l3.rotate(angle=rads(step), axis=vector(0,0,1))
        robot.l3.pos.x = l3_pos[0]
        robot.l3.pos.y = l3_pos[1]

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]

        print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))
    print("#### THIRD LOOP ###")
    deg = -1*deg3
    start = robot.theta3
    if deg < start:
        step = -1
    else:
        step = 1
    angle3 = rads(deg-1)
    axis_y = robot.p4.pos.y
    axis_x = robot.p4.pos.x
    # print("start: %d, deg: %d" % (start, deg))
    for i in range(start, deg, step):
        rate(100)
        robot.l3.rotate(angle=rads(step),axis=vector(0,0,1), origin=vector(axis_x, axis_y,0))
        # print(i)
        robot.theta3 = deg
        # angle1 = rads(0)
        T0_1 = get_transformation(angle1, 0, robot.p2.pos.y)
        # angle2 = rads(0)
        T1_2 = get_transformation(angle2, 0, robot.l1.height+(2*robot.p3.radius))
        angle3 = rads(i)
        T2_3 = get_transformation(angle3, 0, robot.l2.height+(2*robot.p4.radius)) #Really doesn't make a difference p3 or p4 or p5
        # angle4 = rads(0)
        T3_4 = get_transformation(rads(0), 0, robot.l3.height+(2*robot.p5.radius))
        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3)
        T0_4 = np.dot(T0_3, T3_4)

        p2_pos = np.dot(T0_1,np.array([0,0,1]))
        p3_pos = np.dot(T0_2, np.array([0,0,1]))
        p4_pos = np.dot(T0_3, np.array([0,0,1]))
        p5_pos = np.dot(T0_4, np.array([0,0,1]))

        robot.p5.pos.x = p5_pos[0]
        robot.p5.pos.y = p5_pos[1]

        print('deg: %f -> p5 x: %f, y: %f' % (i, p5_pos[0], p5_pos[1]))

def main():
    print("Hello world!")
    joint_radius = 1
    platform = box(pos=vector(0, 0, 0), size=vector(
        100,3, 100), color=color.orange)

    base =  box(pos=vector(0, 6.5, 0), size=vector(
        10,10, 10), color=color.orange) #6.5 = 3/2 + 10/2

    p2 = cylinder(pos=vector(
            0, 12.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2
            
    l1 = box(pos=vector(0, 18.5, 0), size=vector(
        3,10, 3), color=color.purple) #18 = 11.5+2+10/2
 
    p3 = cylinder(pos=vector(
            0, 24.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2


    # print('L1 length: %f' % l1.height)
    # print("p2 y: %f" % p2.pos.y)
    
    l2 = box(pos=vector(0, 30.5, 0), size=vector(3,10, 3), color=color.purple) 

    p4 = cylinder(pos=vector(
            0, 36.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    l3 = box(pos=vector(0, 42.5, 0), size=vector(3,10, 3), color=color.purple)
   
    p5 = cylinder(pos=vector(
            0, 48.5, -1.5), axis=vector(0, 0, 3), radius=joint_radius) #diameter 2

    print('p5_y: %f' % p5.pos.y)
    print('p5_x: %f' % p5.pos.x)
    # origin = sphere(pos=vector(0,0,0), radius=5, color = color.red)
  
    myBot = Robot(p2, l1, p3, l2, p4, l3, p5)
    # rotate_frame(myBot, -100, 100, -45)

    
    # rotate_frame(Robot, [90])
    print("Ready to boogy master chief!")
    # print("Robot awaiting your command...")
    # cmmd = input()
    cmmd_array = list()
    while True:
        # rotate_frame(myBot, int(cmmd))
        print("Robot awaiting your command...")
        for i in range(3):
            cmmd = input("theta"+str(i+1)+":")
            angle = int(cmmd)
            if math.isnan(angle):
                break
            cmmd_array.append(angle)
        # cmmd = input()
        print("cmmd1: %d, cmmd2: %d, cmmd3: %d" % (cmmd_array[0], cmmd_array[1], cmmd_array[2]))
        rotate_frame(myBot, cmmd_array[0], cmmd_array[1], cmmd_array[2])
        cmmd_array.clear()
        # print(cmmd)

# print("Shutting down...")


if __name__ == "__main__":
    main()