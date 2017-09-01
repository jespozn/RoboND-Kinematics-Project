#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

# Global variables
FK_transform = None

def calculate_FK():
    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    # Modified DH params
    s = {alpha0:     0, a0:      0, d1:  0.75,
         alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
         alpha2:     0, a2:   1.25, d3:     0,
         alpha3: -pi/2, a3: -0.054, d4:   1.5,
         alpha4:  pi/2, a4:      0, d5:     0,
         alpha5: -pi/2, a5:      0, d6:     0,
         alpha6:     0, a6:      0, d7: 0.303, q7: 0}

    # Create individual transformation matrices
    T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                   [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [                  0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)
    T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
                   [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [                  0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)
    T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
                   [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                  0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)
    T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                   [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                   [                  0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)
    T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                   [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                   [                  0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)
    T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                   [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                   [                  0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)
    T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
                   [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                   [                  0,                   0,            0,               1]])
    T6_G = T6_G.subs(s)

    # Composition of Homogeneous Transformation matrix
    T0_2 = T0_1 * T1_2  # base_link to link_2
    T0_3 = T0_2 * T2_3  # base_link to link_3
    T0_4 = T0_3 * T3_4  # base_link to link_4
    T0_5 = T0_4 * T4_5  # base_link to link_5
    T0_6 = T0_5 * T5_6  # base_link to link_6
    T0_G = T0_6 * T6_G  # base_link to gripper

    # Gripper orientation correction
    R_z = Matrix([[    cos(np.pi),  -sin(np.pi),              0, 0],
                  [    sin(np.pi),   cos(np.pi),              0, 0],
                  [             0,            0,              1, 0],
                  [             0,            0,              0, 1]])
    R_y = Matrix([[ cos(-np.pi/2),            0,  sin(-np.pi/2), 0],
                  [             0,            1,              0, 0],
                  [-sin(-np.pi/2),            0,  cos(-np.pi/2), 0],
                  [             0,            0,              0, 1]])
    #R_corr = simplify(R_z * R_y)
    R_corr = R_z * R_y

    # Total homogeneous transform
    T_total = T0_G * R_corr
    #R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
    #R0_6 = R0_3*T3_4[0:3,0:3]*T4_5[0:3,0:3]*T5_6[0:3,0:3]

    #print('T_total = ', T_total.evalf())

    return T_total

def handle_calculate_IK(req):
    global FK_transform
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            
            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                                 req.poses[x].orientation.z, req.poses[x].orientation.w])
            #print('px = %f py = %f pz = %f roll = %f pitch = %f yaw = %f ' % (px, py, pz, roll, pitch, yaw))

            # Calculate joint angles using Geometric IK method
            # Calculate nx, ny and nz
            R_z = Matrix([[   cos(yaw), -sin(yaw),          0],
                          [   sin(yaw),  cos(yaw),          0],
                          [          0,         0,          1]])

            R_y = Matrix([[ cos(pitch),         0, sin(pitch)],
                          [          0,         1,          0],
                          [-sin(pitch),         0, cos(pitch)]])

            R_x = Matrix([[          1,         0,          0],
                          [          0, cos(roll), -sin(roll)],
                          [          0, sin(roll),  cos(roll)]])

            Rrpy = R_z * R_y * R_x

            nx = Rrpy[0,0]
            ny = Rrpy[1,0]
            nz = Rrpy[2,0]

            # Wrist position
            wx = px - 0.303 * nx
            wy = py - 0.303 * ny
            wz = pz - 0.303 * nz

            theta1 = atan2(wy, wx)

            # Triangle for theta2 and theta3
            r = sqrt(wx*wx + wy*wy) - 0.35		# 
            h = wz - 0.75
            A = sqrt(1.5*1.5 + 0.054*0.054)
            B = sqrt(r*r + h*h)
            C = 1.25
            angle_a = acos((C*C + B*B - A*A)/(2*C*B))
            angle_b = acos((A*A + C*C - B*B)/(2*A*C))

            theta2 = np.pi/2 - angle_a - atan2(h, r)
            theta3 = np.pi/2 - angle_b - atan2(0.054, 1.5)

            # Gripper orientation
            qq1 = theta1
            qq2 = theta2
            qq3 = theta3
            inv_R0_3 = Matrix([
                              [sin(qq2 + qq3) * cos(qq1), sin(qq1) * sin(qq2 + qq3),  cos(qq2 + qq3)],
                              [cos(qq1) * cos(qq2 + qq3), sin(qq1) * cos(qq2 + qq3), -sin(qq2 + qq3)],
                              [                -sin(qq1),                  cos(qq1),               0]])

            R3_6 = inv_R0_3 * Rrpy
            #print('R3_6', R3_6)

            theta4 = atan2(R3_6[2, 0], -R3_6[0, 0])
            # Shortest rotation
            if x == 0:
                theta4_prev = theta4
            if (theta4 - theta4_prev) > np.pi:
                theta4 = theta4 - 2 * np.pi
            elif (theta4 - theta4_prev) < -np.pi:
                theta4 = theta4 + 2 * np.pi
            # Joint limits
            if theta4 > 6.109:
                theta4 = theta4 - 2 * np.pi
            elif theta4 < -6.109:
                theta4 = theta4 + 2 * np.pi
            theta4_prev = theta4
            
            sin_beta = sqrt(R3_6[0, 0] * R3_6[0, 0] + R3_6[2, 0] * R3_6[2, 0])
            theta5 = atan2(sin_beta, R3_6[1, 0])
            
            theta6 = atan2(R3_6[1, 1], R3_6[1, 2] / sin_beta)
            	# Shortest rotation
            if x == 0:
                theta6_prev = theta6
            if (theta6 - theta6_prev) > np.pi:
                theta6 = theta6 - 2 * np.pi
            elif (theta6 - theta6_prev) < -np.pi:
                theta6 = theta6 + 2 * np.pi
            # Joint limits
            if theta6 > 6.109:
                theta6 = theta6 - 2 * np.pi
            elif theta6 < -6.109:
                theta6 = theta6 + 2 * np.pi
            theta6_prev = theta6

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1.evalf(), theta2.evalf(), theta3.evalf(),
                                                theta4.evalf(), theta5.evalf(), theta6.evalf()]
            joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            #print('positions: ', joint_trajectory_point.positions)

            ######################################################################################
            ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
            ## as the input and output the position of your end effector as your_ee = [x,y,z]

            T_total = FK_transform.evalf(subs={q1: theta1, q2: theta2, q3: theta3,
                                                q4: theta4, q5: theta5, q6: theta6})
            #print('T_total = ', T_total)
            your_ee = [T_total[0, 3], T_total[1, 3], T_total[2, 3]]
            
            # Errors
            ## End effector position
            ee_x_e = T_total[0, 3] - px
            ee_y_e = T_total[1, 3] - py
            ee_z_e = T_total[2, 3] - pz
            ee_offset = sqrt(ee_x_e ** 2 + ee_y_e ** 2 + ee_z_e ** 2)
            print('EEx error = %.8f' % ee_x_e)
            print('EEy error = %.8f' % ee_y_e)
            print('EEz error = %.8f' % ee_z_e)
            print('offset = %.8f' % ee_offset)
            # End effector rotation
            R0_6 = T_total[0:3,0:3]
            R_e = R0_6 - Rrpy
            print('Rotational matrix error = %.8f' % R_e)
        #print('trajectory list: ', joint_trajectory_list)
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    global FK_transform
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    # Calculate transform matrix
    print('Calculating...')
    FK_transform = calculate_FK()
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
