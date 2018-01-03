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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
		### Your FK code here
		# symbols for the DH parameter joint variables
		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta values
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

		# KUKA KR210 DH parameters
		DH = {alpha0:       0, a0:      0, d1:  0.75, q1:           q1,
		      alpha1: -pi / 2, a1:   0.35, d2:     0, q2:  q2 - pi / 2,
		      alpha2:       0, a2:   1.25, d3:     0, q3:           q3,
		      alpha3: -pi / 2, a3: -0.054, d4:  1.50, q4:           q4,
		      alpha4:  pi / 2, a4:      0, d5:     0, q5:           q5,
		      alpha5: -pi / 2, a5:      0, d6:     0, q6:           q6,
		      alpha6:       0, a6:      0, d7: 0.303, q7:            0}

		# transformation matrix function; will pass DH parameters to function for each transformation
		def TF_Matrix(alpha, a, d, q):
		    transform_matrix = Matrix([[             cos(q),             -sin(q),       0,                   a],
		                               [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
		                               [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d],
		                               [                  0,                   0,           0,               1]])
		    return transform_matrix

		# creating the individual transformation matrices
		TF0_1 =  TF_Matrix(alpha0, a0, d1, q1).subs(DH)
		TF1_2 =  TF_Matrix(alpha1, a1, d2, q2).subs(DH)
		TF2_3 =  TF_Matrix(alpha2, a2, d3, q3).subs(DH)
		TF3_4 =  TF_Matrix(alpha3, a3, d4, q4).subs(DH)
		TF4_5 =  TF_Matrix(alpha4, a4, d5, q5).subs(DH)
		TF5_6 =  TF_Matrix(alpha5, a5, d6, q6).subs(DH)
		TF6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH)

		# computing the transformation from 0(origin) to the end effector
		TF0_EE = TF0_1 * TF1_2 * TF2_3 * TF3_4 * TF4_5 * TF5_6 * TF6_EE

		# initialize the symbols for the end effector rotation
		r, p, y = symbols('r p y')
	
		# generate the rotation matrices for roll, pitch, yaw
		# roll
		R_x = Matrix([[1,      0,       0],
			          [0, cos(r), -sin(r)],
			          [0, sin(r), cos(r)]])
		# pitch
		R_y = Matrix([[ cos(p), 0, sin(p)],
			          [      0, 1,      0],
			          [-sin(p), 0, cos(p)]])
		# yaw
		R_z = Matrix([[cos(y), -sin(y), 0],
			          [sin(y),  cos(y), 0],
			          [     0,       0, 1]])
	
		# End Effector rotation matrix
		R_EE = R_z * R_y * R_x

		# calculate the correction for rotation error
		R_Error = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))

		# compute the adjusted/corrected end effector matrix 
		R_EE = R_EE * R_Error
		

    # Initialize service response
    joint_trajectory_list = []
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

	# substituting the roll, pitch, yaw values into the EE rotation matrix
	R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

	# initializing the end effector position matrix with the current data 
	EE = Matrix([[px],
		 [py],
		 [pz]])

	# WC (wrist center)
	WC = EE - 0.303 * R_EE[:,2]

	# Calculate the joint angles using the Inverse kinematics geometry method
	# calculating the arc tangent of the wrist center (x,y) coordinates to get
	# theta1 value at the base of the manipulator
	theta1 = atan2(WC[1], WC[0])

	# calculate the theta2 and theta3 values
	# make a trianlge using joint 1 to joint 3 to WC
	# see hand drawing for location of the sides
	side_a = 1.50
	side_b = 1.25

	# calculating the inputs for side c
	WC_x_2 = WC[0] ** 2
	WC_y_2 = WC[1] ** 2
	WC_z = WC[2]
	resultant_xy = sqrt(WC_x_2 + WC_y_2)

	# calculating side c of the triangle
	side_c = sqrt((resultant_xy - 0.35)**2 + (WC[2] - 0.75)**2)

	# calculate the angles of the triangle using law of cosines
	angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2 * side_b * side_c))
	angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2 * side_a * side_c))
	angle_c = acos((side_b**2 + side_a**2 - side_c**2)/(2 * side_b * side_a))

	# calculate theta 2 and 3 using triangle between 2, 3, 5
	theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2)-0.35)
	theta3 = pi / 2 - (angle_c + 0.036)

	# extract the rotation matrix from the transformation matrix
	R0_3 = TF0_1[0:3, 0:3] * TF1_2[0:3, 0:3] * TF2_3[0:3, 0:3]
	# substitute the theta 1 through 3 into the the rotation matrix
	R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

	# multiple the end effector rotation matrix by the inverse of the 0 to 3 rotation matrix
	R3_6 = R0_3.inv("LU") * R_EE

	# calculate theta4 through theta 6 using Euler angle from rotation matrix
	theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]**2), R3_6[1,2])
	theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	
    # Populate response for the IK request
    # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
