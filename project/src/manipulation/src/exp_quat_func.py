#!/usr/bin/env python
"""Exponential and Quaternion code for Lab 6.
Course: EE 106, Fall 2016
Author: Victor Shia, 9/24/15

This Python file is a code skeleton for Lab 6 which calculates the rigid body transform
given a rotation / translation.

When you think you have the methods implemented correctly, you can test your 
code by running "python exp_quat_func.py at the command line.

This code requires the NumPy and SciPy libraries and kin_func_skeleton which you 
should have written in lab 3.
"""

import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs

def quaternion_to_exp(rot):
    """
    Converts a quaternion vector in 3D to its corresponding omega and theta.
    This uses the quaternion -> exponential coordinate equation given in Lab 6
    
    Args:
    rot - a (4,) nd array or 4x1 array: the quaternion vector (\vec{q}, q_o)
    
    Returns:
    omega - (3,) ndarray: the rotation vector
    theta - a scalar
    """
    #YOUR CODE HERE
    q0 = rot[3];
    theta = 2* math.acos(q0);
    if theta == 0:
    	omega = np.array([0,0,0]);
    else:
    	sca = math.sin(theta/2);
    	omega = np.array([rot[0]/sca,rot[1]/sca,rot[2]/sca]);

    return (omega, theta)
    
def create_rbt(omega, theta, trans):
    """
    Creates a rigid body transform using omega, theta, and the translation component.
    g = [R,p; 0,1], where R = exp(omega * theta), p = trans
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray or 3x1 array: the translation component of the rigid body motion
    
    Returns:
    g - (4,4) ndarray : the rigid body transform
    """
    #YOUR CODE HERE
    R = kfs.rotation_3d(omega,theta);
    R_p = np.array([[R[0][0],R[0][1],R[0][2],trans[0]],
    	[R[1][0],R[1][1],R[1][2],trans[1]],
    	[R[2][0],R[2][1],R[2][2],trans[2]],
    	[0,0,0,1]]);
    return R_p
    
def compute_gab(g0a,g0b):
    """
    Creates a rigid body transform g_{ab} that converts between frame A and B
    given the coordinate frames A,B in relation to the origin
    
    Args:
    g0a - (4,4) ndarray : the rigid body transform from the origin to frame A
    g0b - (4,4) ndarray : the rigid body transform from the origin to frame B
    
    Returns:
    gab - (4,4) ndarray : the rigid body transform
    """
    #YOUR CODE HERE
    gab = (np.linalg.inv(g0a)).dot(g0b);
    return gab
    
def find_omega_theta(R):
    """
    Given a rotation matrix R, finds the omega and theta such that R = exp(omega * theta)
    
    Args:
    R - (3,3) ndarray : the rotational component of the rigid body transform
    
    Returns:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    """
    #YOUR CODE HERE
    theta = math.acos((np.trace(R)-1)/2);
    omega = (1/(2*math.sin(theta)))*(np.array([[R[2][1]-R[1][2]],[R[0][2]-R[2][0]],[R[1][0]-R[0][1]]]).T);
    omega = np.array([omega[0][0],omega[0][1],omega[0][2]]);
    return (omega, theta)
    
def find_v(omega, theta, trans):
    """
    Finds the linear velocity term of the twist (v,omega) given omega, theta and translation
    
    Args:
    omega - (3,) ndarray : the axis you want to rotate about
    theta - scalar value
    trans - (3,) ndarray of 3x1 list : the translation component of the rigid body transform
    
    Returns:
    v - (3,1) ndarray : the linear velocity term of the twist (v,omega)
    """    
    #YOUR CODE HERE
    R = kfs.rotation_3d(omega,theta);
    I = np.identity(3);
    if np.array_equal(R,I):
    	v = trans/np.norm(trans);
    else:
        
        omega1 = np.array([[omega[0]],[omega[1]],[omega[2]]]);
        A = ((I - kfs.rotation_3d(omega,theta)).dot(kfs.skew_3d(omega))+ (omega1).dot(omega1.T)*theta);
        v = (np.linalg.inv(A)).dot(trans);
        v = np.array([[v[0]],[v[1]],[v[2]]]);
    return v

    
#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------

def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')
        
def array_func_test_two_outputs(func_name, args, ret_desireds):
    ret_values = func_name(*args)
    for i in range(2):
        ret_value = ret_values[i]
        ret_desired = ret_desireds[i]
        if i == 0 and not isinstance(ret_value, np.ndarray):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
        elif i == 1 and not isinstance(ret_value, float):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a float')
        elif i == 0 and ret_value.shape != ret_desired.shape:
            print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
        elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
            print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
        else:
            print('[PASS] ' + func_name.__name__ + '() returned the argument %d value!' % i)

if __name__ == "__main__":
    print('Testing...')
    
    #Test quaternion_to_exp()
    arg1 = np.array([1.0, 2, 3, 0.1])
    func_args = (arg1,)
    ret_desired = (np.array([1.005, 2.0101, 3.0151]), 2.94125)
    array_func_test_two_outputs(quaternion_to_exp, func_args, ret_desired)
    
    #Test create_rbt()
    arg1 = np.array([1.0, 2, 3])
    arg2 = 2
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(create_rbt, func_args, ret_desired)
    
    #Test compute_gab(g0a,g0b)
    g0a = np.array(
      [[ 0.4078, -0.6562,  0.6349,  0.5   ],
       [ 0.8384,  0.5445,  0.0242, -0.5   ],
       [-0.3616,  0.5224,  0.7722,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    g0b = np.array(
      [[-0.6949,  0.7135,  0.0893,  0.5   ],
       [-0.192 , -0.3038,  0.9332, -0.5   ],
       [ 0.693 ,  0.6313,  0.3481,  1.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    func_args = (g0a, g0b)
    ret_desired = np.array([[-0.6949, -0.192 ,  0.693 ,  0.    ],
       [ 0.7135, -0.3038,  0.6313,  0.    ],
       [ 0.0893,  0.9332,  0.3481,  0.    ],
       [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(compute_gab, func_args, ret_desired)
    
    #Test find_omega_theta
    R = np.array(
      [[ 0.4078, -0.6562,  0.6349 ],
       [ 0.8384,  0.5445,  0.0242 ],
       [-0.3616,  0.5224,  0.7722 ]])
    func_args = (R,)
    ret_desired = (np.array([ 0.2673,  0.5346,  0.8018]), 1.2001156089449496)
    array_func_test_two_outputs(find_omega_theta, func_args, ret_desired)
    
    #Test find_v
    arg1 = np.array([1.0, 2, 3])
    arg2 = 1
    arg3 = np.array([0.5,-0.5,1])
    func_args = (arg1,arg2,arg3)
    ret_desired = np.array([[-0.1255],
       [ 0.0431],
       [ 0.0726]])
    array_func_test(find_v, func_args, ret_desired)
