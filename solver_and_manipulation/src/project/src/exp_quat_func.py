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


# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

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


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
    >>> numpy.allclose(quaternion_from_matrix(R, isprecise=False),
    ...                quaternion_from_matrix(R, isprecise=True))
    True

    """
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 1, 2, 3
            if M[1, 1] > M[0, 0]:
                i, j, k = 2, 3, 1
            if M[2, 2] > M[i, i]:
                i, j, k = 3, 1, 2
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q

    
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
    print('Testing in eqf...')
    
    #Test quaternion_to_exp()
    # arg1 = np.array([1.0, 2, 3, 0.1])
    # func_args = (arg1,)
    # ret_desired = (np.array([1.005, 2.0101, 3.0151]), 2.94125)
    # # array_func_test_two_outputs(quaternion_to_exp, func_args, ret_desired)
    # print('11111111111111111111111111111111')
    # aa =quaternion_matrix(arg1)
    # bb = quaternion_from_matrix(aa)
    # scale = ret_desired[0][0]/bb[0]
    # scale1 = ret_desired[0][1]/bb[1]
    # scale2 = ret_desired[0][2]/bb[2]
    # scale3 = ret_desired[1]/bb[3]
    # print(scale)
    # print(scale1)
    # print(scale2)
    # print(scale3)

    # print(aa)
    
    #Test create_rbt()
    # arg1 = np.array([1.0, 2, 3])
    # arg2 = 2
    # arg3 = np.array([0.5,-0.5,1])
    # func_args = (arg1,arg2,arg3)
    # ret_desired = np.array(
    #   [[ 0.4078, -0.6562,  0.6349,  0.5   ],
    #    [ 0.8384,  0.5445,  0.0242, -0.5   ],
    #    [-0.3616,  0.5224,  0.7722,  1.    ],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]])
    # array_func_test(create_rbt, func_args, ret_desired)
    
    # #Test compute_gab(g0a,g0b)
    # g0a = np.array(
    #   [[ 0.4078, -0.6562,  0.6349,  0.5   ],
    #    [ 0.8384,  0.5445,  0.0242, -0.5   ],
    #    [-0.3616,  0.5224,  0.7722,  1.    ],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]])
    # g0b = np.array(
    #   [[-0.6949,  0.7135,  0.0893,  0.5   ],
    #    [-0.192 , -0.3038,  0.9332, -0.5   ],
    #    [ 0.693 ,  0.6313,  0.3481,  1.    ],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]])
    # func_args = (g0a, g0b)
    # ret_desired = np.array([[-0.6949, -0.192 ,  0.693 ,  0.    ],
    #    [ 0.7135, -0.3038,  0.6313,  0.    ],
    #    [ 0.0893,  0.9332,  0.3481,  0.    ],
    #    [ 0.    ,  0.    ,  0.    ,  1.    ]])
    # array_func_test(compute_gab, func_args, ret_desired)
    
    # #Test find_omega_theta
    # R = np.array(
    #   [[ 0.4078, -0.6562,  0.6349 ],
    #    [ 0.8384,  0.5445,  0.0242 ],
    #    [-0.3616,  0.5224,  0.7722 ]])
    # func_args = (R,)
    # ret_desired = (np.array([ 0.2673,  0.5346,  0.8018]), 1.2001156089449496)
    # array_func_test_two_outputs(find_omega_theta, func_args, ret_desired)
    
    # #Test find_v
    # arg1 = np.array([1.0, 2, 3])
    # arg2 = 1
    # arg3 = np.array([0.5,-0.5,1])
    # func_args = (arg1,arg2,arg3)
    # ret_desired = np.array([[-0.1255],
    #    [ 0.0431],
    #    [ 0.0726]])
    # array_func_test(find_v, func_args, ret_desired)
