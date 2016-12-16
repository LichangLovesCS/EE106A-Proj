#!/usr/bin/env python
"""Kinematic function skeleton code for Prelab 3.
Course: EE 106A, Fall 2015
Written by: Aaron Bestick, 9/10/14
Used by: EE106A, 9/11/15

This Python file is a code skeleton for Pre lab 3. You should fill in 
the body of the eight empty methods below so that they implement the kinematic 
functions described in the homework assignment.

When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.

This code requires the NumPy and SciPy libraries. If you don't already have 
these installed on your personal computer, you can use the lab machines or 
the Ubuntu+ROS VM on the course page to complete this portion of the homework.
"""

import numpy as np
import scipy as sp
from scipy import linalg

np.set_printoptions(precision=4,suppress=True)

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    omega_hat = [0,0,0]
    omega_hat[0] = (0, -omega[2], omega[1])
    omega_hat[1] = (omega[2], 0, -omega[0])
    omega_hat[2] = (-omega[1], omega[0], 0)
    omega_hat = np.matrix(omega_hat)
    #YOUR CODE HERE

    return omega_hat

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """
    
    #YOUR CODE HERE
    rot = [0,0]
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    rot = np.matrix( [(c_theta, -s_theta), (s_theta, c_theta)])
    return rot

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    #YOUR CODE HERE
    omega_hat = skew_3d(omega)
    rot = linalg.expm(np.multiply(theta, omega_hat))
    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    xi_hat = [0,0,0]
    xi_hat[0] = (0, -xi[2], xi[0])
    xi_hat[1] = (xi[2], 0, xi[1])
    xi_hat[2] = (0, 0, 0)
    xi_hat = np.matrix(xi_hat)

    return xi_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')
    xi_hat = [0,0,0,0]
    xi_hat[0] = (0, -xi[5], xi[4], xi[0])
    xi_hat[1] = (xi[5], 0, -xi[3], xi[1])
    xi_hat[2] = (-xi[4], xi[3], 0, xi[2])
    xi_hat[3] = (0,0,0,0)
    xi_hat = np.matrix(xi_hat)
    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    #YOUR CODE HERE
    omega = xi[2] * theta
    ct = np.cos(omega)
    st = np.sin(omega)
    rot = rotation_2d(omega)
    inverse_mat = np.matrix( [(0, -1), (1,0)] )
    vector = np.matrix( [ (xi[0]/xi[2], xi[1]/ xi[2] ) ] ).T
    last_two = np.matmul( inverse_mat, vector )
    p = np.matmul( np.matrix( [ (1 - ct, st), (-st, 1-ct) ] ) , last_two ) 
    g = np.hstack( (rot, p) )
    g = np.concatenate( (g, np.matrix( (0,0,1) ) ), axis=0 )
    return g

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')
    omega = xi[3:]
    v = xi[0:3]
    rot_3d = rotation_3d( omega, theta)

    norm = linalg.norm(omega)
    norm = norm * norm

    identity = np.eye(3)
    inverse_mat = identity - rot_3d

    first_term = np.matmul( inverse_mat, np.matmul(skew_3d(omega),  np.matrix(v).T) )

    omega_vector = np.matrix(omega).T
    second_term = np.multiply( np.matmul( omega_vector, np.matmul(omega_vector.T, np.matrix(v).T ) ) , theta)

    vector = (first_term+second_term)/norm

    g = np.hstack( (rot_3d, vector) )
    g = np.concatenate( (g, np.matrix( (0,0,0,1) ) ), axis=0 )
    return g

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint function which computes the coordinate transformation between the base and tool frames for the Baxter arm pictured below. Your function should take an array of 7 joint angles as its only argument, and return the 4x4 homogeneous 
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')
    if not xi.shape[1] == theta.shape[0]:
        raise TypeError('xi columns must match theta rows')

    #YOUR CODE HERE
    n = xi.shape[1]
    g = np.eye(4)
    for i in range(n):

        col = xi[:,i]
        col = col.reshape((6,))
        g = np.matmul(g, homog_3d( col, theta[i]) )
    return g

def frame_transform(joint_angles, q=None, omega=None):
    matrixify = lambda a: map(lambda x: np.matrix(x), a)
    if q is None:
        q1 = (.0635,.2598,.1188)
        q2 = (.1106,.3116,.3885)
        q3 = (.1827,.3838,.3881)
        q4 = (.3682,.5684,.3181)
        q5 = (.4417,.6420,.3177)
        q6 = (.6332,.8337,.3067)
        q7 = (.7152,.9158,.3063)
        q_hand = (.7957,.9965,.3058)
        q = [q1,q2,q3,q4,q5,q6,q7]
        q = matrixify(q)
        # q = map(lambda x: np.matrix(x), q)

    if omega is None:
        w1 = (-.0059,.0113,.9999)
        w2 = (-.7077,.7065,-.0122)
        w3 = (.7065,.7077,-.0038)
        w4 = (-.7077,.7065,-.0122)
        w5 = (.7065,.7077,-.0038)
        w6 = (-.7077,.7065,-.0122)
        w7 = (.7065,.7077,-.0038)
        omega = [w1,w2,w3,w4,w5,w6,w7]
        omega = matrixify(omega)
        # omega = map(lambda x: np.matrix(x), omega)

    twists = np.matrix([[],[],[],[],[],[]])
    for index in range(len(q)):
        velocity = np.matrix(np.cross( -omega[index], q[index])).T
        twist = np.matrix(np.concatenate( (velocity, omega[index].T) ))



        twists = np.hstack( (twists, twist) )


    print "TWISTS\n", twists
    transform = prod_exp(np.array(twists), np.array(joint_angles))

    # print "TRANSFORM", transform
    rot = np.matrix(np.eye(3))
    pos = np.matrix([0.7957,0.9965,0.3058]).T
    r4 = np.matrix([0,0,0,1])

    z_config = np.hstack( (rot, pos) )
    z_config = np.concatenate( (z_config, r4), axis=0)

    return np.matmul(transform, z_config)


    # print z_config



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

if __name__ == "__main__":
    print('Testing...')

    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_2d()
    arg1 = 2.658
    func_args = (arg1,)
    ret_desired = np.array([[-0.8853, -0.465 ],
                            [ 0.465 , -0.8853]])
    array_func_test(rotation_2d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_2d()
    arg1 = np.array([2.0, 1, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3.,  0.,  1.],
                            [ 0.,  0.,  0.]])
    array_func_test(hat_2d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_2d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.3924, -0.9198,  0.1491],
                            [ 0.9198, -0.3924,  1.2348],
                            [ 0.    ,  0.    ,  1.    ]])
    array_func_test(homog_2d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    joint_angles = np.matrix([0,0,1,0,0,0,0])
    print "Joint angles", joint_angles.shape
    joint_angles = joint_angles.T
    print "Joint angles", joint_angles.shape
    print frame_transform(joint_angles)
    print('Done!')
