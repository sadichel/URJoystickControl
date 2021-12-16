
import numpy as np
import math

# Support functions

def getT_fromPose(x, y, z, rx, ry, rz):

    Rx = np.mat([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.mat([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.mat([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    t = [x, y, z]
    unit_v = [0, 0, 0, 1]
    R = Rz * Ry * Rx
    T = np.row_stack((np.column_stack((R, t)), unit_v))
    return T


def getPose_fromT(T):
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    rx = math.atan2(T[2, 1], T[2, 2])
    ry = math.asin(-T[2, 0])
    rz = math.atan2(T[1, 0], T[0, 0])
    return np.array([x, y, z, rx, ry, rz])


def rpy2rv(roll, pitch, yaw):

    alpha, beta, gamma = yaw, pitch, roll

    ca, cb, cg = math.cos(alpha), math.cos(beta), math.cos(gamma)
    sa, sb, sg = math.sin(alpha), math.sin(beta), math.sin(gamma)

    r11, r12, r13 = ca*cb, ca*sb*sg-sa*cg, ca*sb*cg+sa*sg
    r21, r22, r23 = sa*cb, sa*sb*sg+ca*cg, sa*sb*cg-ca*sg
    r31, r32, r33 = -sb, cb*sg, cb*cg

    theta = math.acos((r11+r22+r33-1)/2)
    sth = math.sin(theta)
    kx, ky, kz = (r32-r23)/(2*sth), (r13-r31)/(2*sth), (r21-r12)/(2*sth)

    return [theta*kx, theta*ky, theta*kz]

def rv2rpy(rx, ry, rz):
    theta = math.sqrt(rx*rx+ry*ry+rz*rz)
    kx, ky, kz = rx/theta, ry/theta, rz/theta
    cth, sth, vth = math.cos(theta), math.sin(theta), 1-math.cos(theta)

    r11, r12, r13 = kx*kx*vth+cth, kx*ky*vth-kz*sth, kx*kz*vth+ky*sth
    r21, r22, r23 = kx*ky*vth+kz*sth, ky*ky*vth+cth, ky*kz*vth-kx*sth
    r31, r32, r33 = kx*kz*vth-ky*sth, ky*kz*vth+kx*sth, kz*kz*vth+cth

    beta = math.atan2(-r31,math.sqrt(r11*r11+r21*r21))

    if  beta > math.radians(89.99):
        beta = math.radians(89.99)
        alpha = 0
        gamma = math.atan2(r12,r22)
    elif beta < -math.radians(89.99):
        beta = -math.radians(89.99)
        alpha = 0
        gamma = -math.atan2(r12, r22)
    else:
        cb = math.cos(beta)
        alpha = math.atan2(r21/cb,r11/cb)
        gamma = math.atan2(r32/cb,r33/cb)

    return [gamma, beta, alpha]

