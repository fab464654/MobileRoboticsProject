"""
   This script contains a set of auxiliary functions, used in
   'robust_wall_follower.py' main script.

 Authors: Castellini Fabio, Sandrini Michele (June/July 2022)
"""

import numpy as np
from array import array
import math


# FUNCTION DEFINITIONS: ###############################################################################


# Function for the conversion from quaternion to Euler angles XYZ
# Quaternion (w,x,y,z) --> (φ, θ, ψ) in radians if parameter unit="rad"
#                                    in radians if parameter unit="deg"
def quat2eul(w, x, y, z, unit="rad"):
    assert unit=="rad" or unit=="deg", """ERROR: quat2eul() param 'unit' must be "rad" or "deg"! """
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    if unit=="deg":
        X = np.degrees(np.arctan2(t0, t1))
    else:
        X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    if unit=="deg":
        Y = np.degrees(np.arcsin(t2))
    else:
        Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    if unit=="deg":
        Z = np.degrees(np.arctan2(t3, t4))
    else:
        Z = np.arctan2(t3, t4)
    
    #print("(",unit,") X: ", X, ", Y: ", Y , ", Z: ", Z)

    return X, Y, Z

# END quat2eul

#--------------------------------------------------------------------------------------------------------------------------#


"""
Function for converting point (distance, angle) in POLAR COORDINATES, to CARTESIAN COORDINATES (x,y)

 NOTICE:  - distances and angles can be lists, where (distances[i],angles[i]) is the i-th point
            that should be converted from polar to cartesian coords;
          - angles must be expressed in degrees!
"""
def pol2cart(distances, angles):

    if len(distances)!=len(angles):
        raise Exception("ERROR: <pol2cart>: list parameters 'distances' and 'angles' have different length! They SHOULD BE EQUAL!")
    
    length = len(distances)

    # Initializie output list for efficiency
    cartPoints = [(0.0,0.0) for _ in range(length)]

    # Compute for each point given, the corresponding i-th point in cartesian coordinates
    for i in range(length):
        x = distances[i] * np.cos(np.radians(angles[i]))
        y = distances[i] * np.sin(np.radians(angles[i]))
        cartPoints[i] = (x,y)
    
    # If the input is a single point, return a point,
    # not a list with a single element
    if length==1:
        cartPoints = cartPoints[0]
    
    return cartPoints

# END pol2cart

#--------------------------------------------------------------------------------------------------------------------------#


"""
 Function for converting point (x,y) in CARTESIAN COORDINATES,
 to POLAR COORDINATES (r, φ) (r: distance from the origin, φ: angle w.r.t. x axis (positive counter-clockwise))

 NOTICE:  - X and Y can be lists, where (X[i],Y[i]) is the i-th point
            that should be converted from cartesian to polar coords;
          - angles φ are expressed in radians!
          - 'ret_only_lengths' is a boolean variable, that if set to True
            makes the function returning only the corresponding lengths
            r of the points in polar coordinates, without returning the angle φ
"""
def cart2pol(X, Y, ret_only_lengths=True):

    if len(X)!=len(Y):
        raise Exception("ERROR: <cart2pol>: list parameters 'X' and 'Y' have different length! They SHOULD BE EQUAL!")
    
    length = len(X)
    polarPoints = []

    
    # Compute for each point given, the corresponding i-th point in polar coordinates
    if ret_only_lengths:
        # Initializie output list for efficiency
        polarPoints = [0.0 for _ in range(length)]

        for i in range(length):
            rho = np.sqrt(X[i]**2 + Y[i]**2)
            polarPoints[i] = rho
    else:
        # Initializie output list for efficiency
        polarPoints = [(0.0,0.0) for _ in range(length)]
        for i in range(length):
            rho = np.sqrt(X[i]**2 + Y[i]**2)
            phi = np.arctan2(Y[i], X[i])%360
            polarPoints[i] = (rho, phi)
    
    return polarPoints

# END cart2pol


#--------------------------------------------------------------------------------------------------------------------------#



# Sort indeces of an array such that they are compatible with LaserScan ranges convention,
# in which these indeces go from 0 to 360 (this function is specifically useful when
# the input indeces are negatives!)
# Example:
# indeces = [-5,-4,-3,-2,-1,0,1,2,3,4,5]
# Result:
# indeces_out = [0, 1, 2, 3, 4, 5,  -5,  -4,  -3,  -2,  -1]
# that corresponds to:
#               [0, 1, 2, 3, 4, 5, 355, 356, 357, 358, 359]
# You see that in the output the indeces are ordered from 0 to 360, even if they are expressed as negative indeces
def orderLaserScanSectionIndeces(indeces):
    indeces = np.array(indeces)
    neg = indeces<0
    neg_indeces = indeces[neg]
    pos_indeces = np.fmod( indeces[np.logical_not(neg)], 360)
    ordered_indeces = np.concatenate((pos_indeces,neg_indeces)) # TO ADJUST IN 'align_left' (in 'align_right' it works properly instead)

    return ordered_indeces.tolist()
    
    
    
    
    
    
    
    



def add_lidar_readings(lidar_msg):

    lidar_readings = lidar_msg.ranges.tolist()

    actual_length = len(lidar_readings)
    goal_length = 360

    #assumption that goal_length > actual_length
    values_to_add = goal_length - actual_length

    if values_to_add<=0: # the lidar readings has not lost any value, so its length is 360 (or greater, like in simulation, 361)
        return lidar_msg # do not add any value, return the original LaserScan msg already 360 element long

    lin_spaced_idx = np.linspace(1, goal_length-2, num=values_to_add)
    lin_spaced_idx = [round(index) for index in lin_spaced_idx]

    for ind in lin_spaced_idx:
        interpolation = (lidar_readings[ind-1] + lidar_readings[ind])/2
        lidar_readings = np.insert(lidar_readings, ind, interpolation)
        lin_spaced_idx = [x+1 for x in lin_spaced_idx]
    
    intensities = [0.0 for _ in range(goal_length)]

    new_lidar_msg = lidar_msg
    new_lidar_msg.ranges = array("f", lidar_readings)
    new_lidar_msg.intensities = array("f", intensities)
    new_lidar_msg.angle_increment = math.pi/180 # FUDAMENTAL
    # Original message in real robot takes less lidar readings than 360 in simulation.
    # Due to errors of lidar, the measurements are always less (we had 230-240 readings)
    # Since these readings are not 360, the angle increment is greater than 1°, because
    # all readings must cover all the range [0°,360°].
    # Instead, now, since we have modified the message such that it has exactly 360 readings,
    # then we have also to modify this parameter 'angle_increment' such that it is 1°,
    # because we have a readings for each 1°

    return new_lidar_msg

