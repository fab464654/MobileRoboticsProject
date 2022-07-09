"""
This Python script contains the definition of the function ransac

Author: Fabio Castellini, Michele Sandrini (june/july 2022)
"""


import numpy as np
from random import randint
from .auxiliary_functions import *
from sklearn.linear_model import LinearRegression


# FUNCTION DEFINITIONS: ######################################################################################


"""
    inliers, outliers, m_best, q_best, [cart2pol(X[k], y[k]) for k in range(len(X))]
     = ransac(threshold, iterations, distancesForRansac, anglesForRansac, add_noise=False, sigma=0.001)

 INPUT:
 - threshold, iterations:                double,
                                        ransac algorithm parameters, specifying relatively the 'threshold'
                                        distance under which a point gives its "consensus" to a certain
                                        line and the number of 'iterations' that the algorithm is
                                        repeated
 - distancesForRansac, anglesForRansac:   lists of doubles
                                        together, these two lists specifies the 2D points used as
                                        input for RANSAC algorithm, through which the best regressor line
                                        should be found.
                                        In particular they are expressed in POLAR coordinates, where:
                                         > 'distancesForRansac[i]' expresses the distance from the origin
                                                                of the i-th point
                                         > 'anglesForRansac[i]' expresses the angle with repsect the x-axis
                                                                of the same i-th point with distance
                                                                'distancesForRansac[i]'
 - add_noise, sigma:                      boolean, double
                                        if 'add_noise' boolean variable is set to 'True', thean to each distance
                                        in 'distancesForRansac' is added a white noise component, whose mean is
                                        0 and standard deviation 'sigma'

 OUTPUT:
 - inliers:                  list of doubles
                            points in cartesian coords classified as inliers by RANSAC algorithm, onto which the line is fitted
 - outliers:                 list of double that are not inliers
                            points in cartesian coords 
 - m_best:                   float
                            angular coefficient of the best regressor line 'inliers' found by RANSAC
 - q_best:                   float
                            translation coeffiecient of the best regressor line 'inliers' found by RANSAC
                            (REMINDER: this is 'q' of classical line equation    y = m*x + q )
 - lidarPointsUsed:          list of doubles,
                            lidar measurements (distances, in meters), which coincide with 'distancesForRansac'
                            if no noise is added, or 'distancesForRansac'+white noise, if noise is added
"""
def ransac(threshold, iterations, distancesForRansac, anglesForRansac, add_noise=False, sigma=0.001):

    #Retrieve cartesian coordinates from polar lidar scansions (angle is given by the index + the "front_angle_half") 
    #lidar_cart = [pol2cart(distance, anglesForRansac[k]) for k, distance in enumerate(distancesForRansac)] # OLD: remove it
    lidar_cart = pol2cart(distancesForRansac, anglesForRansac)

    # Separate x and y coordinates of 'lidar_cart'
    lidar_cart_x = [coord[0] for coord in lidar_cart] # x coordinates
    lidar_cart_y = [coord[1] for coord in lidar_cart] # y coordinates

    if add_noise:
        # Add white noise (Gaussian) to each point x and y coordinate

        # Mean of the white noise ( standard deviation "sigma" is given as parameter)
        mu = 0

        # Add elementwise, the white noise
        lidar_cart_noise_x = [ (x+np.random.normal(mu,sigma)) for x in lidar_cart_x] 
        lidar_cart_noise_y = [ (y+np.random.normal(mu,sigma)) for y in lidar_cart_y]

        # Use the noisy points as RANSAC input points (coordinates)
        X = lidar_cart_noise_x
        y = lidar_cart_noise_y

    else:
        # RANSAC input points (coordinates)
        X = lidar_cart_x
        y = lidar_cart_y

    # Memorize the consensus and the "(m, q)" parameters associated to each model
    consensus = np.zeros([iterations])
    parameters = np.zeros([iterations,2]) #(m,q)

    n_samples = len(X) # number of points

    # Repeat the procedure 'iterations' number of time
    # Each iteration, a random couple of points is taken, and the line passing through them is computed
    # Then, each point gives its consensus to this line, if the (orthogonal distance) is closer than the threshold
    # At then end, the model with the hisghest consensus, define the set of inliers points
    for i in range(iterations):
        
        # Get a random couple of points
        index1 = randint(0, n_samples-1)
        point1 = X[index1], y[index1] # Retrieve a random tuple (x,y)
        
        index2 = randint(0, n_samples-1)
        while (index2 == index1):
            index2 = randint(0, n_samples-1)  # To have a different point than point1

        point2 = X[index2], y[index2]         # Prelevo un'altra coppia random (x,y)
        
        # Create the estimated model that represents a line (only having 2 params: angular "m" and translation "q" coefficients )
        if point2[0]-point1[0] > 0.001:  #to "handle" division by zero
            m_estimate = (point2[1]-point1[1]) / (point2[0]-point1[0])
        else:
            m_estimate = 10 # to saturate angular coefficient (remember that lines parallel to y axis has infinite ang. coeff.)
        q_estimate = -m_estimate*point1[0] + point1[1]  #y = mx + q --> q = -mx + y

        # Save/store parameter of the line of this iteration
        parameters[i,:] = m_estimate, q_estimate

        # Compute consensus for each model
        for k in range(n_samples):
            # If the distance between point and line is lower than threshold consesus is increased
            y_coord = y[k]
            x_coord = X[k] 

            m_estimate = parameters[i,0]
            q_estimate = parameters[i,1]

            if abs( y_coord - (m_estimate*x_coord + q_estimate) ) < threshold:
                consensus[i] += 1
    # FOR END

    # Extract the highest consensus model
    for ind in range(len(consensus)):
        if(max(consensus) == consensus[ind]):
            m_best, q_best = parameters[ind,0], parameters[ind,1]

    # Retrieve inlier and outlier points of this highest consensus model
    inliers  = np.empty([0,2])  # initialization
    outliers = np.empty([0,2])  # initialization

    for n in range(n_samples):
        y_coord = y[n] 
        x_coord = X[n] 

        if abs( (y_coord - (m_best*x_coord + q_best)) ) < threshold: # inliers point
            inliers = np.vstack([inliers, [x_coord, y_coord]])
        else:
            outliers = np.vstack([outliers, [x_coord, y_coord]])     # outlier point
    
    # Now use these inliers for computing the LEAST SQUARE ERROR line solution (REGRESSION)
    x_inliers = inliers[:,0].reshape((-1,1)) # columns vector of shape (N,1)
    y_inliers = inliers[:,1] # row vector of shape (N,)

    line = LinearRegression().fit( x_inliers, y_inliers )
    m = float(line.coef_)           # angular coefficient of the regressor line of 'inliers' set
    q = float(line.intercept_)      # interceptor of the regressor line of 'inliers' set

    # Return the lidar points (in polar coordinates) used for the RANSAC algorithm.
    # If no noise has been added, then 'lidarPointsUsed' coincides with 'pointsForRansac'
    # otherwise, 'lidarPointsUsed' = 'pointsForRansac' + white noise
    # (JUST FOR VISUALIZATION: THESE POINTS ARE SENT TO RVIZ2 )
    lidarPointsUsed = cart2pol(X, y)

    return inliers, outliers, m, q, lidarPointsUsed

# END ransac


#--------------------------------------------------------------------------------------------------------------------#


# Function that runs RANSAC 2 times, in order to handle corners. In our ROS2
# implementation it won't be used because the first line is considered to be the
# best, having more inliers.
def my_ransac_multiline(threshold, iterations, x, y):
    print("-----------------------")

    # Compute the first candidate model and retrieve inliers, outliers
    inliers_1, outliers_1, m_best_1, q_best_1 = ransac(threshold, iterations, x, y)
    print("First RANSAC iteration, found", len(inliers_1), "inliers and", len(outliers_1), "outliers")

    if len(outliers_1) > 0.2 * len(inliers_1):
        # Compute the second candidate model giving as points the outliers of the first iteration
        inliers_2, outliers_2, m_best_2, q_best_2 = ransac(threshold, iterations, outliers_1[:, 0], outliers_1[:, 1])
        print("Second RANSAC iteration, found", len(inliers_2), "inliers and", len(outliers_2), "outliers")

    else:
        print("Second RANSAC iteration, no inliers found (1 line problem)")
        inliers_2, outliers_2, m_best_2, q_best_2 = [], [], [], []

    print("-----------------------\n")
    return inliers_1, outliers_1, m_best_1, q_best_1, inliers_2, outliers_2, m_best_2, q_best_2

# END my_ransac_multiline


#--------------------------------------------------------------------------------------------------------------------#


class RansacLine():
    def __init__(self, threshold=0.001, nr_iter=100, add_noise=False, sigma_noise=0.001):
        # Attributes relative to Ransac algorithm
        self.threshold = threshold
        self.iterations = nr_iter

        # Optional noise to add to lidar distance measures
        self.add_noise = add_noise
        # Standard deviation of the white noise with 0 mean
        self.sigma_noise = sigma_noise

        self.inliers = None
        self.outliers = None
        self.lidarInputPoints = None
        # the set of distances of the lidar used for ransac, eventually added with gaussian noise
        # (JUST FOR VISUALIZATION, TO SEND IN RVIZ2 THE POINTS WITH NOISE)

        # Attributes relative to the line computed with RANSAC
        self.m = None    # angular coefficient
        self.q = None    # intercept
    
    
    def find_with_ransac(self, distances, angles):
        self.inliers, self.outliers, self.m, self.q, self.lidarInputPoints = ransac(self.threshold, self.iterations, \
                                                                            distances, angles, self.add_noise, self.sigma_noise)

    def reset(self):
        self.m = None
        self.q = None
        self.inliers = None
        self.outliers = None
        self.lidarInputPoints = None

