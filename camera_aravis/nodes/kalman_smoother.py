# Kalman filter example demo in Python

# A Python implementation of the example given in pages 11-15 of "An
# Introduction to the Kalman Filter" by Greg Welch and Gary Bishop,
# University of North Carolina at Chapel Hill, Department of Computer
# Science, TR 95-041,
# http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html

# by Andrew D. Straw

import numpy
import numpy as np
#import pylab


def kalman_smoother_from_center(z, xhat0=0, P0=0, Q=1e-5, R=0.1**2):
    center = int(len(z)/2.)
    z_left = z[0:center]
    z_left_smooth = kalman_smoother(z_left[::-1], xhat0=xhat0, P0=P0, Q=Q, R=R)[::-1]
    
    z_right = z[center:]
    z_right_smooth = kalman_smoother(z_right, xhat0=xhat0, P0=P0, Q=Q, R=R)
    
    z_smooth = np.hstack((z_left_smooth, z_right_smooth))
    
    return z_smooth

def kalman_smoother(z, xhat0=0, P0=0, Q=1e-5, R=0.1**2):

    # intial parameters
    #sz =  # size of array
    #x = -0.37727 # truth value (typo in example at top of p. 13 calls this z)
    #z = numpy.random.normal(x,0.1,size=sz) # observations (normal about x, sigma=0.1)


    # allocate space for arrays
    xhat=numpy.zeros_like(z)      # a posteri estimate of x
    P=numpy.zeros_like(z)         # a posteri error estimate
    xhatminus=numpy.zeros_like(z) # a priori estimate of x
    Pminus=numpy.zeros_like(z)    # a priori error estimate
    K=numpy.zeros_like(z)         # gain or blending factor


    # intial guesses
    xhat[0] = xhat0
    P[0] = P0

    for k in range(1,numpy.max(z.shape)):
        # time update
        xhatminus[k] = xhat[k-1]
        Pminus[k] = P[k-1]+Q

        # measurement update
        K[k] = Pminus[k]/( Pminus[k]+R )
        xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
        P[k] = (1-K[k])*Pminus[k]
        
    return xhat

