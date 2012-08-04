#!/usr/bin/env python
import roslib; roslib.load_manifest('camera_aravis')

from camera_aravis.srv import *
import rospy
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import numpy
import float_msg_to_numpy as fmtn
from scipy import signal 

import template_matching as tm

from kalman_smoother import kalman_smoother
import fit_data
import ransac

from camera_aravis.msg import FloatArray

def gauss_kern(size, sizey=None):
    """ Returns a normalized 2D gauss kernel array for convolutions """
    size = int(size)
    if not sizey:
        sizey = size
    else:
        sizey = int(sizey)
    x, y = np.mgrid[-size:size+1, -sizey:sizey+1]
    g = np.exp(-(x**2/float(size)+y**2/float(sizey)))
    return g / g.sum()

def blur_image(im, n, ny=None) :
    """ blurs the image by convolving with a gaussian kernel of typical
size n. The optional keyword argument ny allows for a different
size in the y direction.
"""
    g = gauss_kern(n, sizey=ny)
    improc = signal.convolve(im,g, mode='valid')
    return(improc)
    
def smoother(a, n):
    
    astack = np.vstack((a,a,a))
    asmooth = blur_image(astack, 1, n)
    a[n:len(a)-n] = asmooth
    
    return a
    

class Live_Plotter:
    
    def __init__(self, camera=""):
        serv_name = camera + "/return_image_float"
        rospy.wait_for_service(serv_name)
        self.return_image_float = rospy.ServiceProxy(serv_name, ReturnImageFloat)
        time.sleep(2)
        
        self.ransac_pub = rospy.Publisher('ransac_data', FloatArray)
        rospy.init_node('liveplotter', anonymous=True)
        
        ### parameters ###
        self.blur_radius = 20
        self.delay = 1
        self.pixel_shifts_template_radius = 2
        self.pixel_shifts_search_radius = 40
        ##################
        
        ## fitter
        # giving initial parameters
        n_inputs = 2
        n_outputs = 1
    
        input_columns = range(n_inputs) # the first columns of the array
        output_columns = [n_inputs+i for i in range(n_outputs)] # the last columns of the array
        self.model = ransac.LinearLeastSquaresModel(input_columns,output_columns,debug=False)
        
        
        data = self.return_image_float()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
        smooth_image = smoother(image_row, self.blur_radius)
        self.smooth_image_prev = smooth_image
        pixel_shifts = tm.calc_pixel_shifts(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), self.pixel_shifts_template_radius, self.pixel_shifts_search_radius)[0,:]
        pixel_shifts_kalman = kalman_smoother(pixel_shifts, xhat0=0, P0=0, Q=1e-5, R=0.1**2)
        
        #fit_data.fit(self.f, [self.slope, self.intercept], pixel_shifts[200:-200])
        
        print '*'*80
        print pixel_shifts_kalman.shape
        
        reichardt_correlator = tm.reichardt_correlator(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), 2)
        reichardt_correlator2 = tm.reichardt_correlator(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), 8)
        
        self.fig = plt.figure()
        self.alphas = np.arange(0, len(image_row), 1)
        
        self.line, = plt.plot(self.alphas, image_row, '.')
        self.smooth_image_line, = plt.plot(self.alphas, smooth_image, 'r-')
        self.smooth_image_prev_line, = plt.plot(self.alphas, self.smooth_image_prev, 'g-')
        self.pixel_shifts_line, = plt.plot(self.alphas, pixel_shifts, 'm*')
        
        #fitted = self.alphas*self.slope() + self.intercept()
        self.pixel_shifts_kalman_line, = plt.plot(self.alphas, self.alphas, 'b')        
        self.reichardt_correlator_line, = plt.plot(self.alphas, reichardt_correlator.reshape([reichardt_correlator.shape[1],1]), 'g')
        self.reichardt_correlator_line2, = plt.plot(self.alphas, reichardt_correlator2.reshape([reichardt_correlator2.shape[1],1]), 'r')
        
        self.pixel_shifts = pixel_shifts
                
        self.image_animation = animation.FuncAnimation(self.fig, self.update_line, self.alphas, init_func=self.init_plot, interval=50, blit=True)
        
        plt.xlim(self.alphas[0], self.alphas[-1])
        plt.ylim(-150, 255)
        plt.xlabel('pixels')
        plt.ylabel('pixel values')
        plt.title('test')
        plt.show()
        
    def f(self, x): 
        return self.slope()*x + self.intercept()

    def update_line(self, i):
    
        data = self.return_image_float()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
        smooth_image = smoother(image_row, self.blur_radius)
        smooth_image -= np.mean(smooth_image)
        
        pixel_shifts = tm.calc_pixel_shifts(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), self.pixel_shifts_template_radius, self.pixel_shifts_search_radius)[0,:] + 150
        reichardt_correlator = tm.reichardt_correlator(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), 8)
        reichardt_correlator2 = tm.reichardt_correlator(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), 20)
                
        
        #fit_data.fit(self.f, [self.slope, self.intercept], pixel_shifts[200:-200])
        ransac_fit = self.get_ransac_fit(pixel_shifts)
        
        # publish msg
        msg = FloatArray()
        msg.shape = [ransac_fit.shape[0], ransac_fit.shape[1]]
        msg.data = [i for i in ransac_fit]
        self.ransac_pub.publish(msg)
        
                
        self.line.set_data(self.alphas, image_row)
        self.smooth_image_line.set_data(self.alphas, smooth_image)
        self.smooth_image_prev_line.set_data(self.alphas, self.smooth_image_prev)
        self.pixel_shifts_line.set_data(self.alphas, pixel_shifts)
        
        #fitted = self.alphas*self.slope() + self.intercept()
        
        all_data = np.vstack((self.alphas, np.ones_like(self.alphas), pixel_shifts)).T
        vals = self.model.get_value(all_data, ransac_fit)
        self.pixel_shifts_kalman_line.set_data(self.alphas, vals)
        self.reichardt_correlator_line.set_data(self.alphas, reichardt_correlator.reshape([reichardt_correlator.shape[1],1]))
        self.reichardt_correlator_line2.set_data(self.alphas, reichardt_correlator2.reshape([reichardt_correlator2.shape[1],1]))
        
        self.smooth_image_prev = smooth_image
        #self.pixel_shifts = kalman_smoothed
            
        return self.line, self.smooth_image_line, self.smooth_image_prev_line, self.pixel_shifts_line, self.pixel_shifts_kalman_line
    
        
    def init_plot(self): # required to start with clean slate
        self.line.set_data([],[])
        self.smooth_image_line.set_data([],[])
        self.smooth_image_prev_line.set_data([],[])
        self.pixel_shifts_line.set_data([],[])
        self.pixel_shifts_kalman_line.set_data([],[])
        self.reichardt_correlator_line.set_data([], [])
        self.reichardt_correlator_line2.set_data([], [])
        return self.line, self.smooth_image_line, self.smooth_image_prev_line, self.pixel_shifts_line, self.pixel_shifts_kalman_line, self.reichardt_correlator_line, self.reichardt_correlator_line2
        
    
    def get_ransac_fit(self, all_data):
        n_samples = len(all_data)
        all_data = np.vstack((self.alphas[200:-200], np.ones_like(self.alphas[200:-200]), all_data[200:-200])).T
        
        # run RANSAC algorithm
        ransac_fit, ransac_data = ransac.ransac(all_data,self.model,
                                         50, 10, 2e3, 100, # misc. parameters
                                         debug=False,return_all=True)
        
        return ransac_fit
        
if __name__ == '__main__':
    live_plotter = Live_Plotter(camera="my_camera")
