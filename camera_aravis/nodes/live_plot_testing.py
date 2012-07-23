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

        ### parameters ###
        self.blur_radius = 10
        self.delay = 1
        self.pixel_shifts_template_radius = 2
        self.pixel_shifts_search_radius = 20
        ##################
        
        data = self.return_image_float()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
        smooth_image = smoother(image_row, self.blur_radius)
        self.smooth_image_prev = smooth_image
        pixel_shifts = tm.calc_pixel_shifts(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), self.pixel_shifts_template_radius, self.pixel_shifts_search_radius)[0,:]
        
        self.fig = plt.figure()
        self.alphas = np.arange(0, len(image_row), 1)
        
        self.line, = plt.plot(self.alphas, image_row, '.')
        self.smooth_image_line, = plt.plot(self.alphas, smooth_image, 'r-')
        self.smooth_image_prev_line, = plt.plot(self.alphas, self.smooth_image_prev, 'g-')
        self.pixel_shifts_line, = plt.plot(self.alphas, pixel_shifts, 'm*')
                
        self.image_animation = animation.FuncAnimation(self.fig, self.update_line, self.alphas, init_func=self.init_plot, interval=50, blit=True)
        
        plt.xlim(self.alphas[0], self.alphas[-1])
        plt.ylim(0, 255)
        plt.xlabel('pixels')
        plt.ylabel('pixel values')
        plt.title('test')
        plt.show()

    def update_line(self, i):
    
        data = self.return_image_float()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
        smooth_image = smoother(image_row, self.blur_radius)
        pixel_shifts = tm.calc_pixel_shifts(self.smooth_image_prev.reshape([1,len(self.smooth_image_prev)]), smooth_image.reshape([1,len(smooth_image)]), self.pixel_shifts_template_radius, self.pixel_shifts_search_radius)[0,:] + 150
                
        self.line.set_data(self.alphas, image_row)
        self.smooth_image_line.set_data(self.alphas, smooth_image)
        self.smooth_image_prev_line.set_data(self.alphas, self.smooth_image_prev)
        self.pixel_shifts_line.set_data(self.alphas, pixel_shifts)
        
        self.smooth_image_prev = smooth_image
            
        return self.line, self.smooth_image_line, self.smooth_image_prev_line, self.pixel_shifts_line
    
        
    def init_plot(self): # required to start with clean slate
        self.line.set_data([],[])
        self.smooth_image_line.set_data([],[])
        self.smooth_image_prev_line.set_data([],[])
        self.pixel_shifts_line.set_data([],[])
        return self.line, self.smooth_image_line, self.smooth_image_prev_line, self.pixel_shifts_line
        
    
        
        
if __name__ == '__main__':
    live_plotter = Live_Plotter(camera="my_camera")
