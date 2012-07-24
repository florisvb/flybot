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

class Live_Plotter:
    
    def __init__(self, camera=""):
        serv_name = camera + "/return_image_float"
        rospy.wait_for_service(serv_name)
        self.return_image_float = rospy.ServiceProxy(serv_name, ReturnImageFloat)
        
        time.sleep(2)
        
        data = self.return_image_float()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
        
        self.fig = plt.figure()
        self.alphas = np.arange(0, len(image_row), 1)
        
        self.line, = plt.plot(self.alphas, image_row, '.')
        #self.smooth_image_line, = plt.plot(self.alphas, self.smooth_image, 'r-')
        #self.smooth_image_line_prev, = plt.plot(self.alphas, self.smooth_image_prev, 'g-')
                
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
                
        self.line.set_data(self.alphas, image_row)
            
        return self.line,        
    
        
    def init_plot(self): # required to start with clean slate
        self.line.set_data([],[])
        return self.line,
        
    
        
        
if __name__ == '__main__':
    live_plotter = Live_Plotter(camera="my_camera")
