#!/usr/bin/env python
import roslib; roslib.load_manifest('optic_flow')
import rospy

import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy
np = numpy

import float_msg_to_numpy as fmtn

from optic_flow.srv import *
from optic_flow.msg import FloatArray

class Live_Plotter:
    
    def __init__(self, camera=""):
        serv_name = camera + "/return_optic_flow"
        rospy.wait_for_service(serv_name)
        self.return_optic_flow = rospy.ServiceProxy(serv_name, ReturnImageFloat)
        time.sleep(2)
        
        rospy.init_node('liveplotter', anonymous=True)
        
        data = self.return_optic_flow()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
        
        self.fig = plt.figure()
        self.alphas = np.arange(0, len(image_row), 1)
        
        self.line, = plt.plot(self.alphas, image_row, '.')
                
        self.image_animation = animation.FuncAnimation(self.fig, self.update_line, self.alphas, init_func=self.init_plot, interval=50, blit=True)
        
        plt.xlim(self.alphas[0], self.alphas[-1])
        plt.ylim(-1, 1)
        plt.xlabel('pixels')
        plt.ylabel('pixel values')
        plt.title('test')
        plt.show()
        

    def update_line(self, i):
    
        data = self.return_optic_flow()
        image = fmtn.get_numpy_array(data.shape, data.data)
        image_row = np.mean(image, axis=0)
                
        self.line.set_data(self.alphas, image_row)

        return self.line,
    
        
    def init_plot(self): # required to start with clean slate
        self.line.set_data([],[])
        return self.line,
        

if __name__ == '__main__':
    live_plotter = Live_Plotter(camera="my_camera")
