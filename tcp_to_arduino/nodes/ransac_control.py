#!/usr/bin/env python
import roslib; roslib.load_manifest('tcp_to_arduino')
import rospy
from sensor_msgs.msg import Joy
from tcp_to_arduino.srv import *
import time

from camera_aravis.msg import FloatArray


class Arduino_Data_Client:
    def __init__ (self, rate):
        rospy.wait_for_service('send_data_to_arduino')
        self.send_data_to_arduino = rospy.ServiceProxy('send_data_to_arduino', SendDataToArduino)
        
        self.ransac_sub = rospy.Subscriber("ransac_data",FloatArray,self.ransac_callback)

        rospy.init_node("ransac_to_arduino")
        self.slope = 0

        self.main()
                        
    def main(self):
        try:
            print 'spinning'
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting Down"
    
    def ransac_callback(self, ransac_data):
        slope = ransac_data.data[0]
        self.slope = (self.slope + slope) / 2.
        self.send_controls()
        
    def send_controls(self):
        
        desired_slope = -0.015
        error = self.slope - desired_slope
        
        Kp = 0.5
        
        control_signal = Kp*error
                    
        print control_signal
            
            

if __name__ == '__main__':

    rate = 10
    arduino_data_transmission = Arduino_Data_Client(rate)
    
