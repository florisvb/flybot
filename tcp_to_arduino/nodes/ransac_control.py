#!/usr/bin/env python
import roslib; roslib.load_manifest('tcp_to_arduino')
import rospy
from sensor_msgs.msg import Joy
from tcp_to_arduino.srv import *
import time

from camera_aravis.msg import FloatArray

def ps3_to_pwm(val, command=None):

    if command == 'speed':
        val /= 1.
    elif command == 'steer':
        val /= 3.

    max_val = 254
    min_val = 127
    
    return int(((val+1.)/2.) * (max_val-min_val) + min_val)
    

class Arduino_Data_Client:
    def __init__ (self, rate):
        rospy.wait_for_service('send_data_to_arduino')
        self.send_data_to_arduino = rospy.ServiceProxy('send_data_to_arduino', SendDataToArduino)
        
        rospy.Subscriber("ransac_data",FloatArray,self.ransac_callback)
        self.Joy = None
        rospy.Subscriber("joy", Joy, self.save_buttons)
        
        rospy.init_node("ransac_to_arduino")
        self.slope = 0

        self.main()
    
    def save_buttons(self, Joy):
        if Joy != self.Joy:
            self.Joy = Joy
                        
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
        
        Kp = -13
        
        control_signal = Kp*error
                    
        #print control_signal
        self.send_data(control_signal)
        
    def send_data(self, control_signal):
        if self.Joy is not None:
            motor_speed_ps3 = ps3_to_pwm(-1*self.Joy.axes[1], command='speed')
        else:
            motor_speed_ps3 = 0
        
        if self.Joy.axes[12] == 1:
            time_start = time.time()
            data = self.send_data_to_arduino("s", "mspd", motor_speed_ps3)
            print 'communication time roundtrip: ', time.time() - time_start, 'val: ', motor_speed_ps3
        
        else:
            time_start = time.time()
            motor_speed_control = ps3_to_pwm(control_signal, command='speed')
            if motor_speed_control < 160:
                motor_speed_control = 160
            if motor_speed_control > 190:
                motor_speed_control = 190
            data = self.send_data_to_arduino("s", "mspd", motor_speed_control)
            print 'val: ', control_signal, motor_speed_control
            
            
            

if __name__ == '__main__':

    rate = 10
    arduino_data_transmission = Arduino_Data_Client(rate)
    
