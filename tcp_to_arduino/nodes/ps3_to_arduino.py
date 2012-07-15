#!/usr/bin/env python
import roslib; roslib.load_manifest('tcp_to_arduino')
import rospy
from sensor_msgs.msg import Joy
from tcp_to_arduino.srv import *
import time

# requires the following nodes and services to be running: 
# rosrun joy joy_node
# rosrun tcp_to_arduino send_data_to_arduino_service.py

def ps3_to_pwm(val):
    return int((val+1.)*255/2.)

class Arduino_Data_Client:
    def __init__ (self, rate):
        rospy.init_node("ps3_to_arduino")
        rospy.wait_for_service('send_data_to_arduino')
        self.send_data_to_arduino = rospy.ServiceProxy('send_data_to_arduino', SendDataToArduino)
        self.Joy = None
        self.joy_changed = False
        rospy.Subscriber("joy", Joy, self.save_buttons)

        self.main(rate)
        rospy.spin()
                
    def main(self, rate):
        try:
            rospy.Timer( rospy.Duration(1/float(rate)), self.send_data )
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting Down"
            
    def save_buttons(self, Joy):
        if Joy != self.Joy:
            self.Joy = Joy
            self.joy_changed = True
        
    def send_data(self, timer):
        if self.joy_changed: # used to only send data if it is new
            time_start = time.time()
            motor_speed_pwm = ps3_to_pwm(self.Joy.axes[1])
            data = self.send_data_to_arduino("set", "motor_speed", motor_speed_pwm)
            #print "set data: ", data            
            print 'communication time roundtrip: ', time.time() - time_start
            
            motor_steer_pwm = ps3_to_pwm(self.Joy.axes[2])
            data = self.send_data_to_arduino("set", "motor_steer", motor_steer_pwm)
            #print "set data: ", data      
                  
            self.joy_changed = False
            
            

if __name__ == '__main__':

    rate = 20
    arduino_data_transmission = Arduino_Data_Client(rate)
    