#!/usr/bin/env python
import roslib; roslib.load_manifest('tcp_to_arduino')
import rospy
from sensor_msgs.msg import Joy
from tcp_to_arduino.srv import *
import time

# requires the following nodes and services to be running: 
# rosrun joy joy_node
# rosrun tcp_to_arduino send_data_to_arduino_service.py

def ps3_to_pwm(val, command=None):

    if command == 'speed':
        val /= 1.
    elif command == 'steer':
        val /= 3.

    max_val = 254
    min_val = 127
    
    return int(((val+1.)/2.) * (max_val-min_val) + min_val)

    #return int((val+1.)*255/2.)

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
        if self.Joy is not None: #self.joy_changed: # used to only send data if it is new
            time_start = time.time()
            motor_speed_pwm = ps3_to_pwm(-1*self.Joy.axes[1], command='speed')
            data = self.send_data_to_arduino("s", "mspd", motor_speed_pwm)
            #print "set data: ", data            
            print 'communication time roundtrip: ', time.time() - time_start
            
            motor_steer_pwm = ps3_to_pwm(-1*self.Joy.axes[2], command='steer')
            data = self.send_data_to_arduino("s", "mstr", motor_steer_pwm)
            #print "set data: ", data      
                  
            self.joy_changed = False
            
            

if __name__ == '__main__':

    rate = 10
    arduino_data_transmission = Arduino_Data_Client(rate)
    
