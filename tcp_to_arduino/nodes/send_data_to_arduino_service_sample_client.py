#!/usr/bin/env python
import roslib
roslib.load_manifest('tcp_to_arduino')
import rospy
from tcp_to_arduino.srv import *
import time

class Arduino_Data_Client:
    def __init__ (self, rate, action="send"):
        rospy.init_node('send_data_to_arduino_client', anonymous=True) 
        rospy.wait_for_service('send_data_to_arduino')
        self.send_data_to_arduino = rospy.ServiceProxy('send_data_to_arduino', SendDataToArduino)
        self.prev_msg = 'set'
        self.action = action
        
        self.main(rate)
                
    def main(self, rate):
        try:
            if action == "send":
                rospy.Timer( rospy.Duration(1/float(rate)), self.send_data )
            elif action == "recieve":
                rospy.Timer( rospy.Duration(1/float(rate)), self.recieve_data )
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting Down"
        
    def send_data(self, timer):
        if self.prev_msg == 'set':
            data = self.send_data_to_arduino("g", "ax", 0)
            self.prev_msg = 'get'
            print "received data: ", data
        elif self.prev_msg == 'get':
            data = self.send_data_to_arduino("s", "mspd", 25)
            self.prev_msg = 'set'
            print "set data: ", data   
            
    def recieve_data(self, timer):         
        time_start = time.time()
        accel_x = self.send_data_to_arduino("get", "accelerometer_x", 0)
        accel_y = self.send_data_to_arduino("get", "accelerometer_y", 0)
        accel_z = self.send_data_to_arduino("get", "accelerometer_z", 0)
        sonar = self.send_data_to_arduino("get", "sonar", 0)
        time_elapsed = time.time() - time_start
        print "accel: x=", accel_x, " y=", accel_y, " z=", accel_z, " sonar: ", sonar, " || time elapsed: ", time_elapsed

if __name__ == '__main__':

    rate = 10
    action = "recieve"
    arduino_data_transmission = Arduino_Data_Client(rate, action)
