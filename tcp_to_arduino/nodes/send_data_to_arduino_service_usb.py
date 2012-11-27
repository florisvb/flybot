#!/usr/bin/env python
import roslib
roslib.load_manifest('tcp_to_arduino')
import rospy
from std_msgs.msg import *
from tcp_to_arduino.srv import *
import serial



class Arduino_Data_Transmission(serial.Serial):
    def __init__ (self, **kwargs):
        super(Arduino_Data_Transmission,self).__init__(**kwargs)
    
        rospy.init_node('send_data_to_arduino_service', anonymous=True) 
        self.service = rospy.Service('send_data_to_arduino', SendDataToArduinoUSB, self.handle_transmission)
        
        self.accel_data = [0 for i in range(100)]
        self.accel_data_smooth = [0 for i in range(100)]
        self.accelerometer = rospy.Publisher('accel', Int32)
        self.speed = rospy.Publisher('speed', Int32)
        
        self.main()
                
    def main(self):
        try:
            self.listen()
        except KeyboardInterrupt:
            print "Shutting Down"
        
    def listen(self):
        while 1:
            data = self.readline().strip()
            
            try:    
                data = int(data)
            except:
                data = None
                
            if data is not None:
                    
                self.accel_data.append(data)
                self.accel_data.pop(0)
                
                alpha = 0.09
                self.accel_data_smooth.append( self.accel_data_smooth[-2] + alpha*(self.accel_data[-1] - self.accel_data_smooth[-2]) )
                self.accel_data_smooth.pop(0)
                    
                self.accelerometer.publish(self.accel_data_smooth[-1])   
        
    def handle_transmission(self, spd):
        
        self.write('[%s,%s]\n'%(spd.dataVal,2))
        self.speed.publish(spd.dataVal)   
        
        return 1
        

if __name__ == '__main__':
    
    arduino_data_transmission = Arduino_Data_Transmission(port='/dev/ttyACM0',timeout=1, baudrate=19200)
