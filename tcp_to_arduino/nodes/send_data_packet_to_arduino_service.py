#!/usr/bin/env python
import roslib
roslib.load_manifest('tcp_to_arduino')
import rospy
import socket
from tcp_to_arduino.srv import *
import time

class Arduino_Data_Transmission:
    def __init__ (self, TCP_IP, TCP_PORT, BUFFER_SIZE):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print 'socket acquired, connecting...'
        self.sock.connect((TCP_IP, TCP_PORT))
        print 'socket connected to: ', TCP_IP, 'port: ', TCP_PORT 
        rospy.init_node('send_data_packet_to_arduino_service', anonymous=True) 
        
        self.service = rospy.Service('send_data_packet_to_arduino', SendDataPacketToArduino, self.handle_transmission)
        
        self.main()
                
    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting Down"
        self.sock.close()
        
    def handle_transmission(self, request):
        msg_to_arduino = ""
        for i, action in enumerate(request.action):
            msg_to_arduino += request.action[i]
            msg_to_arduino += ":"
            msg_to_arduino += request.dataName[i]
            msg_to_arduino += "="
            msg_to_arduino += str(request.dataVal)
            msg_to_arduino += '%'
        msg_to_arduino += '\n'
            
        #print 'sending msg: ', msg_to_arduino 
        self.sock.send(msg_to_arduino)
        
        if 1: #request.action == 'get':
            data = ''
            collect_data = True
            i = 0
            time_busy = 0
            while collect_data:
                i += 1
                time_start = time.time()
                data += self.sock.recv(BUFFER_SIZE)
                if data[-1] == '\n':
                    collect_data = False
                time_done = time.time()
                time_busy += time_done - time_start
                print time_done - time_start, time_busy, i
        else:
            data = 1
        
        data_arr_strings = data.split(",")
        data_arr = [int(d) for d in data_arr_strings]
        
        return SendDataPacketToArduinoResponse(data_arr)
            
        

if __name__ == '__main__':

    TCP_IP = '192.168.0.104'
    TCP_PORT = 437
    BUFFER_SIZE = 1
    
    arduino_data_transmission = Arduino_Data_Transmission(TCP_IP, TCP_PORT, BUFFER_SIZE)
