#!/usr/bin/env python
import roslib; roslib.load_manifest('camera_aravis')

from camera_aravis.srv import *
from camera_aravis.msg import FloatArray
import rospy

class Return_Image_Float:
    def __init__(self, camera=""):
        node_name = camera + "_image_float_service"
        rospy.init_node(node_name)
        sub_name = camera+"/image_float"
        rospy.Subscriber(sub_name, FloatArray, self.process_data)
        
        self.data = None
        self.shape = None
        
        serv_name = camera + "/return_image_float"
        self.service = rospy.Service(serv_name, ReturnImageFloat, self.handle_return_image_float)
        rospy.spin()
    
    def process_data(self, data):
        self.data = data.data
        self.shape = data.shape
        
    def handle_return_image_float(self, request):
        ans = ReturnImageFloatResponse()
        ans.shape = self.shape
        ans.data = self.data
        return ans

if __name__ == "__main__":
    return_image_float = Return_Image_Float(camera="my_camera")
