#!/usr/bin/env python
import roslib
roslib.load_manifest('optic_flow')
import sys
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as numpy
import cv_numpy

from optic_flow.msg import FloatArray
from optic_flow.srv import *

class Optic_Flow_Calculator:

  def __init__(self, camera=None):
  
    if camera is None:
        camera = ''
    self.image_source = camera + "/image_raw"
    
    # Initialize
    self.bridge = CvBridge()
    self.prev_image = None
    self.msg = None
    rospy.init_node('optic_flow', anonymous=True)
    
    # Lucas Kanade Service
    serv_name = camera + "/return_optic_flow"
    self.optic_flow_service = rospy.Service(serv_name, ReturnImageFloat, self.handle_return_optic_flow)
    
    # Lucas Kanade Publisher
    pub_name = camera + "/optic_flow"
    self.optic_flow_pub = rospy.Publisher(pub_name, FloatArray)
    
    # Raw Image Subscriber
    self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)
    
  def image_callback(self,data):
    try:
        curr_image = self.bridge.imgmsg_to_cv(data, "mono8")
        
        # For first loop
        if self.prev_image is not None:
            prev_image = self.prev_image
        else:
            prev_image = curr_image
            
        velx = cv.CreateImage((curr_image.width, curr_image.height), cv.IPL_DEPTH_32F,1)
        vely = cv.CreateImage((curr_image.width, curr_image.height), cv.IPL_DEPTH_32F,1)
        
        winSize = (5,5)
        
        cv.CalcOpticalFlowLK(prev_image, curr_image, winSize, velx, vely)
        velx_np = cv_numpy.cv2array(velx)
        
        # publish
        msg = FloatArray()
        msg.shape = [velx_np.shape[0], velx_np.shape[1]]
        msg.data = [i for i in velx_np.reshape(velx_np.shape[0]*velx_np.shape[1], 1)]
        self.optic_flow_pub.publish(msg)
        self.msg = msg
        
        self.prev_image = curr_image
          
    except CvBridgeError, e:
      print e
  
  def handle_return_optic_flow(self, request):
        ans = ReturnImageFloatResponse()
        ans.shape = self.msg.shape
        ans.data = self.msg.data
        return ans

def main(args):
  optic_flow_calculator = Optic_Flow_Calculator(camera="my_camera")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
