#!/usr/bin/env python
import roslib
roslib.load_manifest('camera_aravis')
import sys
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as numpy

from camera_aravis.msg import FloatArray
from rospy.numpy_msg import numpy_msg

class Image_Converter:

  def __init__(self, camera=None):
  
    if camera is None:
        camera = ''
    self.image_source = camera + "/image_raw"
    self.pub_name = camera + "/image_float"
    self.bridge = CvBridge()
    
    #self.np_image_pub = rospy.Publisher(self.pub_name, numpy_msg(Uint16s))
    self.float_image_pub = rospy.Publisher(self.pub_name, FloatArray)
    rospy.init_node('image_converter', anonymous=True)
    
    self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)
    
    

  def image_callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        np_image = numpy.asarray(cv_image, dtype=numpy.float32)[300:302,:]
        msg = FloatArray()
        msg.shape = [np_image.shape[0], np_image.shape[1]]
        msg.data = [i for i in np_image.reshape(np_image.shape[0]*np_image.shape[1], 1)]
        self.float_image_pub.publish(msg)
          
    except CvBridgeError, e:
      print e
  

def main(args):
  image_converter = Image_Converter(camera="my_camera")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
