#!/usr/bin/env python
import roslib
roslib.load_manifest('camera_aravis')
import sys
import rospy
import numpy as np

from camera_aravis.msg import FloatArray

import time
import pickle

class Save_Movie:

  def __init__(self, camera=None):
  
    if camera is None:
        camera = ''
    sub_name = camera+"/image_float"
    print "subscribing to: ", sub_name
    rospy.Subscriber(sub_name, FloatArray, self.image_callback)
            
    rospy.init_node('save_movie', anonymous=True)
    
    self.frames = {}
    self.time_start = None
    
    rospy.spin()
    
  def image_callback(self,data):
    
    if self.time_start is None:
        self.time_start = time.time()
        self.prev_frame_time = 0
        
    self.aquisition_time = time.time()
    
    #cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(data, "mono8"))
    #image_curr = np.asarray( cv.GetMat(cv_image) )
    #image_curr = np.mean(image_curr, axis=0)
    
    try:
        new_frame = self.frames.keys()[-1]+1
    except:
        new_frame = 0
        
    self.frames.setdefault(new_frame, data)
    print 'frame rate: ', 1/(self.aquisition_time - self.prev_frame_time)
    self.prev_frame_time = self.aquisition_time
    
    if time.time() - self.time_start > 5:
        fd = open( 'movie' , mode='w')
        pickle.dump(self.frames, fd)
        fd.close()
        print 'SAVED!'
        rospy.sleep(100)
        
if __name__ == '__main__':
    save_movie = Save_Movie("my_camera")
