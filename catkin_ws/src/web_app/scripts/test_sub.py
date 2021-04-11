#!/usr/bin/env python

import rospy
from std_msgs.msg import String 
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData
import numpy as np

def hydro_cb(msg):
  rospy.loginfo("Message recieved : ")
  # rospy.loginfo(msg)
  # ch1 = np.array([])
  ch1 = np.array(msg.data_ch1)
  print(len(ch1)) 
  print(type(ch1)) # type(ch1) : <tuple>

if __name__ == '__main__':

  rospy.init_node('smartphone')

  fs = rospy.get_param("/get_sound_data/pcm_sampleRate")

  print(fs)
  print(type(fs)) # type : int

  rospy.Subscriber("/hydrophone_data", HydrophoneData, hydro_cb)

  rospy.spin() # Keep running callbcak function
