#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData
from std_msgs.msg import Header


class PLOT:

    # Static params
    FRAME_ID = "base_link"
    PLOT_FFT_TIME = 5

    def __init__(self):
        # OnstartUp
        self.chunk = rospy.get_param('~chunk', 1024)
        self.fs = rospy.get_param('~fs', 192000)
        #print "Set Format   : ", str(self.format), type(self.format)

        # Initialize
        self.init_fft = False
        self.delta_f = 0.0
        self.delta_t = 0.0
        self.counter = 0
        self.plot_fft_ch1 = np.zeros((10,10))
        #self.fft_msg = HydrophoneFFTData()
        self.fig, self.ax = plt.subplots(1, figsize=(10,3))


        # Subscriber
        #rospy.Subscriber("/hydrophone_data", HydrophoneData, self.hydro_cb)
        rospy.Subscriber("/fft", HydrophoneFFTData, self.fft_cb)

        # Publisher
        #self.pub = rospy.Publisher("/fft", HydrophoneFFTData, queue_size=10)

    def fft_cb(self, msg):

        # Initialize fft plot frame
        if self.init_fft == False:
            print('init===========================================================')
            self.delta_f = msg.delta_f
            self.delta_t = msg.delta_t
            #print(f'delta_t : {self.delta_t}')
            #print(f'cal : {int(PLOT.PLOT_FFT_TIME//self.delta_t)}')
            self.plot_fft_ch1 = np.zeros((len(msg.fft_ch1),int(PLOT.PLOT_FFT_TIME//self.delta_t))) # (513,9375)
            #print(f'shape of plot array : {self.plot_fft_ch1.shape}') 
            self.fft_f_axis = np.arange(0,self.fs//2+self.delta_f,self.delta_f) # +self.delta_f for 2n+"1" # f_axis shape (513,)
            #print(f'shape of f_axis : {self.fft_f_axis.shape}')
            self.init_fft = True

        ch1 = np.array(msg.fft_ch1)[:,np.newaxis] + 170
        #print(f'ch1 shape (513,1): {ch1.shape}') # ch1.shape (513,1)

        # roll & replace previous data
        #self.plot_fft_ch1 = np.roll(self.plot_fft_ch1,-1,0)
        #print(f'plot shape : {self.plot_fft_ch1.shape}') # ch1.shape (513, 1875)(plot 1s)
        #self.plot_fft_ch1[:,-1] = ch1
        #print(f'self.plot_fft_ch1[:,-1], {self.plot_fft_ch1[:,-1].shape}') #(513,)
        self.plot_fft_ch1 = self.plot_fft_ch1[:,1:] # remove left column # (513,9374)
        self.plot_fft_ch1 = np.concatenate((self.plot_fft_ch1,ch1),axis=1) # (513,9375)
        

        # time axis
        self.counter = self.counter + 1
        if self.counter < self.plot_fft_ch1.shape[1]: # 9375
            self.fft_t_axis = np.arange(0,PLOT.PLOT_FFT_TIME-self.delta_t,self.delta_t)
            #print(f'shape of t_axis : {self.fft_t_axis.shape}') # t_axis shape (9375,)
        else:
            self.fft_t_axis = np.arange(0,PLOT.PLOT_FFT_TIME-self.delta_t,self.delta_t) + (self.counter-self.plot_fft_ch1.shape[1])*self.delta_t



    def animate(self,i):
        self.ax.clear()
    
        self.ax.imshow(self.plot_fft_ch1, origin="lower",cmap=plt.cm.jet)#, extent=[self.fft_t_axis[-1]-PLOT.PLOT_FFT_TIME, self.fft_t_axis[-1], 0, self.fft_f_axis[-1]])
        self.ax.axis('auto')
        self.ax.set_xlabel("Time(s)")
        self.ax.set_ylabel("Frequency(Hz)")
        #self.ax[0][0].set_ylim([0, 20000])
    


    def onShutdown(self):
        rospy.loginfo("FFT node Shutdown.")


if __name__ == '__main__':
    rospy.init_node('plot')
    node = PLOT()
    ani = FuncAnimation(node.fig, node.animate, interval=1000)
    rospy.on_shutdown(node.onShutdown)
    plt.show()
    rospy.spin()
