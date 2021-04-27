#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

if __name__ == '__main__':
    plot = np.ones((50,50))
    fig, ax = plt.subplots()
    #obj = ax.imshow(plot, cmap='Blues', interpolation='none')#, origin="lower")#,cmap=plt.cm.jet)#, vmin=60, vmax=180)#, extent=[self.fft_t_axis[-1]-PLOT.PLOT_FFT_TIME, self.fft_t_axis[-1], 0, self.fft_f_axis[-1]])
    obj = ax.imshow(plot, origin="lower",cmap=plt.cm.jet)#, vmin=60, vmax=180)#, extent=[self.fft_t_axis[-1]-PLOT.PLOT_FFT_TIME, self.fft_t_axis[-1], 0, self.fft_f_axis[-1]])
    fig.colorbar(obj, ax=ax)
    #ax.axis('auto')
    #ax.yaxis.set_label_position("left")
    #ax.set_xlabel("Time(s)")
    #ax.set_ylabel("Frequency(Hz)")
    #self.ax[0][0].set_ylim([0, 20000])
    plt.show()
