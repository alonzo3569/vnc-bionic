import numpy as np
import matplotlib.pyplot as plot
from scipy.fftpack import fft
#samples = np.linspace(0, t, int(fs*t), endpoint=False)


# Get x values of the sine wave
time        = np.arange(0, 0.5, 0.1);
# Amplitude of the sine wave is sine of a variable like time
amplitude   = np.sin(time)
# Plot a sine wave using time and amplitude obtained for the sine wave
plot.plot(time, amplitude)
# Give a title for the sine wave plot
plot.title('Sine wave')
# Give x axis label for the sine wave plot
plot.xlabel('Time')
# Give y axis label for the sine wave plot
plot.ylabel('Amplitude = sin(time)')
plot.grid(True, which='both')
plot.axhline(y=0, color='k')


print"Shape of amplitude : ", amplitude.shape

window = np.hanning(amplitude.shape[0])
result = np.fft.rfft(amplitude * window)
print"np rfft result : ", abs(result)


result2 = fft(amplitude * window)
print"scipy fft result : ", abs(result2)



plot.show()
