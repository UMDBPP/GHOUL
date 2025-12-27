import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

from numpy import cos, sin, pi, absolute, arange
from scipy.signal import kaiserord, lfilter, firwin, freqz
from pylab import figure, clf, plot, xlabel, ylabel, xlim, ylim, title, grid, axes, show

# plt.style.use('seaborn-poster')

def DFT(x):
    """
    Function to calculate the 
    discrete Fourier Transform 
    of a 1D real-valued signal x
    """

    N = len(x)
    n = np.arange(N)
    k = n.reshape((N, 1))
    e = np.exp(-2j * np.pi * k * n / N)
    
    X = np.dot(e, x)
    
    return X

my_data = genfromtxt('ns120_ascent_rate.csv', delimiter=',')
fir_code_filter = genfromtxt('fir_output.csv', delimiter=',')
iir_code_filter = genfromtxt('iir_output.csv', delimiter=',') / 1000

#------------------------------------------------
# Create a FIR filter and apply it to x.
#------------------------------------------------

# The Nyquist rate of the signal.
nyq_rate = 1 / 2.0

# The desired width of the transition from pass to stop,
# relative to the Nyquist rate. 
width = 0.01/nyq_rate

# The desired attenuation in the stop band, in dB.
ripple_db = 60.0

# Compute the order and Kaiser parameter for the FIR filter.
N, beta = kaiserord(ripple_db, width)

# The cutoff frequency of the filter.
cutoff_hz = 0.01

# Use firwin with a Kaiser window to create a lowpass FIR filter.
taps = firwin(N, cutoff_hz/nyq_rate, window=('kaiser', beta))

with open("output.txt", "w") as txt_file:
    for line in taps:
        txt_file.write(str(line) + ", ") # works with any number of elements in a line

# Use lfilter to filter x with the FIR filter.
filtered_x = lfilter(taps, 1.0, my_data)

t = np.arange(0,17212,1)

plt.figure(figsize = (8, 6))
plt.plot(t, my_data, 'b', alpha=0.2)
plt.plot(t, fir_code_filter, 'r') 
plt.plot(t, iir_code_filter, 'g')
plt.ylabel('Ascent Rate')

plt.show()

X = DFT(filtered_x)

# calculate the frequency
N = len(X)
n = np.arange(N)
T = N
freq = n/T 

plt.figure(figsize = (8, 6))
plt.stem(freq, abs(X), 'b', markerfmt=" ", basefmt="-b")
plt.xlabel('Freq (Hz)')
plt.ylabel('DFT Amplitude |X(freq)|')
plt.xlim(0,0.5)
plt.show()
