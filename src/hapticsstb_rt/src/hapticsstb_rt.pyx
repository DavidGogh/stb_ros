# cython: profile=True

# These functions are time-sensitive functions for hapticsstb.py, you shouldn't have to call any of them directly

import numpy as np
cimport numpy as np
import matplotlib.pyplot as pl

# Constants for plot types
PLOT_FT = 1
PLOT_M40V = 2
PLOT_ACC = 3
PLOT_POS = 4
PLOT_ANG = 5

# Calibrated linear transform for our Mini40, from channel voltages to forces
# and torques

M40_transform_pregain = np.array(  [[ 0.02830, 0.03219, 0.22129, 6.38303, -0.00591, -6.17983],
[ -0.13072, -7.34148, 0.17871, 3.73494, -0.11637, 3.54944],
[ 10.38185, -0.00020, 10.55504,	0.26375, 10.66492, -0.19923],
[ 0.00159, -0.038886, 0.15213, 0.02358, -0.15058, 0.02134],
[ -0.16999, -0.00076, 0.08629, -0.03158, 0.08695, 0.03151],
[ -0.00339, -0.08649, -0.00075,	-0.08846, 0.00041, -0.08408]], dtype=np.float) 

DAQ_gain = .43
M40_transform = [x / DAQ_gain for x in M40_transform_pregain]

"""
Decodes serial packet and transforms information into the forces and torques read from the
Mini40

@param pack raw data in unicode string form
@param bias bias vector detected by the empty force plate
@return 6-element vector with forces and torques [Fx, Fy, Fz, Tx, Ty, Tz]
"""
def serial_m40(str pack, np.ndarray[np.float64_t, ndim = 1] bias):
	volts = np.zeros((6), dtype = np.float64) #intializing an array to all zeros for the data
	cdef int i, j, y

	for i in range(0,6):
		j = i*2
		#Pack is 12 bit string and each set of two bytes is one data point
		#Read in as two bytes, takes them in and concatenates them
		y = (ord(pack[j])<<8) + (ord(pack[j+1]))

		#If statement accounts for twos complement notation
		if y > 2048:
			#changed from 5-i to just i, 6/7/2018 ecao1@jhu.edu, berna3@umbc.edu
			volts[i] = <float>(y - 4096)*0.002
		else:
			#changed from 5-i to just i, 6/7/2018 ecao1@jhu.edu, berna3@umbc.edu
			volts[i] = <float>y*0.002

	temp_col = np.dot(M40_transform, np.transpose(volts-bias))
	#print(M40_transform)
	return np.transpose(temp_col)


"""
Decodes serial packet and transforms information into accelerometer voltages

@param pack raw data in unicode string form
@return 6-element vector with accelerometer voltages [Acc1X, Acc1Y, Acc1Z, Acc2X, Acc2Y, Acc2Z, Acc3X, Acc3Y, Acc3Z]
"""
def serial_acc(str pack):
	gees = np.zeros((9), dtype = np.float64)
	acc_order = [0,1,2,5,3,4,8,6,7] #Puts acc channels in x,y,z order

	cdef int i,j,y

	for i in range(0,9):
		j = (acc_order[i]+6)*2
		y = (ord(pack[j])<<8) + (ord(pack[j+1]))
		gees[i] = ((<float>y/1241)-1.65)*(15.0/3.3) #Conversion factor

	return gees

"""
Convience function to perform both the m40 force readings and acceleromter voltage readings
from a single packet of data

@param pack raw data in unicode string form
@param bias bias vector detected by the empty force plate
@return 6-element vector with forces and torques [Fx, Fy, Fz, Tx, Ty, Tz]
@return 6-element vector with accelerometer voltages [Acc1X, Acc1Y, Acc1Z, Acc2X, Acc2Y, Acc2Z, Acc3X, Acc3Y, Acc3Z]
"""
def serial_data(str pack, np.ndarray[np.float64_t, ndim = 1] bias):
	FT = serial_m40(pack, bias)
	ACC = serial_acc(pack)
	return np.hstack((FT, ACC))

"""
Decodes serial packet and transforms information into Mini40 voltages

@param pack raw data in unicode string form

@return 6-element vector with voltages [V0, V1, V2, V3, V4, V5]
"""
def serial_m40v(str pack):
	cdef int i, j, y
	volts = np.zeros((6), dtype = np.float64)
	for i in range(0,6):
		j = i*2
		y = (ord(pack[j])<<8) + (ord(pack[j+1]))
		if y > 2048: # This handles the twos complement negatives
			#changed from 5-i to just i, 6/7/2018 ecao1@jhu.edu, berna3@umbc.edu
			volts[i] = <float>(y - 4096)*0.002
		else:
			#changed from 5-i to just i, 6/7/2018 ecao1@jhu.edu, berna3@umbc.edu
			volts[i] = <float>y*0.002
	return volts

# Convenience function for creating two-byte serial packet for ints
def to16bit(x):
	if x > int('0xFFFF',16):
		raise ValueError

	high = (x&int('0xFF00',16))>>8
	low = x&int('0x00FF',16)

	return chr(high)+chr(low)

# Updates plots, called by STB.plot_update()
def plotting_updater(plot_type, np.ndarray[np.float64_t, ndim = 2] data, plot_objects):

	if plot_type == PLOT_FT:

		plot_objects[0].set_ydata(data[:,0].T)
		plot_objects[1].set_ydata(data[:,1].T)
		plot_objects[2].set_ydata(data[:,2].T)
		plot_objects[3].set_ydata(data[:,3].T)
		plot_objects[4].set_ydata(data[:,4].T)
		plot_objects[5].set_ydata(data[:,5].T)

	if plot_type == PLOT_M40V:

		plot_objects[0].set_ydata(data[:,0].T)
		plot_objects[1].set_ydata(data[:,1].T)
		plot_objects[2].set_ydata(data[:,2].T)
		plot_objects[3].set_ydata(data[:,3].T)
		plot_objects[4].set_ydata(data[:,4].T)
		plot_objects[5].set_ydata(data[:,5].T)
	
	if plot_type == PLOT_ACC:

		plot_objects[0].set_ydata(data[:,0].T)
		plot_objects[1].set_ydata(data[:,1].T)
		plot_objects[2].set_ydata(data[:,2].T)		

		plot_objects[3].set_ydata(data[:,3].T)
		plot_objects[4].set_ydata(data[:,4].T)
		plot_objects[5].set_ydata(data[:,5].T)		

		plot_objects[6].set_ydata(data[:,6].T)
		plot_objects[7].set_ydata(data[:,7].T)
		plot_objects[8].set_ydata(data[:,8].T)


	if plot_type == PLOT_POS:

		if abs(data[-1,2]) > .15:
			x = -1*data[-1,4]/data[-1,2]
			y = data[-1,3]/data[-1,2]
		else:
			x = y = 0

		plot_objects[0].set_ydata(y)
		plot_objects[0].set_xdata(x)


	if plot_type == PLOT_ANG:
		plot_objects[0].set_ydata(data[:,0].T)


	pl.draw()
