import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as pl

import glob #Module - finds pathnames matching pattern used in Unix
import serial #Module - pyserial, allows for integration of serial communication in python
import subprocess #Module - Allows us to spawn new processes and connect to their pipes
import sys #Module - provides access to some variable maintained by interpreter
import threading #Module - Allows for high level threading of information
import time #Module - allows time access and conversions

from cv2 import VideoCapture, VideoWriter
#from cv2.cv import CV_FOURCC

from stb_ros import hapticsstb_rt

# Constants for plotting
PLOT_FT = 1
PLOT_M40V = 2
PLOT_ACC = 3
PLOT_POS = 4
PLOT_ANG = 5

# Serial Commands, whenever one of these appear in the code, remember to check
# what it's resulting action in the arduino code is.
START = '\x01'
STOP = '\x02'
ID = '\x03'

# STB and Pedal IDs
STB_ID = '\x01'
PEDAL_ID = '\x02'

# Error for STB class
class EmptyPacketError(Exception):
    pass

# Class for the Haptics STB. Manages initialization, stop, start and reading raw serial packets
class STB(object):

    """
    Constructor for the STB class.

    @param sample_rate The rate at which the STB samples data
    """
    def __init__(self, sample_rate, STB='', pedal=False):

        """
        STB init
        """
        #Attempts to connect to the STB, if it fails it opens a serial port
        if STB == '':
            self.STB_dev = find_device(STB_ID)
        else:
            self.STB_dev = serial.Serial(STB)

        #Sets the timeout duration based on the sample rate
        if sample_rate > 100:
            self.STB_dev.timeout = 0.05
        else:
            self.STB_dev.timeout = 0.5

        """
        Pedal Init
        """
        #If a pedal is in use it connects to the pedal. Otherwise it signifies that the pedal will not be used
        if pedal:
            self._pedal = True
            self.pedal_dev = find_device(PEDAL_ID)
            self.pedal_dev.timeout = 0
        else:
            self._pedal = False
            self.pedal_dev = None

        #Returns an error if the sampling rate is too high for the device
        if sample_rate > 3000:
            print 'Sampling Rate too high!'
            raise ValueError

        self.update_rate(sample_rate)

        # Default bias vector is close to the empty weight, hasn't been tested for drift
        self.bias_vector = np.array([0.200, 0.0922, 0.0845, -0.123, 0.487, -0.0948], dtype=np.float64)
        #Default values to initiate the variables
        self.frame = 0
        self.packet_old = 300
        self.pack = '\00'*31

        #Default null values for the plots that are changed via the plot_init method
        self.plot_objects = None
        self.plot_type = 0
        self.plot_data = None

        #Default null values for the video capture that are changed via the video_init method
        self.video = False
        self.video_thread = None
        self.cap = None

    """
    Initializes the video capture if one is connected.

    @param video_filename the name that the video file will be created with.
    """
    def video_init(self, video_filename):

        if not self.video: #If this is the first time it's been called
            err = subprocess.call(['v4l2-ctl', '-i 4'])
            self.cap = VideoCapture(-1)
            if err == 1:
                print "VIDEO CAPTURE ERROR, CHECK CARD AND TRY AGAIN"
                sys.exit()


        fourcc = None #CV_FOURCC(*'XVID')

        #Creates the outfile for the video capture
        out = VideoWriter(video_filename, fourcc, 29.970, (720, 480))
        self.video_thread = OpenCVThread(self.cap, out)
        self.video = True

    def get_position(self, pos):
	self.pos = pos

    """
    Initializes the plot variables

    @param plot_type the desired plot to be made
    @param plot_length The length size of the box for the plot
    """
    def plot_init(self, plot_type, plot_length):
        self.update_rate(500) #When plotting the sample rate is default 500 Hz
        line_length = self.sample_rate*plot_length

        #Creates the objects for plotting and relevant other pieces
        self.plot_objects = plotting_setup(plot_type, line_length)
        self.plot_type = plot_type
        self.frame = 1

        #Develops appropriate plot data for the plot being used unless it is a plot not recognized
        if self.plot_type in [PLOT_FT, PLOT_M40V, PLOT_POS]:
            self.plot_data = np.zeros((line_length, 6))
        elif self.plot_type == PLOT_ACC:
            self.plot_data = np.zeros((line_length, 9))
	elif self.plot_type == PLOT_ANG:
	    self.plot_data = np.zeros((line_length,2))
        else:
            print "Unrecognized plotting type!"
            sys.exit()

    """
    Allows for the STB to update the rate it is sampling at, and repackages it in bytes as well

    @param sample_rate the new sample rate
    """
    def update_rate(self, sample_rate):
        self.sample_rate = sample_rate
        high = (sample_rate&int('0xFF00', 16))>>8 #Takes the high 8 bits, results in representative two digit hex 
        low = sample_rate&int('0x00FF', 16) #Takes the low 8 bits, results in representative two digit hex 
        self.sample_rate_bytes = chr(high)+chr(low) #The total number of bytes required to send the sample rate in unicode

    """
    Tare function to calculate a bias vector.
    """
    def bias(self):
        #Creates an array of zeroes
        bias_hist = np.zeros((self.sample_rate/5, 6))
        self.start_sampling()

        #Fills empty array with sample rate/5 readings from the force sensor
        for ii in range(0, self.sample_rate/5):
            bias_hist[ii, :] = self.read_m40v()

        self.stop_sampling()

        #Sets the bias vetor equal to the average of the data within each column
        self.bias_vector = np.mean(bias_hist, axis=0)
        self.packet_old = 300

    """
    Begins sampling. This method should be called when the STB is to start collecting data
    """
    def start_sampling(self):
        #Sends a start command as well as the rate to sample at to the microcontroller
        self.STB_dev.write(START + self.sample_rate_bytes)
        self.packet_old = 300

        #Turns on the video capture if video is being used
        if self.video:
            self.video_thread.start()

    """
    Stops sampling. This method should be called when the STB is done collecting data
    """
    def stop_sampling(self):
        #Sends a stop command to the microcontroller and flushes it of input
        self.STB_dev.write(STOP)
        self.STB_dev.flush()

        #If the video is being used and on, turn of the video thread
        if self.video and not self.video_thread == None:
            self.video_thread.stop.set()
            self.video_thread = None

    """
    Method to read packets sent to the computer from the microcontroller
    """
    def read_packet(self):
        #Gets back the packet
        pack = self.STB_dev.read(31)

        #Error if the packet is empty or not of the expected length
        if pack == '' or len(pack) != 31:
            raise EmptyPacketError
        packet_new = ord(pack[30]) #The ID number of the packet

        #Not sure what these four lines are for... Throws up an error if packets are synched up but why?
        if self.packet_old >= 256:
            self.packet_old = packet_new
        elif packet_new != (self.packet_old+1)%256: 
            print 'MISSED PACKET', packet_new, self.packet_old

        #Storing and the returning the last packet
        self.packet_old = packet_new
        self.pack = pack
        return pack

    """
    Reads a packet and sends the data to be transformed into both mini 40 forces
    and torques and accelerometer voltages

    @return 6-element vector with forces and torques [Fx, Fy, Fz, Tx, Ty, Tz]
    @return 6-element vector with accelerometer voltages [Acc1X, Acc1Y, Acc1Z, Acc2X, Acc2Y, Acc2Z, Acc3X, Acc3Y, Acc3Z]
    """
    def read_data(self):
        self.pack = self.read_packet()
        return hapticsstb_rt.serial_data(self.pack, self.bias_vector)

    """
    Reads a packet and sends the data to be transformed into Mini40 forces and torques

    @return Six-element vector with forces and torques [Fx, Fy, Fz, Tx, Ty, Tz]
    """
    def read_m40(self):
        self.pack = self.read_packet()
        return hapticsstb_rt.serial_m40(self.pack, self.bias_vector)

    """
    Uses the packet stored in the code and sends the data to be transformed into 
    Mini40 forces and torques. Can be the same as other calls.

    @return Six-element vector with forces and torques [Fx, Fy, Fz, Tx, Ty, Tz]
    """  
    def get_m40(self):
        return hapticsstb_rt.serial_m40(self.pack, self.bias_vector)

    """
    Reads a packet and sends the data to be transformed into Mini40 voltages

    @return 6-element vector with voltages [V0, V1, V2, V3, V4, V5]
    """
    def read_m40v(self):
        self.pack = self.read_packet()
        with_gain = hapticsstb_rt.serial_m40v(self.pack)
	return with_gain	
	
	
    """
    Uses the packet stored in the code and sends the data to be transformed into 
    Mini40 voltages. Can be the same as other calls.

    @return 6-element vector with voltages [V0, V1, V2, V3, V4, V5]
    """
    def get_m40v(self):
        with_gain = hapticsstb_rt.serial_m40v(self.pack)
	return with_gain 


    """
    Reads a packet and sends the data to be transformed into accelerometer voltages

    @return 6-element vector with accelerometer voltages 
            [Acc1X, Acc1Y, Acc1Z, Acc2X, Acc2Y, Acc2Z, Acc3X, Acc3Y, Acc3Z]
    """
    def read_acc(self):
        self.pack = self.read_packet()
        return hapticsstb_rt.serial_acc(self.pack)

    """
    Uses the packet stored in the code and sends the data to be transformed into 
    accelerometer voltages. Can be the same as other calls.

    @return 6-element vector with accelerometer voltages 
            [Acc1X, Acc1Y, Acc1Z, Acc2X, Acc2Y, Acc2Z, Acc3X, Acc3Y, Acc3Z]
    """  
    def get_acc(self):
        return hapticsstb_rt.serial_acc(self.pack)

    """
    Updates plots with new data depending on the type of plot being used
    """
    def plot_update(self):
        if self.plot_type in [PLOT_FT, PLOT_POS]:
            new_data = self.get_m40()
	    print(new_data)
        elif self.plot_type == PLOT_M40V:
            	volts = self.get_m40v()
		#print(volts)
		volts = volts - self.bias_vector
		#print(volts)
		#print(self.get_m40())
		"""
		M40_transform = np.array(  [[ 0.02830, 0.03219, 0.22129, 6.38303, -0.00591, -6.17983],
		[ -0.13072, -7.34148, 0.17871, 3.73494, -0.11637, 3.54944],
		[ 10.38185, -0.00020, 10.55504,	0.26375, 10.66492, -0.19923],
		[ 0.00159, -0.038886, 0.15213, 0.02358, -0.15058, 0.02134],
		[ -0.16999, -0.00076, 0.08629, -0.03158, 0.08695, 0.03151],
		[ -0.00339, -0.08649, -0.00075,	-0.08846, 0.00041, -0.08408]], dtype=np.float) 

		M40_transform_gained = [x / .43 for x in M40_transform]

		#M40_inverse = inv(M40_transform)
		#Exp_Load = [0, 0, 0, 0, 0, 0]; # where W = weight of whatever you put on there
		#V_exp = np.dot(M40_inverse, np.transpose(Exp_Load))
		#V_error = volts - V_exp # where volts is the amount of voltage we're reading in the code'
		#print(V_error)

		forces = np.dot(M40_transform_gained,np.transpose(volts))

		#testing
		#force = self.get_m40()
		print(forces)
		#print(np.dot(volts,M40_transform))
		#new_data = V_error
		"""

		new_data = volts
        elif self.plot_type == PLOT_ACC:
            new_data = self.get_acc()
            print new_data
	elif self.plot_type == PLOT_ANG:
	    new_data = self.pos
	
	
        self.plot_data = np.roll(self.plot_data, -1, axis=0)
        self.plot_data[-1, 0:] = new_data
        if self.frame % 50 == 0:
            hapticsstb_rt.plotting_updater(self.plot_type, self.plot_data, self.plot_objects)
            self.frame = 1
        else:
            self.frame += 1

    """
    Method to return the state of the pedal (0 unpressed, 1 pressed).
    """
    def pedal(self):
        if not self._pedal:
            return 0
        else:
            state = self.pedal_dev.read(1)
            if state == '':
                return 0
            else:
                return ord(state)

    """
    Closes communication with the STB, ending sampling and closing the pedal if being used.
    """
    def close(self):
        self.stop_sampling()
        self.STB_dev.close()
        if self._pedal:
            self.pedal_dev.close()

"""
Not entirely sure what this is for... Connection to video?
"""
class OpenCVThread(threading.Thread):
    def __init__(self, cap, out):
        threading.Thread.__init__(self)
        self.stop = threading.Event()
        self.out = out
        self.cap = cap

    def run(self):
        while not self.stop.is_set():
            ret, frame = self.cap.read()
            if ret == True:
                self.out.write(frame)

"""
Sets up the desired plots using pylab methods

@param plot_type the desired plot (options listed in README)
@param line_length The length of the line used in the plot 
"""
def plotting_setup(plot_type, line_length):

    pl.ion()

    if plot_type in [PLOT_FT, PLOT_M40V, PLOT_ACC, PLOT_ANG]:
        start_time = -1*(line_length-1)/500.0
        times = np.linspace(start_time, 0, line_length)

    # Force/Torque Graphing
    if plot_type == PLOT_FT:

        f, (axF, axT) = pl.subplots(2, 1, sharex=True)

        axF.axis([start_time, 0, -20, 20])
        axF.grid()
        axT.axis([start_time, 0, -1, 1])
        axT.grid()

        fx_line, = axF.plot(times, [0] * line_length, color='r')
        fy_line, = axF.plot(times, [0] * line_length, color='g')
        fz_line, = axF.plot(times, [0] * line_length, color='b')
        tx_line, = axT.plot(times, [0] * line_length, color='c')
        ty_line, = axT.plot(times, [0] * line_length, color='m')
        tz_line, = axT.plot(times, [0] * line_length, color='y')

        axF.legend([fx_line, fy_line, fz_line], ['FX', 'FY', 'FZ'], loc=2)
        axT.legend([tx_line, ty_line, tz_line], ['TX', 'TY', 'TZ'], loc=2)

        plot_objects = (fx_line, fy_line, fz_line, tx_line, ty_line, tz_line)

        pl.draw()

    # Mini40 Voltage Graphing
    elif plot_type == PLOT_M40V:

        pl.axis([start_time, 0, -5, 5])
        pl.grid()

        c0_line, = pl.plot(times, [0] * line_length, color='brown')
        c1_line, = pl.plot(times, [0] * line_length, color='yellow')
        c2_line, = pl.plot(times, [0] * line_length, color='green')
        c3_line, = pl.plot(times, [0] * line_length, color='blue')
        c4_line, = pl.plot(times, [0] * line_length, color='purple')
        c5_line, = pl.plot(times, [0] * line_length, color='gray')

        pl.legend([c0_line, c1_line, c2_line, c3_line, c4_line, c5_line], 
                  ['Channel 0', 'Channel 1', 'Channel 2', 'Channel 3', 'Channel 4', 'Channel 5'],
                  loc=2)

        plot_objects = (c0_line, c1_line, c2_line, c3_line, c4_line, c5_line)
        pl.draw()

    #Accelerometer Voltage Graphing
    elif plot_type == PLOT_ACC:

        f, (ax1, ax2, ax3) = pl.subplots(3, 1, sharex=True)

        ax1.axis([start_time, 0, -7, 7])
        ax2.axis([start_time, 0, -7, 7])
        ax3.axis([start_time, 0, -7, 7])
        ax1.grid()
        ax2.grid()
        ax3.grid()

        a1x_line, = ax1.plot(times, [0] * line_length, color='r')
        a1y_line, = ax1.plot(times, [0] * line_length, color='g')
        a1z_line, = ax1.plot(times, [0] * line_length, color='b')
        a2x_line, = ax2.plot(times, [0] * line_length, color='r')
        a2y_line, = ax2.plot(times, [0] * line_length, color='g')
        a2z_line, = ax2.plot(times, [0] * line_length, color='b')
        a3x_line, = ax3.plot(times, [0] * line_length, color='r')
        a3y_line, = ax3.plot(times, [0] * line_length, color='g')
        a3z_line, = ax3.plot(times, [0] * line_length, color='b')

        pl.legend([a1x_line, a1y_line, a1z_line], ['X', 'Y', 'Z'], loc=2)
        plot_objects = (a1x_line, a1y_line, a1z_line, a2x_line, a2y_line, a2z_line,
                        a3x_line, a3y_line, a3z_line)

        pl.draw()

    # 2D Position Plotting
    elif plot_type == 4:

        pl.axis([-.075, .075, -.075, .075])
        pl.grid()
        touch_point, = pl.plot(0, 0, marker="o", markersize=50)

        plot_objects = (touch_point,)
        pl.draw()

    #plot angle
    elif plot_type == 5:

        pl.axis([start_time, 0, 90, 160])
        pl.grid()
        angle_line, = pl.plot(times, [100]*line_length, color='r')
	
	pl.legend([angle_line],['Servo Angle'],loc=2)
        plot_objects = (angle_line,)
        pl.draw()

    else:
        print "INVALID GRAPHING MODE"
        return 0

    return plot_objects

"""
Communicates with the microcontroller to connect devices between the computer and the arduino.

@param tar
_id the ID of the target device attempting to be connected to.
"""
def find_device(target_id):
    #Determines the platform that the code is being run on. currently only supports Mac OS X and linux
    #Returns the pathname for the USB connection to the Arduino
    if sys.platform == 'darwin':
        devices = glob.glob('/dev/tty.usbmodem*')
    elif sys.platform == 'linux2':
        devices = glob.glob('/dev/ttyACM*')
    else:
        print "Unrecognized Platform!"
        sys.exit()

    for dev in devices:
        #Attempts to ping the device
        try:
            test_device = serial.Serial(dev, timeout=0.1)
        except:
            continue

        test_device.write(STOP) #Sends the stop signal to the microcontroller to make sure it isn't collecting data
        time.sleep(0.05) #Pauses execution of code
        test_device.flushInput() #Clears serial ports buffer
        test_device.write(ID) #Writes the ID

        dev_id = test_device.read(200)[-1] #Why is this 200 bytes?? It only reads one....

        #Returns the device if it is a match
        if dev_id == target_id:
            return test_device
        else:
            test_device.close()
    else:
        #notifies that the Device was not found
        print 'Device ' + hex(ord(target_id)) + ' not found! Check all cables!'
        sys.exit()
