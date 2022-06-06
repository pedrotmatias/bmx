"""!
    Insert some cute comments to make it look professional

"""


from ast import Import
from operator import truediv
from pickle import TRUE
import sys
import time
from DFRobot_BMX160 import BMX160
from matplotlib import pyplot as plt
import collections
import numpy as np
from matplotlib.animation import FuncAnimation
import csv
import signal
from datetime import datetime
import os.path



bmx = BMX160(1)


#Trying to init the
while not bmx.begin():
    time.sleep(2)

fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
ax = plt.subplot(121)
ax1 = plt.subplot(122)
ax.set_facecolor('#DEDEDE')
ax1.set_facecolor('#DEDEDE')

magn_x = collections.deque(np.zeros(0))
magn_y = collections.deque(np.zeros(0))
magn_z = collections.deque(np.zeros(0))

max_x_registered = 0
max_y_registered = 0
min_x_registered = 0
min_y_registered = 0


# open the file in the write mode
file_exists = os.path.exists('output_data')
if( file_exists ):
    f = open('output_data2', 'w')
else:
    f = open('output_data', 'w')

# create the csv writer
writer = csv.writer(f)


def handler(signum, frame):
    global f
    f.close()
    exit(1)

signal.signal(signal.SIGINT, handler)


def gather_data(i):
    data= bmx.get_fifo_data()
    if data is not False:  
        print(data)
        counter = 0
        while counter < len( data[0] ):
            
            magn_x.append( data[0][counter] )
            magn_y.append( data[1][counter] )
            magn_z.append( data[2][counter] )
            writer.writerow( [ magn_x[-1], magn_y[-1], magn_z[-1] ] )
            global max_x_registered, min_x_registered
            if magn_x[ -1 ] > max_x_registered:
                max_x_registered = magn_x[ -1 ]
            if magn_x[ -1 ] < min_x_registered:
                min_x_registered = magn_x[ -1 ]
                
            global max_y_registered, min_y_registered
            if magn_y[ -1 ] > max_y_registered:
                max_y_registered = magn_y[ -1 ]
            if magn_y[ -1 ] < min_y_registered:
                min_y_registered = magn_y[ -1 ]
            
            counter += 1
            
        ax.cla()
        ax1.cla()
        
        """
        ax1.plot( magn_y )
        ax1.scatter(len(magn_y)-1, magn_y[-1])
        ax1.set_ylim( ( min_y_registered + ( min_y_registered / 10 ) ) , ( max_y_registered + ( max_y_registered / 10 ) ) )
        ax1.set_xlabel(" sample ")
        ax1.set_ylabel(" Y axis Magnetic value ")

        ax.plot( magn_x )
        ax.scatter(len(magn_x)-1, magn_x[-1])
        ax.set_ylim( ( min_x_registered + ( min_x_registered / 10 ) ) , ( max_x_registered + ( max_x_registered / 10 ) ) )
        ax.set_xlabel(" sample ")
        ax.set_ylabel(" X axis Magnetic value ")
        """
        
def main():
    while(True): 
        gather_data("yes")
    #ani = FuncAnimation( fig, gather_data, interval=10 )
    #plt.show()
    # close the file

        
        
if __name__ == "__main__":
    main()




