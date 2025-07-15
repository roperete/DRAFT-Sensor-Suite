#!/usr/bin/python

'''
Original data collection script by Ben Kenney - July 2012
This program reads data coming from the serial port and saves that data to a CSV file.
'''

import serial
import os
import time
from datetime import datetime

# sensorsNum has value 1 or 2 , it is the number of arduino connections
sensorsNum =1

# Function to read data from Arduino
# TODO : DD consider returning empty list when data is empty to enable not writing
#    empty data in CSV
def read_arduino(port, sensor_name):
    line = port.readline().rstrip().decode('utf-8')
    l=len(line)
    #print (f"{l} --{line }" )
    if l > 0:
        data = line.split(',')
        return [sensor_name] + data
    else:
        return []

# Function to write data to CSV file
# DD : Should we not avoid opening again and again ?
def write_to_csv(data, csvFile):
    csvFile.write(','.join(data) + '\n')
    csvFile.flush()

# DD : function to open a serial port and retries in case it is not yet available
# for example start current tool while arduino not yet started
# returns a open port or none if failed
def openSerialRetries(port):
    attempt = 1
    opened = False
    while attempt < 50 and not opened:
        try:
            port = serial.Serial(port,  baudrate=9600, timeout=1)
            opened = port.isOpen()
        except serial.SerialException:
            #print (port + " not available")
            time.sleep(500/1000)
        attempt += 1
    if attempt >= 50:
        return None
    else:
        return port

################### MAIN #############################################"
# Serial ports for Arduino connections
# DD : accept waiting a bit for the arduino to start
# so that one can start sensor reading before arduinbo is plugged in
# TODO : check if really necessary

port0 = openSerialRetries("/dev/ttyUSB0")
     
if sensorsNum >1 :
    port1 = openSerialRetries("/dev/ttyUSB1")
else :
    port1 = None 

# Create CSV file if it doesn't exist
# and put header only at creation
csvFileName = '../Data/sensor_data.csv'
if not os.path.exists(csvFileName):
    with open(csvFileName, 'w') as f:
        f.write('Timestamp,ReadNumber,Source,SensorName,Parameter,Value\n')

# DD reopen file in append to avoid frequent open/close
# in that case, think of flushing
# Note we also might need to use os.fsync() to ensure written on file
csvFile = open(csvFileName, 'a')
        
# Main loop to read data from Arduinos
# DD TODO consider whether to read last row of file to
# restart at last readNumber +1 instead of 0 
read_number = 1
try:
    while True:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if port0:       # Read data from Arduino 0
            arduino0_data = read_arduino(port0, 'Arduino0')
            if arduino0_data: # read a non-empty data during timeout
                arduino0_data = [timestamp, str(read_number)] + arduino0_data
                write_to_csv(arduino0_data, csvFile)
            
        if port1:      # Read data from Arduino 1
            arduino1_data = read_arduino(port1, 'Arduino1')
            arduino1_data = [timestamp, str(read_number)] + arduino1_data
            write_to_csv(arduino1_data, csvFile)
            
        # Increment read number after both Arduinos have been read
        read_number += 1

except KeyboardInterrupt:
    print("\nData collection stopped.")
