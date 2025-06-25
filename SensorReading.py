#!/usr/bin/python

'''
Original data collection script by Ben Kenney - July 2012
This program reads data coming from the serial port and saves that data to a CSV file.
'''

import serial
import os
from datetime import datetime

sensorsNum =1

# Function to read data from Arduino
# TODO : DD consider returning empty list when data is empty to enable not writing
#    empty data in CSV
def read_arduino(port, sensor_name):
    line = port.readline().rstrip().decode('utf-8')
    l=len(line)
    print (f"{l} --{line }" )
    if l > 0:
        data = line.split(',')
        return [sensor_name] + data
    else:
        return []

# Function to write data to CSV file
def write_to_csv(data, csv_file):
    with open(csv_file, 'a') as f:
        f.write(','.join(data) + '\n')

# Serial ports for Arduino connections
port0 = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)
if sensorsNum >1 :
    port1 = serial.Serial("/dev/ttyUSB1", baudrate=9600, timeout=1)

# Create CSV file if it doesn't exist
csv_file = 'sensor_data.csv'
if not os.path.exists(csv_file):
    with open(csv_file, 'w') as f:
        f.write('Timestamp,ReadNumber,SensorName,Parameter,Value\n')

# Main loop to read data from Arduinos
read_number = 1
try:
    while True:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Read data from Arduino 0
        arduino0_data = read_arduino(port0, 'Arduino0')
        if arduino0_data: # read a non-empty dat-a during timeout
            arduino0_data = [timestamp, str(read_number)] + arduino0_data
            write_to_csv(arduino0_data, csv_file)
        
        # Read data from Arduino 1
        if sensorsNum > 1:
            arduino1_data = read_arduino(port1, 'Arduino1')
            arduino1_data = [timestamp, str(read_number)] + arduino1_data
            write_to_csv(arduino1_data, csv_file)
        
         # Increment read number after both Arduinos have been read
    read_number += 1

except KeyboardInterrupt:
    print("\nData collection stopped.")
