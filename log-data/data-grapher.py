from serial import Serial
import csv
from datetime import datetime

port = "COM4"
baud_rate = 115200

def read_serial_data():
    serial_port = Serial(port, baud_rate)

    # csv writer object
    output_file = open("csv-ouput.csv", "w", newline="")
    output_writer = csv.writer(output_file)
    
    # attempt to opern port
    while(True):
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")

        # read x acceleration into a csv file
        line = serial_port.readline()

        output_writer.writerow([current_time, line])
    

read_serial_data()