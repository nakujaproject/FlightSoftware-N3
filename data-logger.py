from serial import Serial
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
import numpy as np
import csv
import atexit


port = "COM4"
baud_rate = 115200

def plot_data():

    # read file
    data_file = cbook.get_sample_data("sensor-data.csv", asfileobj=False)
    with cbook.get_sample_data("sensor-data.csv") as file:
        arr = np.loadtxt(file, delimiter=",", unpack="True")

        plt.plot(arr[0], arr[1])
        plt.title("Unfiltered x acceleration")
        plt.xlabel("Time (ms)")
        plt.ylabel("X acceleration (m/s^2)")

        plt.show()


def read_serial_data():
    
    serial_port = Serial(port, baud_rate)

    # csv writer object
    output_file = open("sensor-data.csv", "w", newline="")
    output_writer = csv.writer(output_file)

    # register function to run when the function exits
    # atexit.register(plot_data)

    while(True):
        now = datetime.now()
        current_time = now.timestamp()

        # read x acceleration into a csv file
        data = serial_port.readline()

        if data:
            data = float(data)

        output_writer.writerow([current_time, data])
    

read_serial_data()