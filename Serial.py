import serial
import matplotlib.pyplot as plt
import struct
import time
import pandas as pd

from matplotlib.animation import FuncAnimation

#Serial port configuration
serial_port = '/dev/ttyACM0' #not sure if this is the right one
baud_rate = 9600 

#Initialize the serial port
ser = serial.Serial(serial_port,baud_rate)

#Data list
x_data = []
y_data = []

x_data_persistent = []
y_data_persistent = []

#Plotting setup
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()

ax.set_title('Just testing') #Change this title
ax.set_xlabel('Time')
ax.set_ylabel('Sensor value')

def update(frame):
    global x_data, y_data, x_data_persistent, y_data_persistent
    
    if ser.in_waiting > 0:
        #line = ser.readline() #just for testing
        #print(line)

        try:
            #line = ser.readline().decode('utf-8').rstrip() #read a line and decode
            line = ser.read_until(b'\x77')
            
            angular_velocity = struct.unpack('f', line[2:5+1])[0]
            print(f"angular_velocity = {angular_velocity }")
            torque = struct.unpack('f', line[6:9+1])[0]
            print(f"torque = {torque}")
            pressure_diff = struct.unpack('f', line[10:13+1])[0]
            print(f"pressure_diff = {pressure_diff}\n")

            #update data list
            x_data += [time.time() - t0]
            x_data_persistent += [time.strftime("%H:%M:%S")]
            y_data += [pressure_diff ]
            y_data_persistent += [pressure_diff ]

            #limit the number of data points displayed
            if len(x_data) > 120:
                x_data.pop(0)
                y_data.pop(0)

            #clear the plot and plot new data
            ax.clear()
            ax.plot(x_data,y_data)

            ax.set_title('pressure_diff ') #Change this title
            ax.set_xlabel('Time since test start (s)')
            ax.set_ylabel("pressure_diff ")

        except struct.error:
            print('Invalid data received')

t0_timestamp = time.strftime("%H:%M:%S")
t0 = time.time()

#animation function
ani = FuncAnimation(fig,update,interval=100)

plt.show()

csv_data = {
    'time': x_data_persistent,
    'angular velocity': y_data_persistent
}

csv_data = pd.DataFrame(csv_data)
csv_data.to_csv('torquemeter-' + t0_timestamp + '.csv', index=False)