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
angular_velocity_data = []
torque_data = [] 
pressure_diff_data = []
pressure1_data = []
pressure2_data = []

x_data_persistent = []
angular_velocity_data_persistent = []
torque_data_persistent = []
pressure_diff_data_persistent = []

#Plotting setup
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()
fig, bx = plt.subplots()
fig, cx = plt.subplots()

ax.set_title('') 
ax.set_xlabel('')
ax.set_ylabel('')

bx.set_title('') 
bx.set_xlabel('')
bx.set_ylabel('')

cx.set_title('') 
cx.set_xlabel('')
cx.set_ylabel('')

#Function definition
def update(frame):
    global x_data, x_data_persistent, angular_velocity_data, angular_velocity_data_persistent, torque_data, torque_data_persistent, pressure_diff_data, pressure_diff_data_persistent, pressure1_data, pressure2_data
    
    if ser.in_waiting > 0:
        try:
            line = ser.read_until(b'\x77')
            
            #Unpack data
            angular_velocity = struct.unpack('f', line[2:6])[0]
            print(f"Angular_Velocity = {angular_velocity}")

            torque = struct.unpack('f', line[6:10])[0]
            print(f"Torque = {torque}")

            pressure_diff = struct.unpack('f', line[10:14])[0]
            print(f"Diff_Pressure= {pressure_diff}")

            pressure1 = struct.unpack('f', line[14:18])[0]
            print(f"pressure_1 = {pressure1}")

            pressure2 = struct.unpack('f', line[18:22])[0]
            print(f"pressure_2 = {pressure2}\n")

            #update data list
            x_data += [time.time() - t0]
            x_data_persistent += [time.strftime("%H:%M:%S")]

            angular_velocity_data += [angular_velocity]
            angular_velocity_data_persistent += [angular_velocity]

            torque_data += [torque]
            torque_data_persistent += [torque]

            pressure_diff_data += [pressure_diff]
            pressure_diff_data_persistent += [pressure_diff]

            pressure1_data += [pressure1]
            pressure2_data += [pressure1]


            #limit the number of data points displayed
            if len(x_data) > 120:
                x_data.pop(0)
                angular_velocity_data.pop(0)
                torque_data.pop(0)
                pressure_diff_data.pop(0)

            #clear the plot and plot new data
            ax.clear()
            ax.plot(x_data,angular_velocity_data)

            ax.set_title('Angular Velocity vs Time') 
            ax.set_xlabel('Time since test start (s)')
            ax.set_ylabel('Angular Velocity (rad/s)')

            bx.clear()
            bx.plot(x_data,torque_data)

            bx.set_title('Torque vs Time') 
            bx.set_xlabel('Time since test start (s)')
            bx.set_ylabel('Torque (T)')

            cx.clear()
            cx.plot(x_data,pressure_diff_data)

            cx.set_title('Differential Pressure vs Time') 
            cx.set_xlabel('Time since test start (s)')
            cx.set_ylabel('Differential Pressure (psi)')

        except struct.error:
            print('Invalid data received')

t0_timestamp = time.strftime("%H:%M:%S")
t0 = time.time()

#animation function
ani = FuncAnimation(fig,update,interval=100)

plt.show()

csv_data = {
    'time': x_data_persistent,
    'angular velocity': angular_velocity_data_persistent,
    'torque': torque_data_persistent,
    'differential pressure': pressure_diff_data_persistent,
    'pressure 1': pressure1_data,
    'pressure 2': pressure2_data
}

csv_data = pd.DataFrame(csv_data)
csv_data.to_csv('torquemeter-' + t0_timestamp + '.csv', index=False)