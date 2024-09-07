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
fig1, V = plt.subplots()
fig2, T = plt.subplots()
fig3, P = plt.subplots()

V.set_title('') 
V.set_xlabel('')
V.set_ylabel('')

T.set_title('') 
T.set_xlabel('')
T.set_ylabel('')

P.set_title('') 
P.set_xlabel('')
P.set_ylabel('')

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

            V.clear()
            V.plot(x_data,angular_velocity_data)

            V.set_title('Angular Velocity vs Time') 
            V.set_xlabel('Time since test start (s)')
            V.set_ylabel('Angular Velocity (rad/s)')

            T.clear()
            T.plot(x_data,torque_data)

            T.set_title('Torque vs Time') 
            T.set_xlabel('Time since test start (s)')
            T.set_ylabel('Torque (T)')
        
            P.clear()
            P.plot(x_data,pressure_diff_data)

            P.set_title('Differential Pressure vs Time') 
            P.set_xlabel('Time since test start (s)')
            P.set_ylabel('Differential Pressure (psi)')
            
        except struct.error:
            print('Invalid data received')

t0_timestamp = time.strftime("%d-%m-%Y_%H:%M:%S")
t0 = time.time()

#animation function
ani1 = FuncAnimation(fig1,update,interval=100)
ani2 = FuncAnimation(fig2,update,interval=100)
ani3 = FuncAnimation(fig3,update,interval=100)

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
csv_data.to_csv('Data/torquemeter_' + t0_timestamp + '.csv', index=False)