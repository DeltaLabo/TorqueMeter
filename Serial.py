import serial
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

#Serial port configuration
serial_port = '/dev/ttyACM0' #not sure if this is the right one
baud_rate = 9600 

#Initialize the serial port
ser = serial.Serial(serial_port,baud_rate)

#Data list
x_data = []
y_data = []

#Plotting setup
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()

ax.set_title('Just testing') #Change this title
ax.set_xlabel('Time')
ax.set_ylabel('Sensor value')

def update(frame):
    global x_data, y_data
    
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').rstrip() #read a line and decode
            sensor_value = float(line)

            #update data list
            x_data += [len(x_data)]
            y_data += [sensor_value]

            #limit the number of data points displayed
            if len(x_data) > 100:
                x_data.pop(0)
                y_data.pop(0)

            #clear the plot and plot new data
            ax.clear()
            ax.plot(x_data,y_data)

            ax.set_title('Just testing') #Change this title
            ax.set_xlabel('Time')
            ax.set_ylabel('Sensor value')

        except ValueError:
            print('Invalid data received')

#animation function
ani = FuncAnimation(fig,update,interval=100)

plt.show()


