import serial
import struct

#Serial port configuration
serial_port = '/dev/ttyACM0' #not sure if this is the right one
baud_rate = 9600 

#Initialize the serial port
ser = serial.Serial(serial_port,baud_rate)

i = 0

while True:
    if ser.in_waiting > 0:
        print('iter',i)
        i +=1

        try:
            line = ser.read_until(b'\x77')

            print('complete package',line,'\n')
            
            print('star byte',line[0],'\n')
            print('length',line[1],'\n')
            print('angula velocity package',line[2:6],'\n')
            print('torque package',line[6:10],'\n')
            print('differential pressure package',line[10:14],'\n')
            print('pressure1 package',line[14:18],'\n')
            print('pressure2 package',line[18:22],'\n')
            print('stop byte',line[23],'\n')
            print('package size',len(line),'\n')

           
            #Unpack data
            # Desempaquetar datos
            angular_velocity = struct.unpack('f', line[2:6])[0]
            print(f"Angular Velocity = {angular_velocity}")

            torque = struct.unpack('f', line[6:10])[0]
            print(f"Torque = {torque}")

            pressure_diff = struct.unpack('f', line[10:14])[0]
            print(f"Differential Pressure = {pressure_diff}")

            pressure1 = struct.unpack('f', line[14:18])[0]
            print(f"Pressure 1 = {pressure1}")

            pressure2 = struct.unpack('f', line[18:22])[0]
            print(f"Pressure 2 = {pressure2}\n")
        except IndexError:
            print('Error') #index out of range