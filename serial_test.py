import serial

#Serial port configuration
serial_port = '/dev/ttyACM0' #not sure if this is the right one
baud_rate = 9600 

#Initialize the serial port
ser = serial.Serial(serial_port,baud_rate)


while True:
    if ser.in_waiting > 0:
            #line = ser.readline() #just for testing
            #print(line)

            try:
                #line = ser.readline().decode('utf-8').rstrip() #read a line and decode
                print("ready to read")
                line = ser.read_until(b"\x77")
                print(line)
                print("done reading")

            except ValueError:
                print('Invalid data received')