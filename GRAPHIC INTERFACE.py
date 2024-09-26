import customtkinter
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import struct
from datetime import datetime
import csv
import threading

################################################################
#                                                              #
#                     FUNCTION DEFINITION                      #
#                                                              #
################################################################

def enviar_hex(hex_string):
    data = bytes.fromhex(hex_string)
    ser.write(data)
    print(f"Enviado: {hex_string}")

def recibir_hex():
    data = ser.read()
    hex_data = data.hex()
    print(f"Recibido: {hex_data}")
    return hex_data

################################################################
#                                                              #
#                      BUTTON DEFINITION                       #
#                                                              #
################################################################

def tiempo_agotado():
    eventStop()
    clear_data()

def eventStart():
    try:
        expTime = 60 * int(stop_display.get())
        enviar_hex(start)
        temporizador = threading.Timer(expTime, tiempo_agotado)
        temporizador.start()
    except:
        print("ERROR")

def eventStop():
    try:
        enviar_hex(stop)
        clear_data()
    except:
        print("ERROR")

################################################################
#                                                              #
#                       GLOBAL VARIABLES                       #
#                                                              #
################################################################

#Data list
dataNumber = 0
time = []
p1 = []
p2 = []
dp = []
torque = []
angularVelocity = []
expTime = 1

# MESSAGES
start = "4D"
stop = "5A"

# SAVE PATH
csv_file_path = 'Results/data_log_{state}.csv'

# Configurar la comunicación serial
try: 
    serial_port = "COM18" # '/dev/ttyACM0'
    baud_rate = 9600 
    ser = serial.Serial(serial_port, baud_rate)
except:
    print("NO SE ENCONTRÓ EL PUERTO")

################################################################
#                                                              #
#                       DATA FUNCTIONS                         #
#                                                              #
################################################################

def read_serial_data():
    try:
        if ser.in_waiting > 23:  # Check if there's enough data for a full packet
            packet = ser.read_all()
            print("Datos recividos:")
            print(packet)
            process_serial_data(packet)
        root.after(10, read_serial_data)
    except:
        print("NEL PERRO")
        root.after(10, read_serial_data)

def process_serial_data(line):
    global dataNumber, p1, p2, dp, torque, angularVelocity, time
    
    try:         
        #Unpack data
        angularVelocity.append(round(struct.unpack('f', line[2:6])[0], 3))
        torque.append(round(struct.unpack('f', line[6:10])[0], 3))
        dp.append(round(struct.unpack('f', line[10:14])[0], 3))
        p1.append(round(struct.unpack('f', line[14:18])[0], 3))
        p2.append(round(struct.unpack('f', line[18:22])[0], 3))
        time.append(dataNumber)

        # Preparar los datos para guardar en CSV
        csv_data = [
            p1[-1],
            p2[-1],
            dp[-1],
            angularVelocity[-1],
            torque[-1],
            dataNumber]
        
        # Guardar datos en CSV
        dataNumber = dataNumber + 1
        update_graphs()
        upgrade_labels()
        save_to_csv(csv_data)
        
    except struct.error:
        print('Invalid data received')

def clear_data():
    global dataNumber, p1, p2, dp, torque, angularVelocity, time
    dataNumber = 0
    time.clear()
    p1.clear()
    p2.clear()
    dp.clear()
    torque.clear()
    angularVelocity.clear()

def save_to_csv(data):
    # Obtener la fecha y hora actuales
    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
    
    # Abrir el archivo CSV en modo anexado
    with open(csv_file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        
        # Escribir los datos en el archivo
        writer.writerow([timestamp] + data)    

################################################################
#                                                              #
#              FUNCIONES QUE ACTUALIZAN GRÁFICAS               #
#                                                              #
################################################################

# Función para actualizar la gráficas gráficas
def update_graphs():

    axPresGraph.clear()
    axPresGraph.plot(range(len(time)), dp)
    axPresGraph.set_xlabel('Time (s)', fontsize=12)
    axPresGraph.set_ylabel('Pressure Diferential (psi)', fontsize=12)
    axPresGraph.set_title("Pressure vs Time", fontsize=12)
    axPresGraph.set_ylim(0, None)
    axPresGraph.set_xlim(0, None)
    axPresGraph.grid(True)
    canvasPresGraph.draw()

    axAngularGraph.clear()
    axAngularGraph.plot(range(len(time)), angularVelocity)
    axAngularGraph.set_xlabel('Time (s)', fontsize=12)
    axAngularGraph.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    axAngularGraph.set_title("Angular Velocity vs Time", fontsize=12)
    axAngularGraph.set_ylim(0, None)
    axAngularGraph.set_xlim(0, None)
    axAngularGraph.grid(True)
    canvasAngularGraph.draw()

    axTorqGraph.clear()
    axTorqGraph.plot(range(len(time)), torque)
    axTorqGraph.set_xlabel('Time (s)', fontsize=12)
    axTorqGraph.set_ylabel('Torque (Nm)', fontsize=12)
    axTorqGraph.set_title("Torque vs Time", fontsize=12)
    axTorqGraph.set_ylim(0, None)
    axTorqGraph.set_xlim(0, None)
    axTorqGraph.grid(True)
    canvasTorqGraph.draw()

def clear_graphs():

    axPresGraph.clear()
    axPresGraph.set_xlabel('Time (s)', fontsize=12)
    axPresGraph.set_ylabel('Pressure Diferential (psi)', fontsize=12)
    axPresGraph.set_title("Pressure vs Time", fontsize=12)
    axPresGraph.set_ylim(0, None)
    axPresGraph.set_xlim(0, None)
    canvasPresGraph.draw()

    axAngularGraph.clear()
    axAngularGraph.set_xlabel('Time (s)', fontsize=12)
    axAngularGraph.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    axAngularGraph.set_title("Angular Velocity vs Time", fontsize=12)
    axAngularGraph.set_ylim(0, None)
    axAngularGraph.set_xlim(0, None)
    canvasAngularGraph.draw()
    
    axTorqGraph.clear()
    axTorqGraph.set_xlabel('Time (s)', fontsize=12)
    axTorqGraph.set_ylabel('Torque (Nm)', fontsize=12)
    axTorqGraph.set_title("Torque vs Time", fontsize=12)
    axTorqGraph.set_ylim(0, None)
    axTorqGraph.set_xlim(0, None)
    canvasTorqGraph.draw()

def upgrade_labels():
    p1_display.configure(state="normal")
    p1_display.delete(0, "end")
    p1_display.insert(0, str(p1[-1])) 
    p1_display.configure(state="readonly")

    p2_display.configure(state="normal")
    p2_display.delete(0, "end")
    p2_display.insert(0, str(p1[-1])) 
    p2_display.configure(state="readonly")

    dp_display.configure(state="normal")
    dp_display.delete(0, "end")
    dp_display.insert(0, str(dp[-1])) 
    dp_display.configure(state="readonly")

    aV_display.configure(state="normal")
    aV_display.delete(0, "end")
    aV_display.insert(0, str(angularVelocity[-1])) 
    aV_display.configure(state="readonly")

    Torq_display.configure(state="normal")
    Torq_display.delete(0, "end")
    Torq_display.insert(0, str(torque[-1])) 
    Torq_display.configure(state="readonly")

################################################################
#                                                              #
#           SE CREA LA ESTRUCTURA DE LA INTERFAZ               #
#                                                              #
################################################################

# Inicializar ventana
root = customtkinter.CTk()
root.title("Calibration System")
root.geometry("1250x700")

# Main frame for calibration
MainFrame = customtkinter.CTkFrame(root)
MainFrame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
MainFrame.grid_rowconfigure(0, weight=5)
MainFrame.grid_rowconfigure(1, weight=1)
MainFrame.grid_rowconfigure(2, weight=1)
MainFrame.grid_columnconfigure(0, weight=1)

#Graph frame
GraphFrame = customtkinter.CTkFrame(master=MainFrame)
GraphFrame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
GraphFrame.grid_rowconfigure(0, weight=1)
GraphFrame.grid_columnconfigure(0, weight=1)
GraphFrame.grid_columnconfigure(1, weight=1)
GraphFrame.grid_columnconfigure(2, weight=1)

# Data Frame
DataFrame = customtkinter.CTkFrame(master=MainFrame)
DataFrame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
DataFrame.grid_columnconfigure(0, weight=1)
DataFrame.grid_columnconfigure(1, weight=1)
DataFrame.grid_columnconfigure(2, weight=1)

# Buttom Frame
ButtonFrame = customtkinter.CTkFrame(master=MainFrame)
ButtonFrame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
ButtonFrame.grid_columnconfigure(0, weight=1)
ButtonFrame.grid_columnconfigure(1, weight=1)
ButtonFrame.grid_columnconfigure(2, weight=1)

# Furation Frame
stopFrame = customtkinter.CTkFrame(master=ButtonFrame)
stopFrame.grid(row=0, column=2, padx=1, pady=1, sticky="nsew")
stopFrame.grid_rowconfigure(0, weight=1)
stopFrame.grid_rowconfigure(1, weight=1)

# RESIZE
root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=1)

################################################################
#                                                              #
#     SE CREAN LAS GRÁFICAS Y SE MANDAN A ACTUALIZAR           #
#                                                              #
################################################################

########################## GRAPHS #############################
# Pressure difference
PresGraph = Figure(figsize=(6, 4), dpi=100)
axPresGraph = PresGraph.add_subplot(111)
axPresGraph.set_xlabel('Time (s)')
canvasPresGraph = FigureCanvasTkAgg(PresGraph, master=GraphFrame)
canvasPresGraph.get_tk_widget().grid(row = 0, column = 0, padx = 2, pady = 2)

# Angular velocity
AngularGraph = Figure(figsize=(6, 4), dpi=100)
axAngularGraph = AngularGraph.add_subplot(111)
axAngularGraph.set_xlabel('Time (s)')
canvasAngularGraph = FigureCanvasTkAgg(AngularGraph, master=GraphFrame)
canvasAngularGraph.get_tk_widget().grid(row = 0, column = 1, padx = 2, pady = 2)

# Torquimeter Graph
TorqGraph = Figure(figsize=(6, 4), dpi=100)
axTorqGraph = TorqGraph.add_subplot(111)
axTorqGraph.set_xlabel('Time (s)')
canvasTorqGraph = FigureCanvasTkAgg(TorqGraph, master=GraphFrame)
canvasTorqGraph.get_tk_widget().grid(row = 0, column = 2, padx = 2, pady = 2)

# Botones
buttonStart = customtkinter.CTkButton(text="START TEST", command=eventStart, master=ButtonFrame, height=50)
buttonStart.grid(row = 0, column = 0, padx = 2, pady = 2, sticky="nsew")

buttonStop = customtkinter.CTkButton(text="STOP TEST", command=eventStop, master=ButtonFrame, height=50)
buttonStop.grid(row = 0, column = 1, padx = 2, pady = 2, sticky="nsew")

# Time Entry
stop_label = customtkinter.CTkLabel(stopFrame, text="Experiment Duration (min): ")
stop_label.grid(row = 0, column = 0, padx = 2, pady = 2, sticky="nsew")
stop_display = customtkinter.CTkEntry(stopFrame, justify = "center", textvariable = customtkinter.StringVar(stopFrame, "1"))
stop_display.grid(row = 1, column = 0, padx=2, pady=2)

# Labels
dp_label = customtkinter.CTkLabel(DataFrame, text="Pressure Difference (psi): ")
dp_label.grid(row = 0, column = 0, padx = 2, pady = 0, sticky="w")
dp_display = customtkinter.CTkEntry(DataFrame, justify="left")
dp_display.grid(row=1, column=0, padx=10, pady=2)

p1_label = customtkinter.CTkLabel(DataFrame, text="Pressure 1 (psi): ")
p1_label.grid(row = 2, column = 0, padx = 2, pady = 0, sticky="w")
p1_display = customtkinter.CTkEntry(DataFrame, justify="left")
p1_display.grid(row=3, column=0, padx=10, pady=2)

p2_label = customtkinter.CTkLabel(DataFrame, text="Pressure 2 (psi): ")
p2_label.grid(row = 4, column = 0, padx = 2, pady = 0, sticky="w")
p2_display = customtkinter.CTkEntry(DataFrame, justify="left")
p2_display.grid(row=5, column=0, padx=10, pady=2)

aV_label = customtkinter.CTkLabel(DataFrame, text="Angular Velocity (rad/s): ")
aV_label.grid(row = 0, column = 1, padx = 2, pady = 0, sticky="w")
aV_display = customtkinter.CTkEntry(DataFrame, justify="left")
aV_display.grid(row=1, column=1, padx=10, pady=2)

Torq_label = customtkinter.CTkLabel(DataFrame, text="Torque (Nm): ")
Torq_label.grid(row = 0, column = 2, padx = 2, pady = 0, sticky="w")
Torq_display = customtkinter.CTkEntry(DataFrame, justify="left")
Torq_display.grid(row = 1, column = 2, padx=10, pady=2)

# Se mandan a crear y a limpiar las gráficas
update_graphs()
clear_graphs()

################################################################
#                                                              #
#           SE INICIALIZA LA INTERFAZ GRÁFICA                  #
#                                                              #
################################################################

# Crear y empezar el hilo para leer datos seriales
read_serial_data()

root.mainloop()