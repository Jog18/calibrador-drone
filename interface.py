##Mario Aguilera Ruiz
##CICESE 2024

##Code for PC Interface with quadcopter on ARMTOP platform

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import collections
import csv
from datetime import datetime

arm = 0
#excel file name to store data
filename = "results.csv"

def get_serialports():
    ports = serial.tools.list_ports.comports()
    lista_ports = [puerto.device for puerto in ports]
    return lista_ports

# connect to selected serial port
def conectar_puerto():
    selected_port = combo_ports.get()  
    if selected_port == "":
        messagebox.showwarning("Warning", "Please select a port")
    else:
        try:
            # try to open serial port
            global ser 
            ser = serial.Serial(selected_port, 115200, timeout=1)
            label_estatus.config(text=f"Estatus: Connected", fg="green")
        except Exception as e:
            messagebox.showerror("Error", f"Error: {e}")
            label_estatus.config(text="", fg="red")

# send slider values through serial port
def send_values():
    value1 = slider1.get()
    value2 = slider2.get()
    value3 = slider3.get()
    value4 = entry4.get()
    value5 = entry5.get()
    value6 = entry6.get()

    
    mensaje = f"{arm},{value1},{value2},{value3},{value4},{value5},{value6}\n"
    ser.write(mensaje.encode()) 
    print(f"Enviado: {mensaje.strip()}") 

#read incoming serial data
def read_serial():
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) >= 2:
                    data1 = float(values[0])
                    data2 = float(values[1])
                    data3 = slider2.get()
                    data4 = slider3.get()
                
                update_plot(data1, data2, data3, data4)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                save_to_file(timestamp,data1,data2,data3,data4)

        except Exception as e:
            print(f"Error: {e}")


def cambiar_boton():
    # Change button text
    global arm
    arm =1 if arm == 0 else 0

    if arm ==1:
        boton_1.config(text="Disarm")
    else:
        boton_1.config(text="Arm")
        slider1.set(0)
        slider2.set(0)
        slider3.set(0)

    send_values()



def update_plot(data1, data2, data3, data4):
    #update plot with received data
    global x_counter
    ydata1.append(data1)
    ydata2.append(data2)
    ydata3.append(data3)
    ydata4.append(data4)
    if len(ydata1) > 100: 
        ydata1.pop(0)
        ydata2.pop(0)
        ydata3.pop(0)
        ydata4.pop(0)

    line1.set_data(range(len(ydata1)), ydata1)
    line2.set_data(range(len(ydata2)), ydata2)
    line3.set_data(range(len(ydata3)), ydata3)
    line4.set_data(range(len(ydata4)), ydata4)

    ax.set_xlim(0, max(100, len(ydata1)))  # x axis limits
    canvas.draw()

#save data to csv file
def save_to_file(t,var1,var2,var3,var4):
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)

        file.seek(0,2)
        if file.tell()==0:
            writer.writerow(['time','roll','pitch','rolld','pitchd'])
        
        writer.writerow([t,var1,var2,var3,var4])

rolld = 0
pitchd = 0

#plot axis limits
Samples = 100
xmin = 0
xmax = Samples
ymin = -55
ymax = 55

fig = plt.figure(figsize=(8,4))    #create new figure
ax = plt.axes(xlim=(xmin,xmax), ylim=(ymin,ymax))
plt.title("Roll")
ax.set_xlabel("Muestras")
ax.set_ylabel("Grados")

ax.legend()

# create main window
ventana = tk.Tk()
ventana.title("Multi rotor testing platform")

fig, ax = plt.subplots(figsize=(8,4))
line1, = ax.plot([], [], lw=2, label="Roll") 
line2, = ax.plot([], [], lw=2, color="red", label="Pitch")  
line3, = ax.plot([], [], lw=2, color="blue", linestyle="--", label="Roll_d")
line4, = ax.plot([], [], lw=2, color="red", linestyle="--", label="Pitch_d")

ax.set_ylim(-60, 60)
ax.set_xlim(0, 100)
ax.plot()
ax.legend()
ydata1, ydata2, ydata3, ydata4 = [], [], [], []
x_counter = 0  

#create different frames for interface controls
Lframe=tk.Frame(ventana)
Lframe.pack(side="left",fill="x",expand=1)
Rframe=tk.Frame(ventana)
Rframe.pack(side="right")

frame1 = tk.Frame(Lframe)
frame1.pack(anchor=tk.NW, padx=30, pady=10,fill=None, expand=False)
frame1.config(relief="ridge")
frame1.config(bd=1)

frame2 = tk.Frame(Lframe)
frame2.pack(anchor=tk.SW, padx=30, pady=10, fill=None, expand=False)
frame2.config(relief="ridge")
frame2.config(bd=1)

canvas = FigureCanvasTkAgg(fig, master=Rframe)
canvas.get_tk_widget().grid(row=0, column=0, columnspan=3)


#Serial Port connection 
label = tk.Label(frame1, text="Select Port:")
label.pack(pady=5, padx=50)

combo_ports = ttk.Combobox(frame1, values=get_serialports(), state="readonly", width=7)
combo_ports.pack(pady=5, padx=15)

boton_conectar = tk.Button(frame1, text="Connect", command=conectar_puerto)
boton_conectar.pack(pady=5, padx=15)

label_estatus = tk.Label(frame1, text="", fg="red")
label_estatus.pack(pady=5)

#Drone control
label2 = tk.Label(frame2, text="Drone Control")
label2.pack(pady=5, padx=45)

boton_1 = ttk.Button(frame2, text="Arm", command=cambiar_boton)
boton_1.pack(padx=10, pady=10, side=tk.TOP)

# sliders configuration
slider1 = tk.Scale(frame2, from_=0, to=60, orient="horizontal", label="Throttle (%)", command=lambda x: send_values())
slider1.pack(padx=10, pady=10)

slider2 = tk.Scale(frame2, from_=-50, to=50, orient="horizontal", label="Roll (°)", command=lambda x: send_values())
slider2.pack(padx=10, pady=10)

slider3 = tk.Scale(frame2, from_=-50, to=50, orient="horizontal", label="Pitch (°)", command=lambda x: send_values())
slider3.pack(padx=10, pady=10)

#PID tuning
frame3 = tk.Frame(Lframe)
frame3.pack(anchor=tk.SW, padx=30, pady=10, fill=None, expand=False)

frame4 = tk.Frame(Lframe)
frame4.pack(anchor=tk.SW, padx=30, pady=10, fill=None, expand=False)

frame5 = tk.Frame(Lframe)
frame5.pack(anchor=tk.SW, padx=30, pady=10, fill=None, expand=False)

label3 = tk.Label(frame3, text="PID Control Tuning")
label3.pack(pady=5, padx=30)

def1=tk.StringVar(value=22)
def2=tk.StringVar(value=20)
def3=tk.StringVar(value=30)
label4 = tk.Label(frame3, text="Roll_P")
label4.pack(side="left",padx=10)
entry4=tk.Spinbox(frame3,from_=0, to=30, width=7, state="readonly", textvariable=def1, command=lambda: send_values())
entry4.pack(side="left", padx=5)

label5 = tk.Label(frame4, text="Roll_I")
label5.pack(side="left",padx=11)
entry5=tk.Spinbox(frame4,from_=0, to=30, width=7, state="readonly", textvariable=def2, command= lambda: send_values())
entry5.pack(side="left", padx=5)

label6 = tk.Label(frame5, text="Roll_D")
label6.pack(side="left",padx=9)
entry6=tk.Spinbox(frame5,from_=0, to=30, width=7, state="readonly", textvariable=def3, command= lambda: send_values())
entry6.pack(side="left", padx=5)

# start serial reading on a thread
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

# start graphical interface
ventana.mainloop()

ser.close()