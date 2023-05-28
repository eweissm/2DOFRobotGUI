import serial
import tkinter as tk
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib import pyplot as plt
import numpy as np
import scipy.optimize

# caclulate the angles using inverse kinematics
def calculate_angles(x, y, L1, L2):
    global theta0
    theta0 = [0,0]
    #scipy.optimize.fsolve(func, theta0, args=(tuple((x, y, L1, L2))))
    limits = scipy.optimize.Bounds(lb=[0, 0], ub=[np.pi, np.pi], keep_feasible=False)
    solution= scipy.optimize.minimize(func, theta0, args=(tuple((x, y, L1, L2))), method=None, jac=None, hess=None, hessp = None, bounds=limits, constraints=None, tol=None, callback=None, options=None )
    return solution.x

#generate and plot the graph
def plot(x_coord, y_coord, theta, L1, L2):
    # the figure that will contain the plot
    fig = Figure(figsize=(5, 5), dpi=100)

    # adding the subplot
    plot1 = fig.add_subplot(111)

    #set limits for graphs
    plot1.set_xlim([-(L1+L2), (L1+L2)])
    plot1.set_ylim([-(L1+L2), (L1+L2)])
    plot1.grid()

    # plotting the graph
    plot1.plot(0, 0, marker="o", markersize=20)
    plot1.plot(x_coord, y_coord, marker="o", markersize=10 )
    plot1.plot(L1*np.cos(theta[0]), L1*np.sin(theta[0]), marker="o", markersize=20)
    plot1.plot([0, L1*np.cos(theta[0])], [0, L1*np.sin(theta[0])])
    plot1.plot([L1 * np.cos(theta[0]),x_coord], [L1 * np.sin(theta[0]), y_coord])
    # creating the Tkinter canvas
    # containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master=RightFrame)
    canvas.draw()

    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().place(relx = 0, rely = 0)

#when update button is pressed--> take entered coordinates and caclulate new coordinates, then update graph, then send to serial
def set_coordinates_state():
    global x_coord
    global y_coord
    global theta #angles of joints 1 and 2
    global L1
    global L2
    #define arm lengths
    L1 = 10
    L2 = 10

    #get the inputs
    x_coord = float(x_coord_entry.get())
    y_coord = float(y_coord_entry.get())

    #perform inverse Kinematics calculation
    theta = calculate_angles(x_coord, y_coord, L1, L2)
    print(theta * 180 / np.pi)
    #generate and plot the graph
    plot(x_coord, y_coord, theta, L1, L2)
    theta1_deg = int(theta[0] * 180 / np.pi)
    theta2_deg = int(theta[1] * 180 / np.pi)

    #send serial data to arduino

    ser.write(bytes( str(theta1_deg), 'UTF-8'))
    ser.write(bytes('A', 'UTF-8'))
    ser.write(bytes( str(theta2_deg), 'UTF-8'))
    ser.write(bytes('B', 'UTF-8'))

def func(angles, x, y, L1, L2):
    #return [L1*np.cos(angles[0])+L2*(np.cos(angles[1])*np.cos(angles[0])-np.sin(angles[1])*np.sin(angles[0]))-x, L1*np.sin(angles[0])+L2*(np.cos(angles[1])*np.sin(angles[0])+np.sin(angles[1])*np.cos(angles[0]))-y]
    return np.sqrt((L1 * np.cos(angles[0]) + L2 * (np.cos(angles[1]) * np.cos(angles[0]) - np.sin(angles[1]) * np.sin(angles[0])) - x)**2 +
            (L1 * np.sin(angles[0]) + L2 * (np.cos(angles[1]) * np.sin(angles[0]) + np.sin(angles[1]) * np.cos(angles[0])) - y)**2)

#theta = [0,0]

#set up serial comms--------------------------------------------------------------------------------------------------------------------------------------------------
ser = serial.Serial('com4', 9600) #create Serial Object
time.sleep(3) #delay 3 seconds to allow serial com to get established

# Build GUI------------------------------------------------------------------------------------------------------------------------------------------------------------
tkTop = tk.Tk()  # Create GUI Box
tkTop.geometry('1200x800')  # size of GUI
tkTop.title("2 DOF GUI")  # title in top left of window

Title = tk.Label(text='Enter the desired coordinates of the 2 DOF arm', font=("Courier", 14, 'bold')).pack()  # Title on top middle of screen

# Fill in the left Side----------------------------------------------------------------------------------------------------------------------------------------
leftFrame = tk.Frame(master=tkTop, width=600) # create frame for the entry controls

leftFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)

TextFrame = tk.Frame(master=leftFrame, width=100)
x_input_Lable = tk.Label(master=TextFrame, text='X Coordinate:',
                                 font=("Courier", 12, 'bold')).pack(side='top', ipadx=0, padx=0, pady=0)
y_input_Lable = tk.Label(master=TextFrame, text='Y Coordinate:',
                                 font=("Courier", 12, 'bold')).pack(side='top', ipadx=0, padx=0, pady=0)

EntryFrame = tk.Frame(master=leftFrame, width=100)

x_coord_entry = tk.Entry(EntryFrame)
x_coord_entry.pack(side='top', ipadx=0, padx=0, pady=0)

y_coord_entry = tk.Entry(EntryFrame)
y_coord_entry.pack(side='top', ipadx=0, padx=0, pady=0)

#set intial coords to zero.
x_coord_entry.insert(0,0)
y_coord_entry.insert(0,0)

UpdateCoordsButton = tk.Button(EntryFrame,
                                   text="Update Coordinates",
                                   command=set_coordinates_state,
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
UpdateCoordsButton.pack(side='top', ipadx=10, padx=10, pady=40)

TextFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)
EntryFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)

# Fill in the right Side of GUI----------------------------------------------------------------------------------------------------------------------------------------
RightFrame = tk.Frame(master=tkTop, width=600, bg="gray")


RightFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)


tk.mainloop() # run loop watching for gui interactions