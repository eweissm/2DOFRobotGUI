import serial
import tkinter as tk
import time
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from scipy.optimize import fsolve

style.use('ggplot') #matplotlib settings

# caclulate the angles using inverse kinematics
def calculate_angles(x, y):
    global theta1 #angle of joint 1
    global theta2 #angle of joint 2
    global theta0
    theta0 = [0, 0]
    [theta1,theta2] = fsolve()


#when update button is pressed--> take entered coordinates and caclulate new coordinates, then update graph, then send to serial
def set_coordinates_state():
    global x_coord
    global y_coord
    x_coord = x_coord_entry.get()
    y_coord = y_coord_entry.get()
    calculate_angles(x_coord, y_coord)

def func(angles, x, y):
    global L1
    global L2
    L1 = 10
    L2 = 10
    return [L1*np.cos(angles[0])+L2*((np.cos(angles[1])*np.cos(angles[0])-np.sin(angles[1])*np.sin(angles[0])))-x,
            L1*np.sin(angles[0])+L2*((np.cos(angles[1])*np.sin(angles[0])+np.sin(angles[1])*np.cos(angles[0])))-y]
#set up serial comms
#ser = serial.Serial('com5', 9600) #create Serial Object
#time.sleep(3) #delay 3 seconds to allow serial com to get established

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