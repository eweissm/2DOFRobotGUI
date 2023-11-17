import serial
import tkinter as tk
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib import pyplot as plt
import numpy as np
import scipy.optimize
import progressbar

L1 = 10
L2 = 10
# calculate the angles using inverse kinematics
#theta0 = [np.pi/2, np.pi/2]
theta0 = [0, 0]
prevTheta1 = 0
def calculate_angles(x, y, L1, L2):
    global theta0
    global ErrorFlag
    global counter
    global prevTheta1 # previous solution to theta1

    counter=0
    ErrorFlag= True

    while(ErrorFlag and counter<5):

        solution,infodict, ier, msg =scipy.optimize.fsolve(func, theta0, args=(tuple((x, y, L1, L2))), full_output=1)

        #normalize angles
        #solution[0] = NormalizeAngle(solution[0])
        solution[1] = NormalizeAngle(solution[1])

        if solution[1] > 3*np.pi/2 or solution[1] < np.pi/2 or ier != 1 or abs(solution[0]-prevTheta1) > 2*np.pi:
            counter = counter + 1
            rng = np.random.default_rng(12345)
            rFloat = np.pi*(rng.random(2)-.5)
            theta0= [theta0[0]+rFloat[0], theta0[1]+rFloat[1]]
        else:
            ErrorFlag = False
            prevTheta1 = solution[0]

        #print(theta0)

    if ErrorFlag:
        print("Error: Angles out of bounds")
        theta0 = [np.pi / 2, np.pi / 2]
    else:
        theta0 = solution

    return solution

#generate and plot the graph
def plot(x_coord, y_coord, theta, L1, L2):
    global pathX
    global pathY

    # the figure that will contain the plot
    fig = Figure(figsize=(8, 8), dpi=100)

    # adding the subplot
    plot1 = fig.add_subplot(111)

    #set limits for graphs
    plot1.set_xlim([-(L1+L2+2), (L1+L2+2)])
    plot1.set_ylim([-(L1+L2+2), (L1+L2+2)])
    plot1.grid()

    #plotting work space
    x, y = generate_semicircle(0, 0, np.sqrt(L1**2+L2**2), 0.01)
    plot1.plot(x, y, color='black', linestyle='dashed')
    plot1.plot(-x, y, color='black', linestyle='dashed')


    #plotting path
    plot1.plot(pathX, pathY, color='blue', linestyle='dashed')

    # plotting the arm
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

def startupPlot(L1, L2):
    global pathX
    global pathY

    # the figure that will contain the plot
    fig = Figure(figsize=(8, 8), dpi=100)

    # adding the subplot
    plot1 = fig.add_subplot(111)

    # set limits for graphs
    plot1.set_xlim([-(L1 + L2 + 2), (L1 + L2 + 2)])
    plot1.set_ylim([-(L1 + L2 + 2), (L1 + L2 + 2)])
    plot1.grid()

    # plotting work space
    x, y = generate_semicircle(0, 0, np.sqrt(L1**2+L2**2), 0.01)
    plot1.plot(x, y, color='black', linestyle='dashed')
    plot1.plot(-x, y, color='black', linestyle='dashed')

    # plotting path
    plot1.plot(pathX, pathY, color='blue', linestyle='dashed')

    # creating the Tkinter canvas
    # containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master=RightFrame)
    canvas.draw()

    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().place(relx=0, rely=0)

#normalize angle between 0 and 2*pi
def NormalizeAngle(angle):
    if angle > 2*np.pi:
        solution = angle - abs(np.floor(angle / (2*np.pi)) * 2 * np.pi)
    elif angle < 0:
        solution = angle + abs(np.floor(angle / (2*np.pi)) * 2 * np.pi)
    else:
        solution = angle
    return solution

def generate_semicircle(center_x, center_y, radius, stepsize=0.1):
    """
    generates coordinates for a semicircle, centered at center_x, center_y
    """
    x = np.arange(center_x, center_x+radius+stepsize, stepsize)
    y = np.sqrt(radius**2 - x**2)

    # since each x value has two corresponding y-values, duplicate x-axis.
    # [::-1] is required to have the correct order of elements for plt.plot.
    x = np.concatenate([x,x[::-1]])

    # concatenate y and flipped y.
    y = np.concatenate([y,-y[::-1]])

    return x, y + center_y

def StartPathFollow():
    global pathX
    global pathY

    set_coordinates_state(pathX, pathY)

#when update button is pressed--> take entered coordinates and caclulate new coordinates, then update graph, then send to serial
def set_coordinates_state(x_coord, y_coord):
    try:
        NumEntries= len(x_coord)
    # handle list / array case
    except TypeError:
        NumEntries = 1
    # oops, was a float

    global theta #angles of joints 1 and 2
    global L1
    global L2
    global ErrorFlag
    #define arm lengths
    L1 = 10
    L2 = 10

    theta1_deg = np.zeros(NumEntries)
    theta2_deg = np.zeros(NumEntries)

    ser.reset_input_buffer()  # clear input buffer

    CheckSerialCounter = 3  # how many times we will check the serial before giving up

    #Pre-Calculate inverse Kinematics calculation
    print("Calculating Angles...")
    with progressbar.ProgressBar(max_value=NumEntries) as bar:
        for i in range(NumEntries):
            if NumEntries > 1:

                thisXCoord = x_coord[i]
                thisYCoord = y_coord[i]
            else:
                thisXCoord = x_coord
                thisYCoord = y_coord


            theta = calculate_angles(thisXCoord, thisYCoord, L1, L2)
            if not ErrorFlag:
                #print(theta * 180 / np.pi)

                #generate and plot the graph
                plot(thisXCoord, thisYCoord, theta, L1, L2)
                theta1_deg[i] = int(theta[0] * 180 / np.pi)
                theta2_deg[i] = int(theta[1] * 180 / np.pi)
            else:
                print("Failed to Calculate Coordinate: "+str(i))
                break

            bar.update(i)

    print("Done Calculating Angles")

    # Run through the angles
    if not ErrorFlag:
        for i in range(NumEntries):

            start = time.time()

            #send serial data to arduino
            ser.write(bytes(str(theta1_deg[i]), 'UTF-8'))
            ser.write(bytes('A', 'UTF-8'))
            ser.write(bytes(str(theta2_deg[i]-90), 'UTF-8'))
            ser.write(bytes('B', 'UTF-8'))

            # get expected move time from arduino
            ExpectedTime_bytes = ser.readline()
            ExpectedTime_string = ExpectedTime_bytes.decode("utf-8")
            print("ExpectedTime: " + ExpectedTime_string)

            try:
                ExpectedTime = max(float(ExpectedTime_string), 0.1)  # convert expected time to float (minimum time is 0.005s)
            except ValueError:
                ExpectedTime = 0.1

            ser.reset_input_buffer()  # clear input buffer

            # if we get a 'y' from arduino, we move on, otherwise we will wait 0.5 sec. We will repeat this 5 times.
            # After which, if we still do not have confirmation, we will print to the monitor that there was a problem and
            # move on
            DidMoveWork = False
            counter = 0
            ArduinoMessage = ''

            time.sleep(ExpectedTime)  # wait for move to complete

            while counter < CheckSerialCounter and not DidMoveWork:
                if ser.inWaiting():
                    ArduinoMessage = ser.read(1)  # read one bit from buffer
                    print(ArduinoMessage)

                if ArduinoMessage == b'y':
                    DidMoveWork = True
                    print("Move was successful")
                else:
                    print("Waiting...")
                    time.sleep(ExpectedTime/2)
                    counter = counter + 1


            if not DidMoveWork:
                print("Move was not successful")

            ser.reset_input_buffer() #clear input buffer
            end = time.time()
            print("Difference between expected time and actual time: " + str(end- start - ExpectedTime))

def func(angles, x, y, L1, L2):
    return [L1*np.cos(angles[0])+L2*(np.cos(angles[1])*np.cos(angles[0])-np.sin(angles[1])*np.sin(angles[0]))-x, L1*np.sin(angles[0])+L2*(np.cos(angles[1])*np.sin(angles[0])+np.sin(angles[1])*np.cos(angles[0]))-y]
    #return np.sqrt((L1 * np.cos(angles[0]) + L2 * (np.cos(angles[1]) * np.cos(angles[0]) - np.sin(angles[1]) * np.sin(angles[0])) - x)**2 + (L1 * np.sin(angles[0]) + L2 * (np.cos(angles[1]) * np.sin(angles[0]) + np.sin(angles[1]) * np.cos(angles[0])) - y)**2)

#set path defaults
ActivePath = 0
pathX = [5, 5, 5, 5, 5, 5, 3, 1, -1, -3, -5, -5, -5, -5, -5, -5, -3, -1, 1, 3, 5]
pathY = [-5, -3, -1, 1, 3, 5, 5, 5, 5, 5, 5, 3, 1, -1, -3, -5, -5, -5, -5, -5, -5]
def ChangeSelectPathButton():
    global ActivePath
    global pathX
    global pathY
    global L1
    global L2

    numCases = 5

    if ActivePath >= numCases-1:
        ActivePath=0
    else:
        ActivePath=ActivePath+1

    match ActivePath:
        case 0: #rectangle
            pathX = [ 5,  5,  5, 5, 5, 5, 3, 1, -1, -3, -5 , -5 , -5, -5, -5 , -5, -3, -1, 1, 3, 5]
            pathY = [-5, -3, -1, 1, 3, 5, 5, 5 , 5,  5,  5,   3,   1, -1, -3,  -5, -5, -5,-5,-5,-5]
        case 1: #involute of circle
            u = np.linspace(0, 6.5 * np.pi, 150)
            c = .45
            pathX = (c * (np.cos(u) + u * np.sin(u)))
            pathY = c * (np.sin(u) - u * np.cos(u))
        case 2:  # Heart
            u = np.linspace(0,  2 * np.pi, 100)
            c = .3
            pathX = (6*c*np.sin(u))**3
            pathY = 13*c*np.cos(u)-5*c*np.cos(2*u)-2*c*np.cos(3*u)-c*np.cos(4*u)
        case 3:  # lemniscate
            u = np.linspace(0, 2 * np.pi, 50)
            c = 5
            pathX = (c * np.cos(u))
            pathY = c * np.sin(2 * u)
        case 4:  # Lisajous curves
            u = np.linspace(0, 15 * np.pi, 400)
            pathX = (8 * np.sin(u*.9))
            pathY = 8 * np.sin(u)
        case default: #rectangle
            pathX = [ 5,  5,  5, 5, 5, 5, 3, 1, -1, -3, -5 , -5 , -5, -5, -5 , -5, -3, -1, 1, 3, 5]
            pathY = [-5, -3, -1, 1, 3, 5, 5, 5 , 5,  5,  5,   3,   1, -1, -3,  -5, -5, -5,-5,-5,-5]

    startupPlot(L1, L2)

#set up serial comms--------------------------------------------------------------------------------------------------------------------------------------------------
ser = serial.Serial('com4', 9600, timeout=10) #create Serial Object, baud = 9600, read times out after 10s
time.sleep(3) #delay 3 seconds to allow serial com to get established

# Build GUI------------------------------------------------------------------------------------------------------------------------------------------------------------
tkTop = tk.Tk()  # Create GUI Box
tkTop.geometry('1200x800')  # size of GUI
tkTop.title("2 DOF GUI")  # title in top left of window

Title = tk.Label(text='Enter the desired coordinates of the 2 DOF arm', font=("Courier", 14, 'bold')).pack()  # Title on top middle of screen

# Fill in the left Side------------------------------------------------------------------------------------------------
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
                                   command=lambda:set_coordinates_state(float(x_coord_entry.get()),float(y_coord_entry.get())),
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
UpdateCoordsButton.pack(side='top', ipadx=10, padx=10, pady=40)

StartPathButton = tk.Button(EntryFrame,
                                   text="Follow Path",
                                   command=StartPathFollow,
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
StartPathButton.pack(side='top', ipadx=10, padx=10, pady=40)

PathSelectorButton = tk.Button(EntryFrame,
                                   text="Change Path",
                                   command=ChangeSelectPathButton,
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
PathSelectorButton.pack(side='top', ipadx=10, padx=10, pady=40)

TextFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)
EntryFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)

# Fill in the right Side of GUI----------------------------------------------------------------------------------------
RightFrame = tk.Frame(master=tkTop, width=600, bg="gray")


RightFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)
startupPlot(L1, L2)

tk.mainloop() # run loop watching for gui interactions