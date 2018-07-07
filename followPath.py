import csv
#from HarCar import HarCar
from HarCarThreaded import HarCar
import serial
import threading
import time
from math import atan2, sqrt, pi

exitFlag = 0

waypoints = []
currentWaypointIndex = 0
curXPos, curYPos, prevXPos, prevYPos = (None, None, None, None)
heading, speed = (None, None)
with open('Figure8_spaced30.csv') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    for row in readCSV:
        waypoints.append([float(row[0]), float(row[1])])
#with open('./logs/'+time.strftime("%Y-%m-%d-%H-%M")+'_LOG') as logfile:


car = HarCar()


class myThread (threading.Thread):
   def __init__(self):
        threading.Thread.__init__(self)
   def run(self):
        read_location()

def read_location():
    global curXPos, curYPos, prevXPos, prevYPos, heading
    print("Begin reading")
    ser = serial.Serial('/dev/ttyACM0')
    ser.flushInput()
    while True:        
        ser_bytes = str(ser.readline())
        data = ser_bytes.split()
        prevXPos = curXPos
        prevYPos = curYPos
        curXPos = float(data[3])
        curYPos = float(data[2])
        #print("current x,y: ", curXPos, curYPos)
        if (curXPos and curYPos and prevXPos and prevYPos):
            xDiff, yDiff = (curXPos - prevXPos, curYPos - prevYPos)
            heading = atan2(yDiff, xDiff)
            #print("current heading: ", heading)
        # if exitFlag:
        #     threadName.exit()

def dist(x1, y1, x2, y2):
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

# Create  thread
location_read_thread = myThread()
# Start Thread
location_read_thread.start()

car.set_speed(0.15)
time.sleep(3)
iteration = 0
try:
    while currentWaypointIndex < len(waypoints):
        waypointX, waypointY = (waypoints[currentWaypointIndex][1], waypoints[currentWaypointIndex][0])
        if curXPos is None or curYPos is None or heading is None or \
        (curXPos == prevXPos and curYPos == prevYPos):  # THIS IS NOT CORRECT!!!!
            continue
        if iteration % 5 != 0:
            iteration += 1
            continue
        print(".", end="")
        distToCurrentWaypoint = dist(curXPos, curYPos, waypointX, waypointY)
        if distToCurrentWaypoint < 0.00003:
            print("*****************Waypoint ", currentWaypointIndex," reached!********************")
            currentWaypointIndex += 1
            continue
        xDiffToWaypoint, yDiffToWaypoint = (waypointX - curXPos, waypointY - curYPos)
        headingToWaypoint = atan2(yDiffToWaypoint, xDiffToWaypoint)
        #print("heading to waypoint: ", headingToWaypoint)
        headingDiff = heading - headingToWaypoint
        if headingDiff < -pi:
            headingDiff += 2*pi
        if headingDiff > pi:
            headingDiff -= 2*pi
        steerValue = headingDiff/(pi/4)
        car.set_steer(steerValue)
        if iteration % 10 == 0:
            #print(chr(27) + "[2J")
            print("Distance to waypoint ", currentWaypointIndex, ": ", distToCurrentWaypoint)
            print("heading difference: ", headingDiff)
            print("steerValue: ", steerValue)
        iteration += 1

except KeyboardInterrupt:
    car.set_steer()
    car.set_speed()

car.set_steer()
car.set_speed()