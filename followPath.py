import csv
from HarCar import HarCar
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

car.set_speed(25)
iteration = 0
try:
    while currentWaypointIndex < len(waypoints):
        waypointX, waypointY = (waypoints[currentWaypointIndex][1], waypoints[currentWaypointIndex][0])
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
        steerValue = headingDiff/(pi/2)*100
        car.set_steer(steerValue)
        if iteration % 10 == 0:
            print("Distance to waypoint: ", distToCurrentWaypoint)
            print("heading difference: ", headingDiff)
            print("steerValue: ", steerValue)
        iteration += 1

except KeyboardInterrupt:
    car.set_steer()
    car.set_speed()

car.set_steer()
car.set_speed()