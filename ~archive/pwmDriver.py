# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import pygame
from pygame.locals import *

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure PWM. Divide by 4096
minPWM = 410  #10%
maxPWM = 820  #20%
nomPWM = 614  #15%
currAccel = currSteer = nomPWM
stepSize = 1

# Set frequency to Hz
pwm.set_pwm_freq(100)

pygame.init()
pygame.key.set_repeat(1,5)
window = pygame.display.set_mode((640,480),1,16)

def setPWM():
    pwm.set_pwm(0,0,currAccel)
    pwm.set_pwm(1,0,currSteer)

def increase(changeIn,name):
    if changeIn < maxPWM:
        changeIn += stepSize
        print(name,"increase:",changeIn)
    else:
        print("At Max. current:",changeIn,", maxPWM",maxPWM)
    return changeIn

def decrease(changeIn,name):
    if changeIn > minPWM:
        changeIn -= stepSize
        print(name,"decrease:",changeIn)
    else:
        print("At Min. current:",changeIn,", minPWM",minPWM)
    return changeIn

try:
    print("Let's go!")
    while True:
        #set_repeat from pygame for holding a button down
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("Exiting")
                    currAccel = currSteer = nomPWM
                    setPWM()
                    pygame.quit()
                if event.key == pygame.K_UP:
                    currAccel = increase(currAccel,"currAccel")
                if event.key == pygame.K_DOWN:
                    currAccel = decrease(currAccel,"currAccel")
                if event.key == pygame.K_RIGHT:
                    currSteer = increase(currSteer,"currSteer")
                if event.key == pygame.K_LEFT:
                    currSteer = decrease(currSteer,"currSteer")
                if event.key == pygame.K_7:
                    currAccel = currSteer = nomPWM
                    print("Centering")
        setPWM()
except KeyboardInterrupt:
    print("Exiting")
    currAccel = currSteer = nomPWM
    setPWM()
    pygame.quit()
