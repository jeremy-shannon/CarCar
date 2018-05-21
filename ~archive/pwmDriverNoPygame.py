# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to Hz
pwm.set_pwm_freq(100)

def setPWM(accel, steer):
    pwm.set_pwm(0,0,accel)
    pwm.set_pwm(1,0,steer)

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

# try:
#     print("Let's go!")
#     while True:
#         #set_repeat from pygame for holding a button down
#         for event in pygame.event.get():
#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_ESCAPE:
#                     print("Exiting")
#                     currAccel = currSteer = nomPWM
#                     setPWM()
#                     pygame.quit()
#                 if event.key == pygame.K_UP:
#                     currAccel = increase(currAccel,"currAccel")
#                 if event.key == pygame.K_DOWN:
#                     currAccel = decrease(currAccel,"currAccel")
#                 if event.key == pygame.K_RIGHT:
#                     currSteer = increase(currSteer,"currSteer")
#                 if event.key == pygame.K_LEFT:
#                     currSteer = decrease(currSteer,"currSteer")
#                 if event.key == pygame.K_7:
#                     currAccel = currSteer = nomPWM
#                     print("Centering")
#         setPWM()
# except KeyboardInterrupt:
#     print("Exiting")
#     currAccel = currSteer = nomPWM
#     setPWM()
#     pygame.quit()

def main():

    # Configure PWM. Divide by 4096
    minPWM = 410  #10%
    maxPWM = 820  #20%
    nomPWM = 648  #15%
    currAccel = currSteer = nomPWM
    stepSize = 1

    accSet = 690
    steerMax = 780
    steerMin = 480

    for i in range(nomPWM,accSet):
        currAccel = i
        setPWM(currAccel, currSteer)
        print("cur Acc: ", i)
        time.sleep(.01)

    time.sleep(1.8)

    for i in range(nomPWM,steerMin):
        currSteer = i
        setPWM(currAccel, currSteer)
        print("cur Steer: ", i)
        time.sleep(.01)
        
    # for i in reversed(range(steerMin,steerMax)):
    #     currSteer = i
    #     setPWM(currAccel, currSteer)
    #     print("cur Steer: ", i)
    #     time.sleep(.01)

    print("### cur steer: ", currSteer)
    for i in range(35):
        setPWM(currAccel, currSteer)
        time.sleep(0.1)

    for i in range(steerMin,nomPWM):
        currSteer = i
        setPWM(currAccel, currSteer)
        print("cur Steer: ", i)
        time.sleep(.01)

    time.sleep(2.5)

    for i in reversed(range(nomPWM,accSet)):
        currAccel = i
        setPWM(currAccel, currSteer)
        print("cur Acc: ", i)
        time.sleep(.01)

if __name__ == '__main__':
    main()