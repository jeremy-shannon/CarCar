#notes:
#PWM output ~3.2V

import pygame
from pygame.locals import *
import sys
from time import sleep

import RPi.GPIO as io
io.setwarnings(False)
io.setmode(io.BCM)

#initialize GPIO
io.setup(13,io.OUT)
io.setup(18,io.OUT)


io.output(18,io.HIGH)


#set GPIO to PWM mode (GPIO,frequency Hz)
acceleration = io.PWM(13,100)
steering = io.PWM(18,100)

correction = -0.4
step_size = 0.5
resting_state = 15 + correction
minimum = 10 + correction
maximum = 20 + correction
current_acceleration = resting_state
current_steering = resting_state

initialization_time = 0.0021

#An initialization sequence and calibration is needed. We can kind of get away with just sleeping. But it might be slightly more complex than that.
#2.5 milliseconds
acceleration.start(maximum)
sleep(initialization_time)
acceleration.ChangeDutyCycle(minimum)
sleep(initialization_time)
acceleration.ChangeDutyCycle(resting_state)
sleep(initialization_time)
print("Let's go!")

#set the duty cycle (%)
#acceleration.start(current_acceleration)
steering.start(current_steering)

pygame.init()
window = pygame.display.set_mode((640,480),1,16)

def increment(motion_variable):
    if motion_variable >= maximum:
        return maximum 
    else:
        return motion_variable + step_size
    return
    
def decrement(motion_variable):
    if motion_variable <= minimum:
        return minimum
    else:
        return motion_variable - step_size
    return

while 1:
    #set_repeat from pygame for holding a button down
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                print("Exiting")
                acceleration.ChangeDutyCycle(resting_state)
                steering.ChangeDutyCycle(resting_state)
                acceleration.stop()
                steering.stop()
                pygame.quit()
                io.cleanup()
                sys.exit()
            if event.key == pygame.K_UP:
                current_acceleration = increment(current_acceleration)
                acceleration.ChangeDutyCycle(current_acceleration)
                print("Acceleration increase. New duty cycle:",current_acceleration,"%")
            if event.key == pygame.K_DOWN:
                current_acceleration = decrement(current_acceleration)
                acceleration.ChangeDutyCycle(current_acceleration)
                print("Acceleration decrease. New duty cycle:",current_acceleration,"%")
            if event.key == pygame.K_RIGHT:
                current_steering = increment(current_steering)
                steering.ChangeDutyCycle(current_steering)
                print("Steering right. New duty cycle:",current_steering,"%")
            if event.key == pygame.K_LEFT:
                current_steering = decrement(current_steering)
                steering.ChangeDutyCycle(current_steering)
                print("Steering left. New duty cycle:",current_steering,"%")
            if event.key == pygame.K_7:
                current_acceleration = resting_state
                current_steering = resting_state
                acceleration.ChangeDutyCycle(resting_state)
                steering.ChangeDutyCycle(resting_state)
                print("Centering")
