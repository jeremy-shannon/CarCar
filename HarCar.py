import time
import Adafruit_PCA9685

# Configure PWM constants. For PCA9685 pulses are 0 to 4096
# PWM min (10%) = 410, max (20%) = 820, nominal (15%) = 615
# these values are not true to life
ZERO_STEER = 648
ZERO_SPEED = 640
MAX_LEFT = 470
MAX_RIGHT = 790
MIN_REVERSE = 605
MAX_REVERSE = 410
MIN_FORWARD = 675
MAX_FORWARD = 820

MAX_ACCEL = 38   #PWM steps per second
MAX_STEER_RATE = 500 

class HarCar:

    def __init__(self):             
        
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(100)
            
        self.speed = ZERO_SPEED
        self.steer = ZERO_STEER

        self.set_speed()
        self.set_steer()

    def set_speed(self, speed_val=0.0, rate=MAX_ACCEL):
        # speed_val should be in range [-1,1], -1 = full reverse, 1 = full forward
        if speed_val > 1.0:
            speed_val = 1.0
        elif speed_val < -1.0:
            speed_val = -1.0
        pwm_val = ZERO_SPEED
        if speed_val < 0:
            pwm_val = int(MIN_REVERSE + float(speed_val) * float(MIN_REVERSE - MAX_REVERSE))
        elif speed_val > 0:
            pwm_val = int(MIN_FORWARD + float(speed_val) * float(MAX_FORWARD - MIN_FORWARD))
        else:
            pwm_val = ZERO_SPEED            
        step = 1
        if self.speed > pwm_val:
            step = -1
        #print("current, set speed:", self.speed, pwm_val)
        for i in range(self.speed, pwm_val, step):
            self.speed = i
            #print("speed:", i)
            self.pwm.set_pwm(0,0,self.speed)
            if i > MIN_REVERSE and i < MIN_FORWARD:
                # blow right on through the deadband - NO SLEEPING IN THE DEADBAND!!
                continue
            time.sleep(1. / rate)

    def set_steer(self, steer_val=0.0, rate=MAX_STEER_RATE):
        # steer_val should be in range [-1,1], -1 = full left, 1 = full right
        if steer_val > 1.0:
            steer_val = 1.0
        elif steer_val < -1.0:
            steer_val = -1.0
        pwm_val = int(ZERO_STEER + float(steer_val) * float(MAX_RIGHT - ZERO_STEER))
        step = 1
        if self.steer > pwm_val:
            step = -1
        print("current, set steer:", self.steer, pwm_val)
        for i in range(self.steer, pwm_val, step):
            self.steer = i
            self.pwm.set_pwm(1,0,self.steer)
            time.sleep(1. / rate)

if __name__ == '__main__':
    car = HarCar()
    car.set_speed(.05)
    time.sleep(1)
    car.set_speed(-.05)
    
    # shut it down
    car.set_steer()
    car.set_speed()