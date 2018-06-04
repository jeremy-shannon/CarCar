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

    def set_speed(self, pct_speed=0, rate=MAX_ACCEL):
        # val should be in %, -100 = full reverse, 100 = full forward
        if pct_speed > 100:
            pct_speed = 100
        elif pct_speed < -100:
            pct_speed = -100
        pwm_val = ZERO_SPEED
        if pct_speed < 0:
            pwm_val = int(MIN_REVERSE + float(pct_speed)/100 * (MIN_REVERSE - MAX_REVERSE))
        elif pct_speed > 0:
            pwm_val = int(MIN_FORWARD + float(pct_speed)/100 * (MAX_FORWARD - MIN_FORWARD))
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

    def set_steer(self, pct_steer=0, rate=MAX_STEER_RATE):
        # val should be in %, -100 = full left, 100 = full right
        if pct_steer > 100:
            pct_steer = 100
        elif pct_steer < -100:
            pct_steer = -100
        pwm_val = int(ZERO_STEER + float(pct_steer) / 100 * (MAX_RIGHT - ZERO_STEER))
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
    car.set_speed(5)
    car.set_speed(-5)
    
    # shut it down
    car.set_steer()
    car.set_speed()