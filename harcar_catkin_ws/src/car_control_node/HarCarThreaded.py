import time
# NOTE: PWM disable
import Adafruit_PCA9685
from threading import Thread

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
MAX_STEER_RATE = 2000 

SPEED_LIMIT = 2.0  # m/s
STEER_LIMIT = 0.35 # radians

class HarCar:

    def __init__(self):             
        
        # Initialise the PCA9685 using the default address (0x40).
        # NOTE: PWM disable
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(100)
            
        self.speed = ZERO_SPEED
        self.steer = ZERO_STEER

        # Flags for interrupting threads with new commands
        self.adjust_speed_thread_running = False
        self.request_speed_thread_interrupt = False
        self.adjust_steer_thread_running = False
        self.request_steer_thread_interrupt = False

        self.set_speed()
        self.set_steer()

    def set_speed(self, speed_val=0.0, rate=MAX_ACCEL):
        # limit speed
        if speed_val > SPEED_LIMIT:
            speed_val = SPEED_LIMIT
        elif speed_val < -SPEED_LIMIT:
            speed_val = -SPEED_LIMIT
        pwm_val = ZERO_SPEED
        if speed_val < 0:
            #pwm_val = int(MIN_REVERSE + float(speed_val) * float(MIN_REVERSE - MAX_REVERSE))
            #for now just dont worry about negative speed! just set it to zero
            pwm_val = ZERO_SPEED
        elif speed_val > 0:
            #Magic formula to convert speed(m/s) to corresponding pwm value
            pwm_val = int((26.472 * speed_val) + 680.45)
        else:
            pwm_val = ZERO_SPEED            
        step = 1
        if self.speed > pwm_val:
            step = -1
        #print("current, set speed:", self.speed, pwm_val)
        while self.adjust_speed_thread_running:
            self.request_speed_thread_interrupt = True
        self.request_speed_thread_interrupt = False
        Thread(target=self.adjust_speed_thread, args=(pwm_val, step, rate,)).start()
    
    def adjust_speed_thread(self, target, step, rate):
        self.adjust_speed_thread_running = True
        for i in range(self.speed, target, step):
            self.speed = i
            #print("speed:", i)
            # NOTE: PWM disable
            self.pwm.set_pwm(0,0,self.speed)
            if i > MIN_REVERSE and i < MIN_FORWARD:
                # blow right on through the deadband - NO SLEEPING IN THE DEADBAND!!
                continue
            if self.request_speed_thread_interrupt:
                #print("interrupting speed adjust")
                break
            time.sleep(1. / rate)
        self.adjust_speed_thread_running = False

    def set_steer(self, steer_val=0.0, rate=MAX_STEER_RATE):
        # steer_val should be in range [-1,1], -1 = full left, 1 = full right
        if steer_val > STEER_LIMIT:
            steer_val = STEER_LIMIT
        elif steer_val < -STEER_LIMIT:
            steer_val = -STEER_LIMIT
        #pwm_val = int(ZERO_STEER + float(steer_val) * float(MAX_RIGHT - ZERO_STEER))
        #Magic formula to convert streeing angle(radians) to corresponding pwm value
        pwm_val = int((411.96 * steer_val) + 652.51)
        step = 1
        if self.steer > pwm_val:
            step = -1
        #print("current, set steer:", self.steer, pwm_val)
        while self.adjust_steer_thread_running:
            self.request_steer_thread_interrupt = True
        self.request_steer_thread_interrupt = False
        Thread(target=self.adjust_steer_thread, args=(pwm_val, step, rate,)).start()

    def adjust_steer_thread(self, target, step, rate):
        self.adjust_steer_thread_running = True   
        for i in range(self.steer, target, step):
            self.steer = i
            #print("steer:", i)
            # NOTE: PWM disable
            self.pwm.set_pwm(1,0,self.steer)
            time.sleep(1. / rate)
            if self.request_steer_thread_interrupt:
                #print("interrupting steer adjust")
                break
        self.adjust_steer_thread_running = False

if __name__ == '__main__':
    car = HarCar()

    car.set_speed(.4)
    time.sleep(7)

    # shut it down
    car.set_steer()
    car.set_speed()
