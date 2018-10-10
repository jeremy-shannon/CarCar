import time
# NOTE: PWM disable
import Adafruit_PCA9685
from threading import Thread

# Configure PWM constants. For PCA9685 pulses are 0 to 4096
# PWM min (10%) = 410, max (20%) = 820, nominal (15%) = 615
# these values are not true to life
ZERO_STEER = 652
ZERO_SPEED = 640
MAX_LEFT = 470
MAX_RIGHT = 790
MIN_REVERSE = 605
MAX_REVERSE = 410
MIN_FORWARD = 675
MAX_FORWARD = 820

MAX_ACCEL = 38   #PWM steps per second
MAX_STEER_RATE = 2000000 

SPEED_LIMIT = 2.0  # m/s
STEER_LIMIT = 0.35 # radians

class HarCar:

    def __init__(self):             
        # Initialise the PCA9685 using the default address (0x40).
        # NOTE: PWM disable
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(100)
            
        self.currentSpeed = ZERO_SPEED
        self.currentSteer = ZERO_STEER
        self.targetSpeed = ZERO_SPEED
        self.targetSteer = ZERO_STEER

        self.speedRate = MAX_ACCEL
        self.steerRate = MAX_STEER_RATE

        self.shutDownThreads = False

        Thread(target=self.adjust_speed_thread, args=(0,)).start()
        Thread(target=self.adjust_steer_thread, args=(0,)).start()

        self.set_speed()
        self.set_steer()

    def shut_down(self):
        print('***************Shutting down HarCar*****************')
        car.set_steer()
        car.set_speed()
        time.sleep(2)
        self.shutDownThreads = True
    
    def get_speed_and_steer(self):
        # invert these following equations, to return current speed/steer in m/s and radians, respectively
        #pwm_val = int((26.472 * speed_val) + 680.45)
        #pwm_val = int((-411.96 * steer_val) + 652.51)
        current_speed = (float(self.currentSpeed) - 680.45) / 26.472  
        current_steer = (float(self.currentSteer) - 652.51) / -411.96
        return (current_speed, current_steer)

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
        self.targetSpeed = pwm_val
        self.speedRate = rate
        #print("current, target speed:", self.currentSpeed, self.targetSpeed)
    
    def adjust_speed_thread(self, _):
        while not self.shutDownThreads:
            if self.targetSpeed == self.currentSpeed:
                pass
                #continue
            if self.targetSpeed > self.currentSpeed:
                self.currentSpeed += 1
            elif self.targetSpeed < self.currentSpeed:
                self.currentSpeed -= 1
            # NOTE: PWM disable
            print('set speed pwm: ' + str(self.currentSpeed))
            self.pwm.set_pwm(0,0,self.currentSpeed)
            if self.currentSpeed > MIN_REVERSE and self.currentSpeed < MIN_FORWARD:
                # blow right on through the deadband - NO SLEEPING IN THE DEADBAND!!
                continue
            time.sleep(1. / self.speedRate)

    def set_steer(self, steer_val=0.0, rate=MAX_STEER_RATE):
        # steer_val should be in radians positive = left, negative = right
        if steer_val > STEER_LIMIT:
            steer_val = STEER_LIMIT
        elif steer_val < -STEER_LIMIT:
            steer_val = -STEER_LIMIT
        #Magic formula to convert streeing angle(radians) to corresponding pwm value
        pwm_val = int((-411.96 * steer_val) + 652.51)
        self.targetSteer = pwm_val
        self.steerRate = rate
        #print("current, target steer:", self.currentSteer, self.targetSteer)

    def adjust_steer_thread(self, _):
        while not self.shutDownThreads:
            if self.targetSteer == self.currentSteer:
                pass
                #continue
            if self.targetSteer > self.currentSteer:
                self.currentSteer += 1
            elif self.targetSteer < self.currentSteer:
                self.currentSteer -= 1
            # NOTE: PWM disable
            print('set steer pwm: ' + str(self.currentSteer))
            self.pwm.set_pwm(1,0,self.currentSteer)
            time.sleep(1. / self.steerRate)

if __name__ == '__main__':
    try:
        car = HarCar()

        car.set_speed(.5)
        time.sleep(3)
        car.set_steer(.3)
        time.sleep(3)

        # shut it down
        car.shut_down()   
    
    except KeyboardInterrupt:
        car.shut_down()
