# HarCar!

## Harman Self-Driving Car - Mini Version

---

This code is for the HarCar RC car prototype platform built on the Traxxas Fiesta Rally 1/10 scale RC car. The current hardware configuration includes:
- Rev Spark motor driver
- Raspberry Pi
- Adafruit PCA9685 PWM driver module
- Reach RTK precision GPS unit

Future hardware will include:
- Nvidia Jetson
- Zed stereo camera
- (other hardware TBD)

### HarCar.py

Includes PWM driver logic for smooth control of car. Abstracts PWM commands and exposes simple methods `set_speed` and `set_steer`, each of which accepts a `pct_speed` or `pct_steer` integer percent value argument from -100 to +100 representing the spectrum from full-speed reverse to full-speed forward, and full-left and full-right turns, respectively. The percent value is optional and if not specified will set speed/steer to zero. Each method also accepts an optional `rate` argument which defaults to the (mostly empirically-determined) maximum rate of change in speed/steering. A reduced rate can be specified for smoother motion.