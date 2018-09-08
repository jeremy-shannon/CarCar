#https://engineersportal.com/blog/2018/2/25/python-datalogger-reading-the-serial-output-from-arduino-to-analyze-data-using-pyserial
#https://community.emlid.com/t/reach-view-2-2-5-usb-to-pc-solution-output/5168
import serial
ser = serial.Serial('/dev/ttyACM1')
ser.flushInput()

print("Begin reading")
while True:
    ser_bytes = ser.readline()
    print(ser_bytes)
