# EG2310 
# This code is used to control the servo 
# Students will be given this code but will need to understand how it works 
# Additional h/w exercise is to write a function to turn the servo by an angle 
# The servo motor may be chosen to tilt the payload 

import time 
import RPi.GPIO as GPIO 

# Set pin numbering convention 
GPIO.setmode(GPIO.BOARD) 

# Choose an appropriate pwm channel to be used to control the servo 
servo_pin = 32
 
# Set the pin as an output 
GPIO.setup(servo_pin, GPIO.OUT)
 
# Initialise the servo to be controlled by pwm with 50 Hz frequency 
p = GPIO.PWM(servo_pin, 50)
 
# Set servo to 90 degrees as it's starting position 
p.start(7.5)

try: 
  while True:
    n = int(input())
    ang = (n/180*10)+2.5
    p.ChangeDutyCycle(ang) #90 deg position 
except Keyboardlnterrupt: 
  p.stop() 
  GPIO.cleanup()
