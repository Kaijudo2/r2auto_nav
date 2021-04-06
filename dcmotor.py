import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(33,GPIO.OUT)

startstop = 1 #Command to start or stop
p = GPIO.PWM(33,200) # GPIO13 as PWM output with 100Hz Frequency
p.start(0) # generate PWM signal with 50% duty cycle value
x=0 #integer for storing duty cycle value

try:
	while True:
		n= int(input())
		p.ChangeDutyCycle(n)
		print(n) 
except KeyboardInterrupt:
	pass
	
p.stop()
GPIO.cleanup()




