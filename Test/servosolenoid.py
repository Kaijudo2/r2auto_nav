import time 
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

GPIO.setup(36, GPIO.OUT)

ss = GPIO.PWM(36,50)
ss.start(7.5)

try: 
	while True:
		n1 = int(input())
		ang1 = (n1/180*10) +2.5
		ss.ChangeDutyCycle(ang1)
except KeyboardInterrupt:
	ss.stop()	
	GPIO.cleanup	
