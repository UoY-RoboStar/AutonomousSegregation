import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
servo =  GPIO.PWM(11,50)
servo.start(0)
print("Wait 1 second")
time.sleep(1)
print("rotating in 12 degreee intervals to 180 degrees")
while True:
	duty = 2
	#while duty <= 17:
        	#servo.ChangeDutyCycle(duty)
        	#time.sleep(1)
        	#duty = duty + 1

	print("turning back to 0 degrees")
	servo.ChangeDutyCycle(3.95)
	time.sleep(1)
servo.ChangeDutyCycle(7.0)
time.sleep(1)
servo.ChangeDutyCycle(0)
cleanup
servo.stop()
GPIO.cleanup()
