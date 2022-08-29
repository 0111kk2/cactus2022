from operator import truediv
import RPi.GPIO as GPIO
import time

led = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(led, GPIO.OUT)
print("start")
GPIO.output(led, GPIO.LOW)
time.sleep(3)
print("melt_start")
GPIO.output(led, GPIO.HIGH)
time.sleep(10)
GPIO.output(led, GPIO.LOW)
print("off")
time.sleep(3)

GPIO.cleanup()
