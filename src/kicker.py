import RPi.GPIO as GPIO
import time

pin_num = 7

def kick_init():
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(pin_num, GPIO.OUT)

def kick():
	GPIO.output(pin_num, GPIO.HIGH)
	time.sleep(.2)
	GPIO.output(pin_num, GPIO.LOW)

if __name__ == '__main__':
	kick_init()
	kick()


