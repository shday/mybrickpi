
import time, random, threading
from BrickPi import *   #import BrickPi.py file to use BrickPi operations

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print "Error importing RPi.GPIO. You need to run this with superuser privileges. Try sudo python LED.py"

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)    #GPIO 18
GPIO.setup(13, GPIO.OUT)    #GPIO 27

BrickPiSetup()  # setup the serial port for communication

BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
BrickPi.MotorEnable[PORT_D] = 1 #Enable the Motor D
BrickPi.MotorEnable[PORT_C] = 1 #Enable the Motor B - US Sensor Mount


BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT	
BrickPi.SensorType[PORT_1] = TYPE_SENSOR_TOUCH
BrickPi.SensorType[PORT_4] = TYPE_SENSOR_TOUCH

BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

sensor_dist = [None] * 5
SENSOR_ANGLE = [-90,-45,0,45,90]
SENSOR_ANGLE.sort()

class Scanner(threading.Thread):
    def run(self):
        BrickPi.MotorEnable[PORT_C] = 1
        SCAN_SPEED = 150
        direction = 1
        last_read = None
        while True:
            BrickPiUpdateValues()
            angle = BrickPi.Encoder[PORT_C]
            if angle >= SENSOR_ANGLE[-1]:
                direction = -1
                sensor_dist[-1] = BrickPi.Sensor[PORT_3]
                last_read = -1
            elif angle <= SENSOR_ANGLE[0]:
                direction = 1
                sensor_dist[0] = BrickPi.Sensor[PORT_3]
                last_read = 0
            elif last_read != 2:
                if direction == 1 and angle >= SENSOR_ANGLE[2]:
                    sensor_dist[2] = BrickPi.Sensor[PORT_3]
                    last_read = 2
                elif direction == -1 and angle <= SENSOR_ANGLE[2]:
                    sensor_dist[2] = BrickPi.Sensor[PORT_3]
                    last_read = 2
            BrickPi.MotorSpeed[PORT_C] = direction * SCAN_SPEED
            time.sleep(0.1)
            

scanner = Scanner()
scanner.deamon = True
scanner.start()
            
while True:
    try:
      


        print sensor_dist
        time.sleep(1.0)



    except KeyboardInterrupt:			#Triggered by pressing Ctrl+C
        GPIO.cleanup()
        print "Bye"
        break					#Exit

