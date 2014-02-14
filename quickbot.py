
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

running = True
class Scanner(threading.Thread):
    def run(self):
        BrickPi.MotorEnable[PORT_C] = 1
        SCAN_SPEED = 60
        direction = 1
        last_read = 2
        while running:
            result = BrickPiUpdateValues()
            if result:
                print result
            angle = BrickPi.Encoder[PORT_C]
            if direction == 1 and angle >= 2*SENSOR_ANGLE[last_read+1]:
                sensor_dist[last_read+1] = (angle/2,BrickPi.Sensor[PORT_3])
                last_read = last_read+1
                if last_read == len(SENSOR_ANGLE)-1:
                    direction = -1
            elif direction == -1 and angle <= 2*SENSOR_ANGLE[last_read-1]:
                sensor_dist[last_read-1] = (angle/2,BrickPi.Sensor[PORT_3])
                last_read = last_read-1
                if last_read == 0:
                    direction = 1
                               
            BrickPi.MotorSpeed[PORT_C] = direction * SCAN_SPEED
            time.sleep(0.05)
            

scanner = Scanner()
scanner.daemon = True
scanner.start()
            
while True:
    try:
      


        print sensor_dist
        time.sleep(1.0)



    except KeyboardInterrupt:			#Triggered by pressing Ctrl+C
        running = False
        time.sleep(1.0)
        GPIO.cleanup()
        print "Bye"
        break					#Exit

