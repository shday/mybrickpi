
import time, random
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

#BrickPi.MotorSpeed[PORT_A] = -200  #Set the speed of MotorA (-255 to 255)
#BrickPiUpdateValues() 
#BrickPi.Sensor[PORT_4]
#GPIO.output(13, False)
#GPIO.cleanup()

print BrickPi.Encoder[PORT_C]

STOP_THRESHOLD = 30

def get_distance():
    return BrickPi.Sensor[PORT_3]

def rotate_turret():
    pass

def moving():
    global current_state
    #print "moving"
    GPIO.output(13, True)
    BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
    BrickPi.MotorEnable[PORT_D] = 1 #Enable the Motor D
    BrickPi.MotorSpeed[PORT_A] = 150
    BrickPi.MotorSpeed[PORT_D] = 150

    if (get_distance() < STOP_THRESHOLD
        or BrickPi.Sensor[PORT_4]
        or BrickPi.Sensor[PORT_1]):
        current_state = 'stopped'
    else:
        current_state = 'moving'

def backing_up():
    global current_state
    print "backing_up"
    power = 150
    deg = random.randint(360,360*5)
    motorRotateDegree([power,power],[-deg,-deg],[PORT_A,PORT_D])
    current_state = 'turning'    

def stopped():
    global current_state
    print "stopped"
    GPIO.output(12, False)
    GPIO.output(13, False)
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0

    if BrickPi.Sensor[PORT_4] or BrickPi.Sensor[PORT_1]:
        current_state = 'backing_up'
        print "hit something"
    elif get_distance() > STOP_THRESHOLD:
        current_state = 'moving'
    else:
        current_state = 'turning'

def turning():
    global current_state
    print "turning"
    GPIO.output(12, True)
    power = 150
    deg = random.randint(-360,360)
    motorRotateDegree([power,power],[deg,-deg],[PORT_A,PORT_D])
    current_state = 'stopped'

def looking():
    pass
    

states = { 'turning':turning, 'stopped':stopped,
           'moving':moving,'backing_up':backing_up }


def go_forward():
    pass
    #BrickPi.MotorSpeed[PORT_A] = 150
    #BrickPi.MotorSpeed[PORT_D] = 150

def turn_right():
    print BrickPi.Encoder[PORT_C]
    #BrickPi.MotorSpeed[PORT_A] = -150
    #BrickPi.MotorSpeed[PORT_D] = 150

current_state = 'stopped'
while True:
    try:
        BrickPiUpdateValues()

        states[current_state]()
        
        time.sleep(0.1)



    except KeyboardInterrupt:			#Triggered by pressing Ctrl+C
        GPIO.cleanup()
        print "Bye"
        break					#Exit

