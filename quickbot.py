
import time, random, threading
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
from robot_utils import Robot, PIDController, Supervisor, Pose

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
BrickPiUpdateValues()

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
        t = time.time()
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

            dt = time.time() - t
            update_robot(self.robot, self.supervisor, dt)
            t = time.time()
                               
            #BrickPi.MotorSpeed[PORT_C] = direction * SCAN_SPEED
            time.sleep(0.05)
            
class MyRobot(Robot):

    @property
    def left_power(self):
        return BrickPi.MotorSpeed[PORT_A]

    @left_power.setter
    def left_power(self,power):
        BrickPi.MotorSpeed[PORT_A] = power

    @property
    def right_power(self):
        return BrickPi.MotorSpeed[PORT_D]

    @right_power.setter
    def right_power(self,power):
        BrickPi.MotorSpeed[PORT_D] = power

    @property
    def left_ticks(self):
        return BrickPi.Encoder[PORT_A]

    @property
    def right_ticks(self):
        return BrickPi.Encoder[PORT_D]
        
    def vel2power(self,v):
        v_ratio = (v - self.min_velocity)/(self.max_velocity - self.min_velocity)
        return v_ratio*(self.max_power-self.min_power)+self.min_power

    
robot = MyRobot(0.043/2,0.147,720.0,15.0,3.0)
robot.max_power = 255
robot.min_power = 60

controller = PIDController(1,0,0,0.2)

class Goal(object):
    pass
goal = Goal()
goal.x = 1.0
goal.y = 0.0

supervisor = Supervisor(robot,Pose(0,0,0),goal)

supervisor.controller = controller


def update_robot(robot, supervisor,dt):
    uni = supervisor.execute(dt)
    diff = robot.uni2diff(uni)
    diff = robot.ensure_w(diff)
    lv,rv = diff
    robot.left_power = int(robot.vel2power(lv))
    robot.right_power = int(robot.vel2power(rv))

scanner = Scanner()
scanner.robot = robot
scanner.supervisor = supervisor
scanner.daemon = True
scanner.start()    
    
print robot.left_power,robot.right_power,robot.left_ticks,robot.right_ticks
time.sleep(0.055)
    
while True:
    try:
      
      
        print robot.left_power,robot.right_power,robot.left_ticks,robot.right_ticks
        time.sleep(0.5)



    except KeyboardInterrupt:			#Triggered by pressing Ctrl+C
        running = False
        time.sleep(1.0)
        GPIO.cleanup()
        print "Bye"
        break					#Exit

