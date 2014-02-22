from math import pi
from robot_utils import Robot, PIDController, Pose, Supervisor
import matplotlib.pyplot as plt

class TestRobot(Robot):
    def __init__(self,r,b,tpr,max_vel=0,min_vel=0):
        super(TestRobot,self).__init__(r,b,tpr,max_vel,min_vel)
        #max_speed = 15 #radians/second
    left_power = 0
    right_power = 0
    left_ticks = 0
    right_ticks = 0
        

robot = TestRobot(0.043/2,0.147,720,15,3)

controller = PIDController(5,0,0,0.4)

class Goal(object):
    pass

goal = Goal()
goal.x = 0.2
goal.y = 0.2

supervisor = Supervisor(robot,Pose(0,0,0),goal)

supervisor.controller = controller

p = []

for x in range(100):
    dt = 0.01
    uni = supervisor.execute(dt)

    diff = robot.uni2diff(uni)

    robot.left_ticks = robot.left_ticks + diff[0]/(2*pi)*robot.ticks_per_rev*dt
    robot.right_ticks = robot.right_ticks + diff[1]/(2*pi)*robot.ticks_per_rev*dt
    p.append((supervisor.pose_est.x,supervisor.pose_est.y))
print(uni,diff)
print(robot.left_ticks,robot.right_ticks)

supervisor = Supervisor(robot,Pose(0,0,0),goal)

supervisor.controller = controller

p2 = []

for x in range(100):
    dt = 0.01
    uni = supervisor.execute(dt)

    diff = robot.uni2diff(uni)
    diff = robot.ensure_w(diff)

    robot.left_ticks = robot.left_ticks + diff[0]/(2*pi)*robot.ticks_per_rev*dt
    robot.right_ticks = robot.right_ticks + diff[1]/(2*pi)*robot.ticks_per_rev*dt
    p2.append((supervisor.pose_est.x,supervisor.pose_est.y))
print(uni,diff)
print(robot.left_ticks,robot.right_ticks)


plt.plot([x[0] for x in p],[x[1] for x in p],[x[0] for x in p2],[x[1] for x in p2])
plt.show()
