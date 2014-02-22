from math import pi, sin, cos, atan2,copysign

class Robot(object):

    def __init__(self,wheel_radius,base_width,ticks_per_rev,max_vel=0,min_vel=0):
        self.wheel_radius = wheel_radius  #meters
        self.base_width = base_width   #meters
        self.ticks_per_rev = ticks_per_rev 
        self.max_velocity = max_vel
        self.min_velocity = min_vel


    def uni2diff(self,uni):
        """Convert between unicycle model to differential model"""
        (v,w) = uni

        summ = 2*v/self.wheel_radius
        diff = self.base_width*w/self.wheel_radius

        vl = (summ-diff)/2
        vr = (summ+diff)/2

        return (vl,vr)

    def ensure_w(self,v_lr):
        # This code is taken directly from Sim.I.Am week 4
        # I'm sure one can do better. 

        v_max = self.max_velocity
        v_min = self.min_velocity
       
        R = self.wheel_radius
        L = self.base_width
        
        def diff2uni(vl,vr):
            return (vl+vr) * R/2, (vr-vl) * R/L
        
        v, w = diff2uni(*v_lr)
        
        if v == 0:
            
            # Robot is stationary, so we can either not rotate, or
            # rotate with some minimum/maximum angular velocity

            w_min = R/L*(2*v_min);
            w_max = R/L*(2*v_max);
            
            if abs(w) > w_min:
                w = copysign(max(min(abs(w), w_max), w_min), w)
            else:
                w = 0
            
            return self.uni2diff((0,w))
            
        else:
            # 1. Limit v,w to be possible in the range [vel_min, vel_max]
            # (avoid stalling or exceeding motor limits)
            v_lim = max(min(abs(v), (R/2)*(2*v_max)), (R/2)*(2*v_min))
            w_lim = max(min(abs(w), (R/L)*(v_max - v_min)), 0)
            
            # 2. Compute the desired curvature of the robot's motion
            
            vl,vr = self.uni2diff((v_lim, w_lim))
            
            # 3. Find the max and min vel_r/vel_l
            v_lr_max = max(vl, vr);
            v_lr_min = min(vl, vr);
            
            # 4. Shift vr and vl if they exceed max/min vel
            if (v_lr_max > v_max):
                vr -= v_lr_max - v_max
                vl -= v_lr_max - v_max
            elif (v_lr_min < v_min):
                vr += v_min - v_lr_min
                vl += v_min - v_lr_min
            
            # 5. Fix signs (Always either both positive or negative)
            v_shift, w_shift = diff2uni(vl,vr)
            
            v = copysign(v_shift,v)
            w = copysign(w_shift,w)
            
            return self.uni2diff((v,w))


        
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

    def get_left_ticks(self):
        return BrickPi.Encoder[PORT_A]

    left_ticks = property(get_left_ticks)

    @property
    def right_ticks(self):
        return BrickPi.Encoder[PORT_D]

class PIDController(object):

    def __init__(self,kp,ki,kd,v):
        self.velocity = v
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.restart()
        self.heading_angle = 0
    
    def execute(self, pose, goal, dt):
     
        # This is the direction we want to go
        self.heading_angle = self.get_heading_angle(pose,goal)

        # 1. Calculate simple proportional error
        # The direction is in the robot's frame of reference,
        # so the error is the direction.
        # Note that the error is automatically between pi and -pi.
        error = self.heading_angle

        # 2. Calculate integral error
        self.E += error*dt
        self.E = (self.E + pi)%(2*pi) - pi

        # 3. Calculate differential error
        dE = (error - self.error_1)/dt
        self.error_1 = error #updates the error_1 var

        # 4. Calculate desired omega
        w_ = self.kp*error + self.ki*self.E + self.kd*dE
        
        # The linear velocity is given to us:
        #print error, self.E, dE
        return [self.velocity, w_]

    
    def restart(self):
        self.E = 0
        self.error_1 = 0

    def get_heading(self,pose,goal):

        goal_angle = self.get_heading_angle(pose,goal)
        return np.array([cos(goal_angle),sin(goal_angle),1])
        
        
    def get_heading_angle(self, pose, goal):
        """Get the direction from the robot to the goal as a vector."""
        
        # The goal:
        x_g, y_g = goal.x, goal.y
        
        # The robot:
        x_r, y_r, theta = pose

        # Where is the goal in the robot's frame of reference?
        return (atan2(y_g - y_r, x_g - x_r) - theta + pi)%(2*pi) - pi




class Supervisor(object):

    def __init__(self,robot,pose,goal):
        self.left_ticks = robot.left_ticks
        self.right_ticks = robot.right_ticks
        self.pose_est = pose
        self.goal = goal
        self.robot = robot

    def execute(self,dt):
        self.update_pose_est()
        return self.controller.execute(self.pose_est,self.goal,dt)


    def update_pose_est(self):
        """Update self.pose_est using odometry"""
        
        # Get tick updates
        dtl = self.robot.left_ticks - self.left_ticks
        dtr = self.robot.right_ticks - self.right_ticks
        
        # Save the wheel encoder ticks for the next estimate
        self.left_ticks += dtl
        self.right_ticks += dtr
        
        x, y, theta = self.pose_est

        R = self.robot.wheel_radius
        L = self.robot.base_width
        m_per_tick = (2*pi*R)/self.robot.ticks_per_rev
            
        # distance travelled by left wheel
        dl = dtl*m_per_tick
        # distance travelled by right wheel
        dr = dtr*m_per_tick
            
        theta_dt = (dr-dl)/L
        theta_mid = theta + theta_dt/2
        dst = (dr+dl)/2
        x_dt = dst*cos(theta_mid)
        y_dt = dst*sin(theta_mid)
            
        theta_new = theta + theta_dt
        x_new = x + x_dt
        y_new = y + y_dt

        self.pose_est = Pose(x_new, y_new, (theta_new + pi)%(2*pi)-pi)
           

class Pose(object):
    def __init__(self, x=0,y=0,theta=0):
        self.x, self.y, self.theta = x,y,theta

    def get_list(self):
        """Get the pose as a list ``[x, y, theta]``. Equivalent to ``list(pose)``."""
        return [self.x, self.y, self.theta]

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.theta

    def get_transformation(self):
        """Get the 3x3 transformation matrix associated with the pose."""
        #Z-axis ccw rotation transformation matrix
        T = np.array([\
            [np.cos(self.theta), -np.sin(self.theta), self.x],\
            [np.sin(self.theta), np.cos(self.theta), self.y],\
            [0, 0, 1.0]])
        return T

    def __str__(self):
        return "(%f,%f) %f" % (self.x,self.y,self.theta)
    
    def iscloseto(self,other,epsilon):
        """Compare this pose to *other*. Returns True if the relative distance
           in x, y and theta is smaller than *epsilon*
        """
        return (self.x - other.x)/(self.x + other.x) < epsilon \
           and (self.y - other.y)/(self.y + other.y) < epsilon \
           and (self.theta - other.theta)%(2*pi)/(2*pi) < epsilon
    
    def __eq__(self,other):
        if not isinstance(other,Pose):
            return NotImplemented
        return self.iscloseto(other,1e-8)
    
    def __rshift__(self,other):
        """A shifted Pose is the same pose in the coordinate system defined by the other pose.
           This operation is not commutative.
           
           If ``b`` is a pose in ``a`` frame of reference, ``b >> a`` is the same pose 
           if the frame of reference that ``a`` is defined.
           """
        if not isinstance(other,Pose):
            return NotImplemented
        rx, ry, rt = other
        return Pose(rx+self.x*cos(rt)-self.y*sin(rt),ry+self.x*sin(rt)+self.y*cos(rt),self.theta+rt)

    def __lshift__(self,other):
        """An unshifted Pose is the same pose in the local coordinate system of other pose.
           This operation is not commutative.
           
           If ``a`` and ``b`` are poses in the same frame of reference, then ``b << a``
           is ``b`` in ``a`` frame of reference.           
           """
        if not isinstance(other,Pose):
            return NotImplemented
        rx, ry, rt = other
        return Pose((self.x-rx)*cos(rt)+(self.y-ry)*sin(rt),-(self.x-rx)*sin(rt)+(self.y-ry)*cos(rt),self.theta-rt)
        
        
