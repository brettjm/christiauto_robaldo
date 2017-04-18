import numpy as np

M = 0

class Vector(object):
    """Vector object
    Create an object with an x, y, theta based on magnitude and angle
    """
    def __init__(self, r, theta=0):
        """
        theta is expected in radians
        """
        super(Vector, self).__init__()
        self.r = r
        self.theta = theta
        self.x = r*np.cos(theta)
        self.y = r*np.sin(theta)

# All measurements in m
def init():
    global M

    R = 0.018 #.025    #radius of the wheels (in m)

    rx1 = 0.067  #0.067
    ry1 = 0.036  #0.038
    rx2 = -0.067 #-0.067
    ry2 = 0.036  #0.038
    rx3 = 0
    ry3 = -0.077 #-0.038
    
    sx1 = -0.468 #-0.567
    sy1 = 0.884  #0.824
    sx2 = -0.543 #-0.567
    sy2 = -0.840 #-0.824
    sx3 = 1
    sy3 = 0

    Msub = np.matrix([[sx1, sy1, (sy1*rx1 - sx1*ry1)], 
                      [sx2, sy2, (sy2*rx2 - sx2*ry2)], 
                      [sx3, sy3, (sy3*rx3 - sx3*ry3)]])
    M = (1.0/R)*Msub

def world_to_wheels(vx, vy, w, th):
    global M  
    
    w  =  w * np.pi/180 # convert omega from degress to radians
    th = th * np.pi/180 # convert th from degrees to radians
    th = th + np.pi/2   # orient theta to match with vision 
    
    v1 = 0  # Wheel 1 velocity
    v2 = 0  # Wheel 2 velocity
    v3 = 0  # Wheel 3 velocity

    V = np.matrix([[vx], [vy], [w]])
    R = np.matrix([[ np.cos(th), np.sin(th), 0.0], 
                   [-np.sin(th), np.cos(th), 0.0], 
                   [0.0, 0.0, 1.0]])
    
    # omega(1,2,3) = MR(th)[vx, vy, w]
    wheel_velocities = M*R*V
       
    # wheel_velocities are v1, v2, v3
    return wheel_velocities

def v_to_rps(v1, v2, v3):
    rps1 = 0 # Wheel 1 rev/sec
    rps2 = 0 # Wheel 2 rev/sec
    rps3 = 0 # Wheel 3 rev/sec

    twopi = np.pi*2
    rps1 = v1/(twopi)
    rps2 = v2/(twopi)
    rps3 = v3/(twopi)

    wheel_velocities_rps = (rps1, rps2, rps3)
    return wheel_velocities_rps




