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

    R = .035    #radius of the wheels (in m)

    rx1 = 0.067
    ry1 = 0.038
    rx2 = -0.067
    ry2 = 0.038
    rx3 = 0
    ry3 = -0.038
    
    sx1 = -0.567
    sy1 = 0.824
    sx2 = -0.567
    sy2 = -0.824
    sx3 = 1
    sy3 = 0

    # R = .0282977488817 # radius of wheel
    # r = .035 # radius from robot center to each wheel

    # # r_k is a vector that points from center of robot to center of each wheel
    # r1 = Vector(r,theta=np.pi/3)
    # r2 = Vector(r,theta=np.pi)
    # r3 = Vector(r,theta=5*np.pi/3)

    # # s_k is a unit vector that points in the direction of spin
    # s1 = Vector(1,theta=(r1.theta + np.pi/2))
    # s2 = Vector(1,theta=(r2.theta + np.pi/2))
    # s3 = Vector(1,theta=(r3.theta + np.pi/2))

    # # Create the M matrix that relates body and world coordinates
    # mSub = np.matrix([ [s1.x, s1.y, (s1.y*r1.x - s1.x*r1.y)],
    #                    [s2.x, s2.y, (s2.y*r2.x - s2.x*r2.y)],
    #                    [s3.x, s3.y, (s3.y*r3.x - s3.x*r3.y)]
    #                  ])
    # M = (1.0/R)*mSub

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




