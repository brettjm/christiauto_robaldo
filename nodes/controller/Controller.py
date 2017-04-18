import numpy as np

from controllers import P

P_x = None
P_y = None
P_theta = None

_set_point = (0, 0, 0)  # x, y, theta

# A flag to determine whether or not we are at our set point
_arrived = False

# These let us update theta at smaller rates than x and y
_loop_count = 0
_theta_loops = 2 # so every 2 loops do theta controller

velocities = (0, 0, 0)

def init():
    global P_x, P_y, P_theta

#####################################################
#                   Pixel gains
    # # Proportional gains
    # kpx  =  0.004
    # kpy  =  0.004
    # kpth = -0.09

    # # Derivative gains
    # kdx  = -.003
    # kdy  = -.001
    # kdth = .1

    # # Intergral gains
    # kix  = 0
    # kiy  = 0
    # kith = -.0001

    # # Instantiate x, y, and th PD controller
    # P_x     = P(kpx,  kdx, kix, .5, 0.05)
    # P_y     = P(kpy,  kdy, kiy, .5, 0.05)
    # P_theta = P(kpth, kdth, kith, 360, 0.05)

#####################################################
#                   Pixel gains
    # Proportional gains
    kpx  =  0.757
    kpy  =  0.757
    kpth = -.09

    # Derivative gains
    kdx  = -.7
    kdy  = -.3
    kdth = .1

    # Intergral gains
    kix  = 0
    kiy  = 0
    kith = -.0001

    # Instantiate x, y, and th PD controller
    P_x     = P(kpx,  kdx, kix, 1)
    P_y     = P(kpy,  kdy, kiy, 1)
    P_theta = P(kpth, kdth, kith, 360)


def set_commanded_position(x, y, theta):
    """Set Commanded Position
    x_c, y_c, theta_c. These will tell the controller where it wants to go.
    theta_c (degrees) can be given on the interval [0, 360].
    This function can also receive theta from [-360, 0].
    """
    global _set_point, _arrived

    # We aren't there yet!
    _arrived = False

    # This deals with negative theta inputs
    theta = theta % 360

    _set_point = (x, y, theta)
    return True

def get_commanded_position():
    return _set_point

def update(time_since_last_update, xhat, yhat, thetahat):
    global velocities, _arrived, _loop_count

    if _arrived:
        print "Quiting on _arrived."
        return (0, 0, 0)  # Don't even try

    if P_x is None or P_y is None or P_theta is None:
        # Controller hasn't been properly initialized
        print "Controller hasn't been intialized properly"
        return (0, 0, 0)
    
    # Break out variables for easy access
    x_c = _set_point[0]
    y_c = _set_point[1]
    theta_c = _set_point[2]

    Ts = time_since_last_update

    # Initialize velocities
    vx = vy = w = 0

    # Only control the positions that aren't 'close'
    if not _close(x_c, xhat):
        vx = P_x.update(x_c, xhat, Ts)

    if not _close(y_c, yhat):
        vy = P_y.update(y_c, yhat, Ts)

    # We've had another motion loop!
    _loop_count = _loop_count + 1

    # Only update theta every _theta_loops times
    if  (
            # _loop_count % _theta_loops == 0 and
            _close(x_c, xhat) and
            _close(y_c, yhat)
        ):
        update_theta = True
        _loop_count = 0
    else:
        update_theta = False
    
    if  (   
            update_theta and 
            not _close(theta_c, thetahat, tolerance=20) and
            not (abs(thetahat-theta_c) > (360 - 20))
        ): # degrees
        # Since the max distance you should ever go is 180 degrees,
        # test to see so that the commanded value is proportional to
        # the error between commanded and actual.
        # Basically, this makes going in circles cooler.
        if abs(thetahat-theta_c) > 180:
            if theta_c < thetahat:
                theta_c = theta_c + 360
            else:
                theta_c = theta_c - 360
    
        w  = P_theta.update(theta_c, thetahat, Ts, max_error_window=0)

    #print ("vx: %.1f, vy: %.1f, w: %.1f" % (vx, vy, w))

    # Are we there yet?
    _arrived = (vx == 0 and vy == 0 and w == 0 and update_theta)
    
    velocities = (vx, vy, w)

    return velocities

def _close(a, b, tolerance=0.1):
    return abs(a - b) <= tolerance


def move_to_location(x, y, theta):
    #_set_point is desired position
    #x, y, and theta are our current position
    distance_from_pos = np.sqrt(((x-_set_point[0])**2)+((y-_set_point[1])**2))
    print ("distance: ", distance_from_pos)
    print ("---------------")