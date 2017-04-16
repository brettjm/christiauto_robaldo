#!/usr/bin/env python
import rospy
import numpy as np
import Controller
import Relationship
import CommandPSOC
from geometry_msgs.msg import Twist, Pose2D

# Steps to execute in controller_node:
#  1. Get actual position and desired position
#  2. Run positions through a P controller to get linear velocities
#  3. Run linear velocities through M matrix to get wheel_velocities
#  4. Convert wheel_velocities to rev/sec
#  5. Send wheel_velocities (rev/sec) to PSOC (has a PI controller) to get PWM

_ctrl_period = 1.0/100

#Current Robot Location
_xhat = 0
_yhat = 0
_thetahat = 0

_ctrl_on = True
_initializing = True

#  1. Get actual position and desired position
def _handle_robot_state(msg):
    global _xhat, _yhat, _thetahat, _initializing, _ctrl_on

    _xhat = msg.x
    _yhat = msg.y
    _thetahat = msg.theta

def _handle_desired_position(msg):
    global _ctrl_on
    Controller.set_commanded_position(msg.x, msg.y, msg.theta)
    _ctrl_on = True


def main():
    global _xhat, _yhat, _thetahat, _arrived
    rospy.init_node('controller', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('pred_robot_state_ally1', Pose2D, _handle_robot_state)

    # Controller.set_commanded_position(0, 60, 0) 
    # rospy.Subscriber('desired_position_ally1', Pose2D, _handle_desired_position)

    # initialize the controller and PSOC
    Controller.init()
    CommandPSOC.init()
    Relationship.init()

    rate = rospy.Rate(int(1/_ctrl_period))

    while not rospy.is_shutdown():
         global _ctrl_on
#        Controller.move_to_location(_xhat, _yhat, _thetahat)

         if _ctrl_on:
            Controller.set_commanded_position(500, 300, 90) 

	        #2. Run positions through a P controller to get linear velocities
            (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)
            #print "vw, vy, and w are ", vx, vy, w
	        
	        #3. Run linear velocities through M matrix to get wheel_velocities
            (v1, v2, v3) = Relationship.world_to_wheels(vx, vy, w, _thetahat)

	        #4. Convert wheel_velocities to rev/sec
            (rps1, rps2, rps3) = Relationship.v_to_rps(v1, v2, v3)          

            # hack to rps values so they break friction threshold
            limhigh = 1.75
            rps1 = np.sign(rps1)*limhigh if 0.15 < abs(rps1) and abs(rps1) < limhigh else rps1
            rps2 = np.sign(rps2)*limhigh if 0.15 < abs(rps2) and abs(rps2) < limhigh else rps2
            rps3 = np.sign(rps3)*limhigh if 0.15 < abs(rps3) and abs(rps3) < limhigh else rps3

            # just stop, don't even try when this close
            rps1 = 0.0 if 0.15 >= abs(rps1) else rps1
            rps2 = 0.0 if 0.15 >= abs(rps2) else rps2
            rps3 = 0.0 if 0.15 >= abs(rps3) else rps3

	        #5. Send wheel_velocities (rev/sec) to PSOC (has a PI controller) to get PWM
            CommandPSOC.setWheelVelocities(rps1, rps2, rps3)

            # debug
            print("rps: %.2f, %.2f, %.2f" % (rps1, rps2, rps3))            
            
         rate.sleep()
        
    CommandPSOC.disengage()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
