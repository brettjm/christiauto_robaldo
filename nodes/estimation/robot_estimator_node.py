#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
# from estimators import Robot

class Robot(object):
   def __init__(self, xhat, yhat, thhat):
      self.xhat  = xhat
      self.yhat  = yhat
      self.thhat = thhat
      self.vhat_x  = 0.0
      self.vhat_y  = 0.0
      self.vhat_th  = 0.0

   def get_state(self):
      return(self.xhat, self.yhat, self.thhat)

_ctrl_period = 1.0/100  
Tc = .01 # sample rate of controller

robot = Robot(0, 0, 0)
runCorrector = False
alpha = 0.3 #close to 0 for lots of noise, close to 1 for little noise
beta = 0.95
meas_x = 0.0
meas_y = 0.0
meas_th = 0.0

def _handle_robot_state(msg):
   global runCorrector
   global meas_x, meas_y, meas_th

   meas_x  = msg.x
   meas_y  = msg.y
   meas_th = msg.theta

   runCorrector = True
   
# predictor: between measurements
def predictor(obj):
   global sem

   # phat = phat + Tc*V
   obj.xhat  = obj.xhat  + Tc * obj.vhat_x
   obj.yhat  = obj.yhat  + Tc * obj.vhat_y
   obj.thhat = obj.thhat + Tc * obj.vhat_th

# corrector: measure received
def corrector(obj):
   global alpha, beta
   global meas_x, meas_y, meas_th

   # First time running, pass measured vals through
   # to avoid winding up velocity
   if obj.xhat == 0 and obj.yhat == 0 and obj.thhat == 0:
      obj.xhat = meas_x
      obj.yhat = meas_y
      obj.thhat = meas_th

   else:
      # update velocity using current position
      # and estimated position at last time step
      obj.vhat_x = beta*obj.vhat_x + (1-beta)*((meas_x - obj.xhat) / Tc)
      obj.vhat_y = beta*obj.vhat_y + (1-beta)*((meas_y - obj.yhat) / Tc)
      obj.vhat_th = beta*obj.vhat_th + (1-beta)*((meas_th - obj.thhat) / Tc)

      # update position using predictor
      obj.xhat = obj.xhat + Tc * obj.vhat_x
      obj.yhat = obj.yhat + Tc * obj.vhat_y
      obj.thhat = obj.thhat + Tc * obj.vhat_th

      # update position using a weighted average of 
      # predicted value vs measured value
      obj.xhat = alpha*obj.xhat + (1-alpha)*meas_x
      obj.yhat = alpha*obj.yhat + (1-alpha)*meas_y
      obj.thhat = alpha*obj.thhat + (1-alpha)*meas_th

def main():
   global runCorrector

   rospy.init_node('robot_estimator', anonymous=False)

   # subscribe to robot state and robot state
   rospy.Subscriber('tracker_ally1', Pose2D, _handle_robot_state)
  
   # publish predicted robot position
   pub_predictedRobotPos = rospy.Publisher('pred_robot_state_ally1', Pose2D, queue_size=10)
   msg_robot = Pose2D()
   
   # set sleep rate for loop to stay consistent 
   rate = rospy.Rate(int(1/_ctrl_period))

   while not rospy.is_shutdown():

      if runCorrector:
         corrector(robot)
         runCorrector = False
         
      else:
         predictor(robot)  
   
      # print robot.vhat_x

      # publish new robot position
      (msg_robot.x, msg_robot.y, msg_robot.theta) = robot.get_state()
      pub_predictedRobotPos.publish(msg_robot)
      
      # debug
      # print(msg_robot.x)
      
      rate.sleep()

   rospy.spin()

if __name__ == '__main__':
   main()
