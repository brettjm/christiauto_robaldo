#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from estimators import Ball

_ctrl_period = 1.0/100  
Tc = .01 # sample rate of controller

ball = Ball(0, 0)
runCorrector = False
alpha = 0.1 #close to 0 for lots of noise, close to 1 for little noise
beta = 0.95
meas_x = 0.0
meas_y = 0.0

def _handle_ball_state(msg):
   global runCorrector
   global meas_x, meas_y

   meas_x = msg.x
   meas_y = msg.y
   runCorrector = True
   
# predictor: between measurements
def predictor(obj):
   global sem

   # phat = phat + Tc*V
   obj.xhat = obj.xhat + Tc * obj.vhat_x
   obj.yhat = obj.yhat + Tc * obj.vhat_y

# corrector: measure received
def corrector(obj):
   global alpha, beta
   global meas_x, meas_y

   # First time running, pass measured vals through
   # to avoid winding up velocity
   if obj.xhat == 0 and obj.yhat == 0:
      obj.xhat = meas_x
      obj.yhat = meas_y

   else:
      # update velocity using current position
      # and estimated position at last time step
      obj.vhat_x = beta*obj.vhat_x + (1-beta)*((meas_x - obj.xhat) / Tc)
      obj.vhat_y = beta*obj.vhat_y + (1-beta)*((meas_y - obj.yhat) / Tc)

      # update position using predictor
      obj.xhat = obj.xhat + Tc * obj.vhat_x
      obj.yhat = obj.yhat + Tc * obj.vhat_y

      # update position using a weighted average of 
      # predicted value vs measured value
      obj.xhat = alpha*obj.xhat + (1-alpha)*meas_x
      obj.yhat = alpha*obj.yhat + (1-alpha)*meas_y

def main():
   global runCorrector

   rospy.init_node('estimator', anonymous=False)

   # subscribe to robot state and ball state
   # rospy.Subscriber('robot_state', Pose2D, _handle_robot_state)
   rospy.Subscriber('tracker_ball', Pose2D, _handle_ball_state)
  
   # publish predicted ball position
   pub_predictedBallPos = rospy.Publisher('pred_ball_state_ally1', Pose2D, queue_size=10)
   msg_ball = Pose2D()
   
   # set sleep rate for loop to stay consistent 
   rate = rospy.Rate(int(1/_ctrl_period))

   while not rospy.is_shutdown():

      if runCorrector:
         corrector(ball)
         runCorrector = False
         
      else:
         predictor(ball)  
   
      # print ball.vhat_x

      # publish new ball position
      (msg_ball.x, msg_ball.y) = ball.get_state()
      pub_predictedBallPos.publish(msg_ball)
      
      # debug
      # print(msg_ball.x)
      
      rate.sleep()

   rospy.spin()

if __name__ == '__main__':
   main()
