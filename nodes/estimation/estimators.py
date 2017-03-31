

class Ball(object):
   def __init__(self, xhat, yhat):
      self.xhat  = xhat
      self.yhat  = yhat
      self.xhat_d1 = 0.0
      self.yhat_d1 = 0.0
      self.vhat_x  = 0.0
      self.vhat_y  = 0.0

   def update_state(self, x, y):
      self.xhat  = x
      self.yhat  = y

   def update_previous_state(self):
       self.xhat_d1 = self.xhat
       self.yhat_d1 = self.yhat

   def get_state(self):
      return(self.xhat, self.yhat)
 
   def get_prev_state(self):
      return(self.xhat_d1, self.yhat_d1)

   def calculate_predicted_vel(self, Ts):
       # Calculate zeroth-order derivative
       self.vhat_x   = (self.xhat - self.xhat_d1) / Ts
       self.vhat_y   = (self.yhat - self.yhat_d1) / Ts

   def calculate_predicted_pos(self, Ts):
       # p(t + T) = p(t) + V*T
       self.xhat  = self.xhat + self.vhat_x * Ts
       self.yhat  = self.yhat + self.vhat_y * Ts

      
