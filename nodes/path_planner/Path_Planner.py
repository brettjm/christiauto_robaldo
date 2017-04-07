#!/usr/bin/env python

robot_radius = .25

#This function will calculate the slope of 'me' to the 
#ball and determine if any of the opponents are on that path
def need_planned_path(_me, _x_c, _y_c, _opp1, _opp2):
	# print "me_hat_future: " + str(_me.xhat_future) + ", " + str(_me.yhat_future) + "\n"
	# print "desired pos: " + str(_x_c) + ", " + str(_y_c) + "\n"
	# print "opp1: " + str(_opp1.xhat_future) + ", " + str(_opp1.yhat_future) + "\n"
	# print "opp2: " + str(_opp2.xhat_future) + ", " + str(_opp2.yhat_future) + "\n"
	# print "--------------------------------------------------\n\n"
	if _me.yhat_future is 0 or _me.xhat_future is 0:
		return None
	if _me.xhat_future == _x_c:
		return None
	slope = (_me.yhat_future - _y_c) / (_me.xhat_future - _x_c)

	opp1_intersect = slope * _opp1.xhat_future
	opp2_intersect = slope * _opp2.xhat_future
	obstacle = None

	#Check to see if the intersection point is anywhere between the diameter of the opponent
	if (_opp1.yhat_future <= opp1_intersect+robot_radius) and (_opp1.yhat_future >= opp1_intersect-robot_radius):
		obstacle = _opp1
	elif (_opp2.yhat_future <= opp2_intersect+robot_radius) and (_opp2.yhat_future >= opp2_intersect-robot_radius):
		obstacle = _opp2
	else:
		return None

	if (slope != 0) and ((abs(_x_c) - abs(_me.xhat_future)) > (abs(obstacle.xhat_future) - abs(_me.xhat_future))):
		return obstacle

	if (slope == 0) and ((abs(_x_c) - abs(_me.xhat_future)) > (abs(obstacle.xhat_future) - abs(_me.xhat_future))):
		return obstacle

	return None

def plan_path(_me, _x_c, _y_c, _opp1, _opp2, obstacle):
	# pick four arbitrary points one radius length away from the opponent's position
	# and run need_path_planned on each point treating each as a new "me" position
	# On the points that don't need a planned path, run need_path_planned again
	# treating them as the desired position. 
	# Choose the first point that returns false.

	new_me = [obstacle, obstacle, obstacle, obstacle]
	new_me[0].xhat_future -= robot_radius
	new_me[1].xhat_future += robot_radius
	new_me[2].xhat_future -= robot_radius
	new_me[3].xhat_future += robot_radius

	for i in new_me:
		if not need_planned_path(i, _x_c, _y_c, _opp1, _opp2):
			if not need_planned_path(_me, i.xhat_future, i.yhat_future, _opp1, _opp2):
				return i.xhat_future, i.yhat_future

	return None, None

# class msg():
# 	def __init__(self, name, x, y, theta):
# 		self.name = name
# 		self.x = x
# 		self.y = y
# 		self.theta = theta


# if __name__ == '__main__':
# 	_me   = msg("me", 10,10,0)
# 	_x_c  = 50
# 	_y_c  = 30
# 	_opp1 = msg("opp1", 40,25,0)
# 	_opp2 = msg("opp2", 50,20,0)

# 	if need_planned_path(_me, _x_c, _y_c, _opp1, _opp2):
# 		print "path planned"
# 	else:
# 		print "no path"