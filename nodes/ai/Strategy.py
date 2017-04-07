import numpy as np

import Roles
import Plays
import Skills
import Utilities
import Constants

from GameObjects import Ball, Robot

#Variables for determining if ball stuck
_ball_last_x = 0
_ball_last_y = 0
_ball_stuck_counter = 0
_ball_stuck_tolerance = 0.1
_ball_stuck_max_time = 100

#Variables for tracking opponent's strategy
_avg_dist_between_opponents         = 0
_averaging_factor                   = 0
_percent_time_ball_in_our_half      = 0
_percent_time_opponents_in_our_half = 0
_our_score                          = 0
_opponent_score                     = 0

# For detecting goal 
_goal_check_counter                 = 0
_GOAL_COUNTER_MAX                   = 2 # 10 for real life, 2 for simulator

_resume_game_counter                = 0
_RESUME_GAME_MAX                    = 50 # (5 seconds)

class G:
    NO_ONE = 0
    US = 1
    THEM = 2


# ally1 is designated as the "main" attacker, or the robot closest to the opponent's goal at the beginning of the game
# ally2 is designated as the "main" defender, or the robot closest to our goal at the beginnning of the game
def choose_strategy(me, my_teammate, opponent1, opponent2, ball, game_state, team_side):
    global _avg_dist_between_opponents, _averaging_factor, _percent_time_ball_in_our_half, _percent_time_opponents_in_our_half
    global _our_score, _opponent_score
    global _ball_last_x, _ball_last_y, _ball_stuck_counter
    update_opponents_strategy_variables(opponent1, opponent2, ball)
    
    # Check to see if someone scored a goal and to see if we are losing
    losing = update_score(game_state, team_side)

    if losing or _our_score == _opponent_score:
        if (abs(ball.xhat - _ball_last_x) <= _ball_stuck_tolerance):
            if (abs(ball.yhat - _ball_last_y) <= _ball_stuck_tolerance):
                _ball_stuck_counter += 1
            else:
                _ball_stuck_counter = 0
        else:
            _ball_stuck_counter = 0
        _ball_last_x = ball.xhat
        _ball_last_y = ball.yhat
        if _ball_stuck_counter > _ball_stuck_max_time:
            if _ball_stuck_counter > 3 * _ball_stuck_max_time:
                return aggressive_defense(me, my_teammate, opponent1, opponent2, ball)
            return (ball.xhat, ball.yhat, me.thetahat)


    # one_v_one or two_v_two?
    us_count = getattr(game_state, '{}_bot_count'.format(team_side))
    one_v_one = us_count == 1

    opp_strong_offense = (_percent_time_ball_in_our_half >= 0.50 and _avg_dist_between_opponents <=  1.5 )  
    #for now, we will just focus on aggressive offense
    if (one_v_one):
        return one_on_one(me, opponent1, ball)
    else:
        if (_opponent_score > _our_score) or opp_strong_offense:
            return aggressive_offense(me, my_teammate, opponent1, opponent2, ball)
        elif _opponent_score == _our_score:
            return passive_aggressive(me, my_teammate, opponent1, opponent2, ball)
        else:
            return aggressive_defense(me, my_teammate, opponent1, opponent2, ball)
            #return aggressive_offense(me, my_teammate, opponent1, opponent2, ball)



def aggressive_offense(me, my_teammate, opponent1, opponent2, ball):
    global _beginning_trick_shot
    section = Utilities.get_field_section(ball.xhat)

    if me.ally1:
        #if not Plays.beginning_trick_shot_done():
        #    return Plays.shoot_off_the_wall(me, ball)
        if section == 1:
            return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos
    else:
        if   section == 1:
            return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball) #This used to be offensive defender, but i want to see the goalie do it's thing
        elif section == 3:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos


def aggressive_defense(me, my_teammate, opponent1, opponent2, ball):
    section = Utilities.get_field_section(ball.xhat)

    if me.ally1:
        if section == 1:
            return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos
    else:
        if   section == 1:
            return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball) #This used to be offensive defender, but i want to see the goalie do it's thing
        elif section == 3:
            return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos


def passive_aggressive(me, my_teammate, opponent1, opponent2, ball): #AKA, mild offense/defense
    section = Utilities.get_field_section(ball.xhat)

    if me.ally1:
        if   section == 1:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos
    else:
        if   section == 1:
            return Roles.neutral_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos


def one_on_one(me, opponent1, ball):
    my_teammate = None
    opponent2 = None
    section = Utilities.get_field_section(ball.xhat)
    # print ("section = {}" .format(section))
    # if Utilities.i_am_stuck(me):
    #     return Skills.get_unstuck(me)
    # else:
        # if not Plays.beginning_trick_shot_done():
        #     return Plays.shoot_off_the_wall(me, ball)
    if   section == 1:
        return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball, True)
    elif section == 2:
        return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball, True)
    elif section == 3:
        return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball, True)
    elif section == 4:
        return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball, True)
    else:
        return (me.xhat, me.yhat, me.thetahat) #default, returns current pos




################################################################
###                     Helper Functions                     ###
################################################################

def check_for_goal(ball):
    global _goal_check_counter, _GOAL_COUNTER_MAX, _is_goal_global
    # If someone just scored, then don't do anything
    if not _is_goal_global:
        far_enough_away_from_goal = 0.10
        if abs(ball.xhat) <= Constants.field_length/2 - far_enough_away_from_goal:
            # Reset Counter because ball is far enough away from the goal
            _goal_check_counter = 0 
        elif abs(ball.xhat) >= Constants.field_length/2 + Constants.goal_score_threshold:
            # Update counter
            _goal_check_counter = _goal_check_counter + 1
            if _goal_check_counter >= _GOAL_COUNTER_MAX:
                _goal_check_counter = 0
                _is_goal_global = True
                # Update the score here, so it only does it once
                update_score(ball)


def update_score(game_state, team_side):
    global _our_score, _opponent_score

    # Break out the scores from the message
    us_score = getattr(game_state, '{}_score'.format(team_side))
    other_team_side = 'away' if team_side == 'home' else 'home'
    them_score = getattr(game_state, '{}_score'.format(other_team_side))

    goal = False

    if us_score > _our_score:
        _our_score = us_score
        goal = True
    elif them_score > _opponent_score:
        _opponent_score = them_score
        goal = True

    return them_score > us_score


def update_opponents_strategy_variables(opponent1, opponent2, ball):
    global _avg_dist_between_opponents, _averaging_factor, _percent_time_ball_in_our_half, _percent_time_opponents_in_our_half
    _averaging_factor = _averaging_factor + 1

    # Compute the distance between the opponents, if distance is large, then they have a goalie implemented. 
    new_dist_between_opponents = Utilities.get_distance_between_points(opponent1.xhat, opponent1.yhat, opponent2.xhat, opponent2.yhat)
    _avg_dist_between_opponents = (_avg_dist_between_opponents + new_dist_between_opponents)/_averaging_factor

    # Calculate the amount of time the ball is spent in our half
    if Utilities.is_in_our_half(ball):
        _percent_time_ball_in_our_half = (_percent_time_ball_in_our_half + 1)/_averaging_factor
    else:
        _percent_time_ball_in_our_half = _percent_time_ball_in_our_half/_averaging_factor

    # Then Update the amount of time the opponent(s) are in our half.
    if Utilities.is_in_our_half(opponent1) and Utilities.is_in_our_half(opponent2):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 2)/_averaging_factor # if both players are on our side, then they are playing very offensively
    elif Utilities.is_in_our_half(opponent1):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 1)/_averaging_factor
    elif Utilities.is_in_our_half(opponent2):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 1)/_averaging_factor
    else:
        _percent_time_opponents_in_our_half = _percent_time_opponents_in_our_half/_averaging_factor


def are_robots_in_reset_position(me, my_teammate):
    if my_teammate is None:
        return Utilities.robot_close_to_point(me, Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2])
    else:
        if me.ally1:
            return Utilities.robot_close_to_point(me, Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2]) and Utilities.robot_close_to_point(my_teammate, Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])
        else:
            return Utilities.robot_close_to_point(my_teammate, Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2]) and Utilities.robot_close_to_point(me, Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])


def reset_positions_after_goal(me):
    if me.ally1:
        return (Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2])
    else:
        return (Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])


def done_waiting_for_resume_game():
    global _resume_game_counter, _RESUME_GAME_MAX

    _resume_game_counter = _resume_game_counter + 1
    if _resume_game_counter >= _RESUME_GAME_MAX:
        _resume_game_counter = 0
        return True

    return False


