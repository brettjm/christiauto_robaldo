#include <iostream>
#include <cmath>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "christiauto_robaldo/GameState.h"

using std::string;
using std::cout;
using std::endl;

const double pi = 3.14159265358979;
bool attack = false;

string homeOrAway;

int robot_x;
int robot_y;
int robot_theta;

int ball_x;
int ball_y;
int ball_theta;

ros::Publisher pub;

bool play = false;
bool reset_field = false;
bool second_half = false;

void game_state_callback(const christiauto_robaldo::GameState& msg) {
	play = msg.play;
	reset_field = msg.reset_field;
	second_half = msg.second_half;
}

void robot_state_callback(const geometry_msgs::Pose2D& msg) {
	robot_x = msg.x;
	robot_y = msg.y;
	robot_theta = msg.theta;
}

void ball_state_callback(const geometry_msgs::Pose2D& msg) {
	ball_x = msg.x;
	ball_y = msg.y;
	ball_theta = msg.theta;

	geometry_msgs::Pose2D pub_msg;

    	if (homeOrAway == "home" && !second_half) {
		pub_msg.x = 70;
	}
	else if (homeOrAway == "home" && second_half) {
		pub_msg.x = 560;
	}
	else if (homeOrAway == "away" && !second_half) {
		pub_msg.x = 560;
	}
	else if (homeOrAway == "away" && second_half) {
		pub_msg.x = 70;
	}

	if (reset_field || !play) {
		pub_msg.y = 215;
	}
	else {
    		if (ball_y >= 170 && ball_y <= 260) {
    			pub_msg.y = ball_y;
		}
		else if (ball_y < 170) {
			pub_msg.y = 170;
		}
		else {
			pub_msg.y = 260;
		}
    		pub_msg.theta = 0;
	}

    	pub.publish(pub_msg);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Must provide 1 command line arguments to " << argv[0] << endl;
        cout << "<home || away>" << endl;
        return 1;
    }

    homeOrAway = string(argv[1]);
    if (homeOrAway != "home" && homeOrAway != "away") {
        cout << "First argument must be 'home' or 'away'" << endl;
        return 1;
    }

    ros::init(argc, argv, "QuickDirtyAI");
    ros::NodeHandle n;

    ros::Subscriber robotStateSub = n.subscribe("pred_robot_state_ally2", 10, robot_state_callback);
    ros::Subscriber ballStateSub = n.subscribe("pred_ball_state_ally1", 10, ball_state_callback);
    ros::Subscriber gameStateSub = n.subscribe("game_state", 1, game_state_callback);
    pub = n.advertise<geometry_msgs::Pose2D>("desired_position_ally2", 100);

    ros::spin();
}
