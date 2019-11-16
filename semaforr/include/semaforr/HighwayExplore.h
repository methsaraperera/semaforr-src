#ifndef HIGHWAYEXPLORE_H
#define HIGHWAYEXPLORE_H

/**!
  * HighwayExplore.h
  * 
  * /author: Raj Korpan
  *
  *          Construct Highway Graph
  */

#include <FORRGeometry.h>
#include <Position.h>
#include <FORRAction.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <numeric>
#include <queue>
#include <sensor_msgs/LaserScan.h>

using namespace std;

struct DecisionPoint{
	Position point;
	sensor_msgs::LaserScan laser;
	double farthest_view_left, farthest_view_middle, farthest_view_right;
	double farthest_angle_left, farthest_angle_middle, farthest_angle_right;
	double farthest_distance_left, farthest_distance_middle, farthest_distance_right;
	double middle_distance, middle_distance_min, left_distance, right_distance;
	bool direction;
	Position middle_point;
	Position left_point;
	Position right_point;
	DecisionPoint(): point(Position()), laser(sensor_msgs::LaserScan()) { }
	DecisionPoint(Position p, sensor_msgs::LaserScan ls, bool dir = false){
		point = p;
		laser = ls;
		direction = dir;
		double start_angle = ls.angle_min;
		double increment = ls.angle_increment;
		double r_x = p.getX();
		double r_y = p.getY();
		double r_ang = p.getTheta();
		farthest_view_left = 0;
		farthest_view_middle = 329;
		farthest_view_right = 659;
		farthest_distance_left = 0;
		farthest_distance_middle = 0;
		farthest_distance_right = 0;
		middle_distance = 0;
		middle_distance_min = 0;
		left_distance = 0;
		right_distance = 0;
		double max_value_left = 0;
		double max_value_middle = 0;
		double max_value_right = 0;
		double min_value_middle = 50;
		for(int i = 0; i < ls.ranges.size(); i++){
			double angle = start_angle + r_ang;
			if(i >= 575 and i <= 625){
				left_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_left){
					farthest_distance_left = ls.ranges[i];
					farthest_view_left = i;
					farthest_angle_left = angle;
					max_value_left = ls.ranges[i];
					left_point = Position(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle), 0);
				}
			}
			else if(i >= 35 and i <= 85){
				right_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_right){
					farthest_distance_right = ls.ranges[i];
					farthest_view_right = i;
					farthest_angle_right = angle;
					max_value_right = ls.ranges[i];
					right_point = Position(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle), 0);
				}
			}
			else if(i >= 315 and i <= 345){
				middle_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_middle){
					farthest_distance_middle = ls.ranges[i];
					farthest_view_middle = i;
					farthest_angle_middle = angle;
					max_value_middle = ls.ranges[i];
					middle_point = Position(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle), 0);
				}
				if(ls.ranges[i] < min_value_middle){
					middle_distance_min = ls.ranges[i];
					min_value_middle = ls.ranges[i];
				}
			}
			start_angle = start_angle + increment;
		}
		middle_distance = middle_distance / 31.0;
		left_distance = left_distance / 51.0;
		right_distance = right_distance / 51.0;

	}
	bool operator==(const DecisionPoint p) {
		return (point == p.point);
	}
	bool operator < (const DecisionPoint p) const{
		if(farthest_distance_left < p.farthest_distance_left or farthest_distance_middle < p.farthest_distance_middle or farthest_distance_right < p.farthest_distance_right)
			return true;
		else
			return false;
	}
};

class Highway{
public:
	Highway(DecisionPoint point){
		highway_points.push_back(point);
	}
	vector<DecisionPoint> getHighwayPoints(){return highway_points;}

	void addPointToHighway(DecisionPoint new_point){
		std::vector<DecisionPoint>::const_iterator it;
		it = find(highway_points.begin(), highway_points.end(), new_point);
		if(it == highway_points.end()){
			highway_points.push_back(new_point);
		}
	}

	DecisionPoint getClosestPointOnHighway(Position point){
		DecisionPoint closest;
		double min_distance = 10000000;
		for(int i = 0; i < highway_points.size(); i++){
			double distance = highway_points[i].point.getDistance(point);
			if(distance < min_distance){
				min_distance = distance;
				closest = highway_points[i];
			}
		}
		return closest;
	}

private:
	vector<DecisionPoint> highway_points;
};

class HighwayExplorer{
public:
	HighwayExplorer(int l, int h, double threshold, double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax){
		distance_threshold = threshold;
		length = l;
		height = h;
		numMoves = moveArrMax;
		numRotates = rotateArrMax;
		for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
		for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
		cout << "Highway length " << length << " height " << height << endl;
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			highway_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(0);
			}
			traveled_grid.push_back(col);
		}
		highways = vector<Highway>();
		highways_complete = false;
		go_to_top_point = false;
		too_close = false;
	};
	~HighwayExplorer(){};

	bool getHighwaysComplete(){return highways_complete;}

	FORRAction exploreDecision(Position current_point, sensor_msgs::LaserScan current_laser){
		DecisionPoint current_position = DecisionPoint(current_point, current_laser);
		cout << "current_position " << current_position.point.getX() << " " << current_position.point.getY() << " " << current_position.point.getTheta() << " mid avg " << current_position.middle_distance << " mid min " << current_position.middle_distance_min << " mid max " << current_position.farthest_distance_middle << " left avg " << current_position.left_distance << " left max " << current_position.farthest_distance_left << " right avg " << current_position.right_distance << " right max " << current_position.farthest_distance_right << endl;
		if(highways.size() == 0){
			Highway initial_highway = Highway(current_position);
			highways.push_back(initial_highway);
			last_position = current_position;
			last_highway = 0;
			current_target = current_position.middle_point;
			cout << "current_target " << current_target.getX() << " " << current_target.getY() << endl;
		}
		cout << "last_position " << last_position.point.getX() << " " << last_position.point.getY() << " " << last_position.point.getTheta() << " " << last_highway << endl;

		if(current_target.getDistance(current_position.point) > 1 and go_to_top_point == false){
			cout << "Going to current_target " << current_target.getX() << " " << current_target.getY() << endl;
			highways[last_highway].addPointToHighway(current_position);
			cout << "Current grid value " << highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] << endl;
			if(current_position.farthest_distance_right >= distance_threshold){
				DecisionPoint right_position = current_position;
				right_position.direction = true;
				if(pointAlreadyInStack(right_position) == false){
					cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_right << " view " << current_position.farthest_view_right << endl;
					highway_stack.insert(highway_stack.begin(), right_position);
				}
			}
			if(current_position.farthest_distance_left >= distance_threshold){
				DecisionPoint left_position = current_position;
				left_position.direction = false;
				if(pointAlreadyInStack(left_position) == false){
					cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_left << " view " << current_position.farthest_view_left << endl;
					highway_stack.insert(highway_stack.begin(), left_position);
				}
			}
			if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1)
				highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] = last_highway;
			cout << "Highway grid" << endl;
			for(int i = (int)(current_position.point.getY())-20; i < (int)(current_position.point.getY())+20; i++){
				for(int j = (int)(current_position.point.getX())-20; j < (int)(current_position.point.getX())+20; j++){
					if(i >= 0 and j >= 0 and i < length and j < height){
						if(i == (int)(current_position.point.getY()) and j == (int)(current_position.point.getX())){
							cout << "[" << highway_grid[j][i] << "] "; 
						}
						else{
							cout << highway_grid[j][i] << " ";
						}
					}
				}
				cout << endl;
			}
			return goTowardsPoint(current_position, current_target);
		}
		else if(current_position.middle_distance_min > 1.0 and go_to_top_point == false){
			cout << "current_target achieved, keep going with more space" << endl;
			highways[last_highway].addPointToHighway(current_position);
			cout << "Current grid value " << highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] << endl;
			if(current_position.farthest_distance_right >= distance_threshold){
				DecisionPoint right_position = current_position;
				right_position.direction = true;
				if(pointAlreadyInStack(right_position) == false){
					cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_right << " view " << current_position.farthest_view_right << endl;
					highway_stack.insert(highway_stack.begin(), right_position);
				}
			}
			if(current_position.farthest_distance_left >= distance_threshold){
				DecisionPoint left_position = current_position;
				left_position.direction = false;
				if(pointAlreadyInStack(left_position) == false){
					cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_left << " view " << current_position.farthest_view_left << endl;
					highway_stack.insert(highway_stack.begin(), left_position);
				}
			}
			if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1)
				highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] = last_highway;
			cout << "Highway grid" << endl;
			for(int i = (int)(current_position.point.getY())-20; i < (int)(current_position.point.getY())+20; i++){
				for(int j = (int)(current_position.point.getX())-20; j < (int)(current_position.point.getX())+20; j++){
					if(i >= 0 and j >= 0 and i < length and j < height){
						if(i == (int)(current_position.point.getY()) and j == (int)(current_position.point.getX())){
							cout << "[" << highway_grid[j][i] << "] "; 
						}
						else{
							cout << highway_grid[j][i] << " ";
						}
					}
				}
				cout << endl;
			}
			current_target = current_position.middle_point;
			return goTowardsPoint(current_position, current_target);
		}
		else if(highway_stack.size() > 0 and go_to_top_point == false){
			cout << "Going to top point on stack" << endl;
			top_point = highway_stack[0];
			cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
			highway_stack.erase(highway_stack.begin());
			Highway new_highway = Highway(top_point);
			highways.push_back(new_highway);
			last_highway = last_highway + 1;
			double dist_to_top_point = top_point.point.getDistance(current_position.point);
			cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.point.getTheta() << " top point angles " << top_point.farthest_angle_left << " " << top_point.farthest_angle_right << endl;
			if(dist_to_top_point <= 0.1){
				cout << "Top point achieved, turn towards stretch" << endl;
				if(top_point.direction == true){
					current_target = top_point.right_point;
					return goTowardsPoint(current_position, current_target);
				}
				else{
					current_target = top_point.left_point;
					return goTowardsPoint(current_position, current_target);
				}
			}
			else if(dist_to_top_point <= 1){
				cout << "Top point close, go towards" << endl;
				go_to_top_point = true;
				return goTowardsPoint(current_position, top_point.point);
			}
			else{
				cout << "Top point not in range, go to top point by following grid" << endl;
				go_to_top_point = true;
				findPathOnGrid(current_position, top_point);
				cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
				return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
			}
		}
		else if(go_to_top_point == true){
			cout << "Go to top point" << endl;
			cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
			double dist_to_top_point = top_point.point.getDistance(current_position.point);
			cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.point.getTheta() << " top point angles " << top_point.farthest_angle_left << " " << top_point.farthest_angle_right << endl;
			if(dist_to_top_point <= 0.1){
				cout << "Top point achieved, turn towards stretch" << endl;
				if(top_point.direction == true){
					current_target = top_point.right_point;
					go_to_top_point = false;
					return goTowardsPoint(current_position, current_target);
				}
				else{
					current_target = top_point.left_point;
					go_to_top_point = false;
					return goTowardsPoint(current_position, current_target);
				}
			}
			else if(dist_to_top_point <= 1){
				cout << "Top point close, go towards" << endl;
				return goTowardsPoint(current_position, top_point.point);
			}
			else if(path_to_top_point.size() > 0){
				cout << "Top point not in range, go to top point by following path" << endl;
				waypointAchieved(current_position);
				if(path_to_top_point.size() == 0){
					last_position = current_position;
					return FORRAction(FORWARD, 0);
				}
				else{
					cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
					return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
				}
			}
			else{
				findPathOnGrid(current_position, top_point);
				cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
				return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
			}
		}
		else{
			last_position = current_position;
			return FORRAction(FORWARD, 0);
		}
	}
		// if(current_position.middle_distance >= 1.0 and go_to_top_point == false){
		// 	highways[last_highway].addPointToHighway(current_position);
		// 	cout << "Current grid value " << highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] << endl;
		// 	if(current_position.farthest_distance_right >= distance_threshold and highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1){
		// 	// if(current_position.right_distance >= distance_threshold){
		// 		// cout << "Adding to priority queue " << highway_queue.size() << " distance " << current_position.farthest_distance << " view " << current_position.farthest_view << endl;
		// 		// highway_queue.push(current_position);
		// 		DecisionPoint right_position = current_position;
		// 		right_position.direction = true;
		// 		cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_right << " view " << current_position.farthest_view_right << endl;
		// 		highway_stack.insert(highway_stack.begin(), right_position);
		// 	}
		// 	if(current_position.farthest_distance_left >= distance_threshold and highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1){
		// 	// if(current_position.left_distance >= distance_threshold){
		// 		// cout << "Adding to priority queue " << highway_queue.size() << " distance " << current_position.farthest_distance << " view " << current_position.farthest_view << endl;
		// 		// highway_queue.push(current_position);
		// 		DecisionPoint left_position = current_position;
		// 		left_position.direction = false;
		// 		cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_left << " view " << current_position.farthest_view_left << endl;
		// 		highway_stack.insert(highway_stack.begin(), left_position);
		// 	}
		// 	if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1)
		// 		highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] = last_highway;
		// 	last_position = current_position;
		// 	cout << "Highway grid" << endl;
		// 	for(int i = (int)(current_position.point.getY())-20; i < (int)(current_position.point.getY())+20; i++){
		// 		for(int j = (int)(current_position.point.getX())-20; j < (int)(current_position.point.getX())+20; j++){
		// 			if(i >= 0 and j >= 0 and i < length and j < height){
		// 				if(i == (int)(current_position.point.getY()) and j == (int)(current_position.point.getX())){
		// 					cout << "[" << highway_grid[j][i] << "] "; 
		// 				}
		// 				else{
		// 					cout << highway_grid[j][i] << " ";
		// 				}
		// 			}
		// 		}
		// 		cout << endl;
		// 	}
		// 	double current_direction = current_position.point.getTheta();
		// 	double target_direction = current_position.farthest_angle_middle;
		// 	double required_rotation = target_direction - current_direction;
		// 	if(required_rotation > M_PI)
		// 		required_rotation = required_rotation - (2*M_PI);
		// 	if(required_rotation < -M_PI)
		// 		required_rotation = required_rotation + (2*M_PI);
		// 	cout << "current_direction " << current_direction << " target_direction " << target_direction << " required_rotation " << required_rotation << endl;
		// 	FORRAction decision;
		// 	int rotIntensity=0;
		// 	while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
		// 		rotIntensity++;
		// 	}
		// 	cout << "rotIntensity " << rotIntensity << " Distance diff " << current_position.farthest_distance_middle - current_position.middle_distance << endl;
		// 	if (rotIntensity > 1 and current_position.farthest_distance_middle - current_position.middle_distance > 10 and current_position.middle_distance_min < 1.5) {
		// 		if (required_rotation < 0){
		// 			decision = FORRAction(RIGHT_TURN, 1);
		// 		}
		// 		else {
		// 			decision = FORRAction(LEFT_TURN, 1);
		// 		}
		// 		return decision;
		// 	}
		// 	else{
		// 		return FORRAction(FORWARD, 1);
		// 	}
		// }
		// // else if(highway_queue.size() > 0){
		// // 	DecisionPoint top_point = highway_queue.top();
		// // 	highway_queue.pop();
		// else if(highway_stack.size() > 0 and go_to_top_point == false){
		// 	cout << "Going to top point on stack" << endl;
		// 	top_point = highway_stack[0];
		// 	cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
		// 	highway_stack.erase(highway_stack.begin());
		// 	Highway new_highway = Highway(top_point);
		// 	highways.push_back(new_highway);
		// 	last_position = current_position;
		// 	last_highway = last_highway + 1;
		// 	double dist_to_top_point = top_point.point.getDistance(current_position.point);
		// 	cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.point.getTheta() << " top point angles " << top_point.farthest_angle_left << " " << top_point.farthest_angle_right << endl;
		// 	if(dist_to_top_point <= 0.05 or ((int)(current_position.point.getX()) == (int)(top_point.point.getX()) and (int)(current_position.point.getY()) == (int)(top_point.point.getY()))){
		// 		cout << "Top point in range, turn towards stretch" << endl;
		// 		if(top_point.direction == true){
		// 			double robot_direction = current_position.point.getTheta();
		// 			double goal_direction = top_point.farthest_angle_right;
		// 			double required_rotation = goal_direction - robot_direction;
		// 			if(required_rotation > M_PI)
		// 				required_rotation = required_rotation - (2*M_PI);
		// 			if(required_rotation < -M_PI)
		// 				required_rotation = required_rotation + (2*M_PI);

		// 			if(top_point.farthest_distance_right >= distance_threshold and fabs(required_rotation) > 0.2){
		// 				go_to_top_point = true;
		// 				return turnTowardsPoint(current_position, Position(top_point.point.getX(), top_point.point.getY(), top_point.farthest_angle_right));
		// 			}
		// 			else{
		// 				go_to_top_point = false;
		// 				return FORRAction(FORWARD, 0);
		// 			}
		// 		}
		// 		else{
		// 			double robot_direction = current_position.point.getTheta();
		// 			double goal_direction = top_point.farthest_angle_left;
		// 			double required_rotation = goal_direction - robot_direction;
		// 			if(required_rotation > M_PI)
		// 				required_rotation = required_rotation - (2*M_PI);
		// 			if(required_rotation < -M_PI)
		// 				required_rotation = required_rotation + (2*M_PI);
		// 			if(top_point.farthest_distance_left >= distance_threshold and fabs(required_rotation) > 0.2){
		// 				go_to_top_point = true;
		// 				return turnTowardsPoint(current_position, Position(top_point.point.getX(), top_point.point.getY(), top_point.farthest_angle_left));
		// 			}
		// 			else{
		// 				go_to_top_point = false;
		// 				return FORRAction(FORWARD, 0);
		// 			}
		// 		}	
		// 	}
		// 	else{
		// 		cout << "Top point not in range, go to top point by following grid" << endl;
		// 		go_to_top_point = true;
		// 		findPathOnGrid(current_position, top_point);
		// 		cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
		// 		return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
		// 	}
		// }
		// else if(go_to_top_point == true){
		// 	cout << "Follow grid to top point" << endl;
		// 	last_position = current_position;
		// 	cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
		// 	double dist_to_top_point = top_point.point.getDistance(current_position.point);
		// 	cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.point.getTheta() << " top point angles " << top_point.farthest_angle_left << " " << top_point.farthest_angle_right << endl;
		// 	if(dist_to_top_point <= 0.05 or ((int)(current_position.point.getX()) == (int)(top_point.point.getX()) and (int)(current_position.point.getY()) == (int)(top_point.point.getY()))){
		// 		cout << "Top point in range, turn towards stretch" << endl;
		// 		if(top_point.direction == true){
		// 			double robot_direction = current_position.point.getTheta();
		// 			double goal_direction = top_point.farthest_angle_right;
		// 			double required_rotation = goal_direction - robot_direction;
		// 			if(required_rotation > M_PI)
		// 				required_rotation = required_rotation - (2*M_PI);
		// 			if(required_rotation < -M_PI)
		// 				required_rotation = required_rotation + (2*M_PI);

		// 			if(top_point.farthest_distance_right >= distance_threshold and fabs(required_rotation) > 0.2){
		// 				go_to_top_point = true;
		// 				return turnTowardsPoint(current_position, Position(top_point.point.getX(), top_point.point.getY(), top_point.farthest_angle_right));
		// 			}
		// 			else{
		// 				go_to_top_point = false;
		// 				return FORRAction(FORWARD, 0);
		// 			}
		// 		}
		// 		else{
		// 			double robot_direction = current_position.point.getTheta();
		// 			double goal_direction = top_point.farthest_angle_left;
		// 			double required_rotation = goal_direction - robot_direction;
		// 			if(required_rotation > M_PI)
		// 				required_rotation = required_rotation - (2*M_PI);
		// 			if(required_rotation < -M_PI)
		// 				required_rotation = required_rotation + (2*M_PI);
		// 			if(top_point.farthest_distance_left >= distance_threshold and fabs(required_rotation) > 0.2){
		// 				go_to_top_point = true;
		// 				return turnTowardsPoint(current_position, Position(top_point.point.getX(), top_point.point.getY(), top_point.farthest_angle_left));
		// 			}
		// 			else{
		// 				go_to_top_point = false;
		// 				return FORRAction(FORWARD, 0);
		// 			}
		// 		}	
		// 	}
		// 	else if(path_to_top_point.size() > 0){
		// 		cout << "Top point not in range, go to top point by following path" << endl;
		// 		waypointAchieved(current_position);
		// 		if(path_to_top_point.size() == 0){
		// 			return FORRAction(FORWARD, 0);
		// 		}
		// 		else{
		// 			cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
		// 			return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
		// 		}
		// 	}
		// 	else{
		// 		go_to_top_point = true;
		// 		findPathOnGrid(current_position, top_point);
		// 		cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
		// 		return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
		// 	}
		// }
		// else{
		// 	return FORRAction(FORWARD, 0);
		// }
	// }

	// FORRAction goToPointOnGrid(DecisionPoint current_point, DecisionPoint target_point){
	// 	int current_x = (int)(current_point.point.getX());
	// 	int current_y = (int)(current_point.point.getY());
	// 	int target_x = (int)(target_point.point.getX());
	// 	int target_y = (int)(target_point.point.getY());
	// 	cout << "Current point coordinates in grid " << current_x << " " << current_y << " target_point " << target_x << " " << target_y << endl;
	// 	if((current_x == target_x and current_y == target_y) or current_point.point.getDistance(target_point.point) <= 1){
	// 		double current_direction = current_point.point.getTheta();
	// 		double target_direction = target_point.farthest_angle_middle;
	// 		if(target_point.farthest_distance_left >= distance_threshold){
	// 			target_direction = target_point.farthest_angle_left;
	// 		}
	// 		else if(target_point.farthest_distance_right >= distance_threshold){
	// 			target_direction = target_point.farthest_angle_right;
	// 		}
	// 		double required_rotation = target_direction - current_direction;
	// 		if(required_rotation > M_PI)
	// 			required_rotation = required_rotation - (2*M_PI);
	// 		if(required_rotation < -M_PI)
	// 			required_rotation = required_rotation + (2*M_PI);
	// 		cout << "current_direction " << current_direction << " target_direction " << target_direction << " required_rotation " << required_rotation << endl;
	// 		FORRAction decision;
	// 		int rotIntensity=0;
	// 		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
	// 			rotIntensity++;
	// 		}
	// 		if (rotIntensity > 1) {
	// 			if (required_rotation < 0){
	// 				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
	// 			}
	// 			else {
	// 				decision = FORRAction(LEFT_TURN, rotIntensity-1);
	// 			}
	// 			return decision;
	// 		}
	// 		else{
	// 			go_to_top_point = false;
	// 			return FORRAction(FORWARD, 0);
	// 		}
	// 	}
	// 	cout << "Highway grid" << endl;
	// 	for(int i = current_y-5; i < current_y+5; i++){
	// 		for(int j = current_x-5; j < current_x+5; j++){
	// 			if(i >= 0 and j >= 0 and i < length and j < height){
	// 				if(i == current_y and j == current_x){
	// 					cout << "[" << highway_grid[j][i] << "] "; 
	// 				}
	// 				else{
	// 					cout << highway_grid[j][i] << " ";
	// 				}
	// 			}
	// 		}
	// 		cout << endl;
	// 	}
	// 	cout << "Traveled grid" << endl;
	// 	for(int i = current_y-5; i < current_y+5; i++){
	// 		for(int j = current_x-5; j < current_x+5; j++){
	// 			if(i >= 0 and j >= 0 and i < length and j < height){
	// 				if(i == current_y and j == current_x){
	// 					cout << "[" << traveled_grid[j][i] << "] "; 
	// 				}
	// 				else{
	// 					cout << traveled_grid[j][i] << " ";
	// 				}
	// 			}
	// 		}
	// 		cout << endl;
	// 	}
	// 	traveled_grid[current_x][current_y] = 1;
	// 	if(traveled_grid[current_x+1][current_y] == 1 and traveled_grid[current_x-1][current_y] == 1 and traveled_grid[current_x][current_y+1] == 1 and traveled_grid[current_x][current_y-1] == 1){
	// 		cout << "All neighbors visited already, reset traveled grid" << endl;
	// 		traveled_grid.clear();
	// 		for(int i = 0; i < length; i++){
	// 			vector<int> col;
	// 			for(int j = 0; j < height; j ++){
	// 				col.push_back(0);
	// 			}
	// 			traveled_grid.push_back(col);
	// 		}
	// 	}
	// 	cout << "Current highway " << highway_grid[current_x][current_y] << " Neighbors [" << highway_grid[current_x+1][current_y] << ", " <<  traveled_grid[current_x+1][current_y] << "] [" << highway_grid[current_x-1][current_y] << ", " << traveled_grid[current_x-1][current_y] << "] [" << highway_grid[current_x][current_y+1] << ", " << traveled_grid[current_x][current_y+1] << "] [" << highway_grid[current_x][current_y-1] << ", " << traveled_grid[current_x][current_y-1] << "]" << endl;
	// 	if(highway_grid[current_x+1][current_y] > -1 and traveled_grid[current_x+1][current_y] != 1){
	// 		double dist_to_next = current_point.point.getDistance(Position(current_x+1.5, current_y, 0));
	// 		double current_direction = current_point.point.getTheta();
	// 		double angle_to_next = atan2((current_y - current_point.point.getY()), (current_x+1.5 - current_point.point.getX()));
	// 		double required_rotation = angle_to_next - current_direction;
	// 		if(required_rotation > M_PI)
	// 			required_rotation = required_rotation - (2*M_PI);
	// 		if(required_rotation < -M_PI)
	// 			required_rotation = required_rotation + (2*M_PI);
	// 		FORRAction decision;
	// 		int rotIntensity=0;
	// 		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
	// 			rotIntensity++;
	// 		}
	// 		if (rotIntensity > 1) {
	// 			if (required_rotation < 0){
	// 				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
	// 			}
	// 			else {
	// 				decision = FORRAction(LEFT_TURN, rotIntensity-1);
	// 			}
	// 		}
	// 		else {
	// 			int intensity=0;
	// 			while(dist_to_next > move[intensity] and intensity < numMoves) {
	// 				intensity++;
	// 			}
	// 			if(intensity > 1)
	// 				intensity--;
	// 			if(move[intensity] > current_point.middle_distance){
	// 				intensity = 1;
	// 			}
	// 			decision = FORRAction(FORWARD, intensity);
	// 		}
	// 		return decision;
	// 	}
	// 	else if(highway_grid[current_x-1][current_y] > -1 and traveled_grid[current_x-1][current_y] != 1){
	// 		double dist_to_next = current_point.point.getDistance(Position(current_x-1.5, current_y, 0));
	// 		double current_direction = current_point.point.getTheta();
	// 		double angle_to_next = atan2((current_y - current_point.point.getY()), (current_x-1.5 - current_point.point.getX()));
	// 		double required_rotation = angle_to_next - current_direction;
	// 		if(required_rotation > M_PI)
	// 			required_rotation = required_rotation - (2*M_PI);
	// 		if(required_rotation < -M_PI)
	// 			required_rotation = required_rotation + (2*M_PI);
	// 		FORRAction decision;
	// 		int rotIntensity=0;
	// 		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
	// 			rotIntensity++;
	// 		}
	// 		if (rotIntensity > 1) {
	// 			if (required_rotation < 0){
	// 				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
	// 			}
	// 			else {
	// 				decision = FORRAction(LEFT_TURN, rotIntensity-1);
	// 			}
	// 		}
	// 		else {
	// 			int intensity=0;
	// 			while(dist_to_next > move[intensity] and intensity < numMoves) {
	// 				intensity++;
	// 			}
	// 			if(intensity > 1)
	// 				intensity--;
	// 			if(move[intensity] > current_point.middle_distance){
	// 				intensity = 1;
	// 			}
	// 			decision = FORRAction(FORWARD, intensity);
	// 		}
	// 		return decision;
	// 	}
	// 	else if(highway_grid[current_x][current_y+1] > -1 and traveled_grid[current_x][current_y+1] != 1){
	// 		double dist_to_next = current_point.point.getDistance(Position(current_x, current_y+1.5, 0));
	// 		double current_direction = current_point.point.getTheta();
	// 		double angle_to_next = atan2((current_y+1.5 - current_point.point.getY()), (current_x - current_point.point.getX()));
	// 		double required_rotation = angle_to_next - current_direction;
	// 		if(required_rotation > M_PI)
	// 			required_rotation = required_rotation - (2*M_PI);
	// 		if(required_rotation < -M_PI)
	// 			required_rotation = required_rotation + (2*M_PI);
	// 		FORRAction decision;
	// 		int rotIntensity=0;
	// 		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
	// 			rotIntensity++;
	// 		}
	// 		if (rotIntensity > 1) {
	// 			if (required_rotation < 0){
	// 				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
	// 			}
	// 			else {
	// 				decision = FORRAction(LEFT_TURN, rotIntensity-1);
	// 			}
	// 		}
	// 		else {
	// 			int intensity=0;
	// 			while(dist_to_next > move[intensity] and intensity < numMoves) {
	// 				intensity++;
	// 			}
	// 			if(intensity > 1)
	// 				intensity--;
	// 			if(move[intensity] > current_point.middle_distance){
	// 				intensity = 1;
	// 			}
	// 			decision = FORRAction(FORWARD, intensity);
	// 		}
	// 		return decision;
	// 	}
	// 	else if(highway_grid[current_x][current_y-1] > -1 and traveled_grid[current_x][current_y-1] != 1){
	// 		double dist_to_next = current_point.point.getDistance(Position(current_x, current_y-1.5, 0));
	// 		double current_direction = current_point.point.getTheta();
	// 		double angle_to_next = atan2((current_y-1.5 - current_point.point.getY()), (current_x - current_point.point.getX()));
	// 		double required_rotation = angle_to_next - current_direction;
	// 		if(required_rotation > M_PI)
	// 			required_rotation = required_rotation - (2*M_PI);
	// 		if(required_rotation < -M_PI)
	// 			required_rotation = required_rotation + (2*M_PI);
	// 		FORRAction decision;
	// 		int rotIntensity=0;
	// 		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
	// 			rotIntensity++;
	// 		}
	// 		if (rotIntensity > 1) {
	// 			if (required_rotation < 0){
	// 				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
	// 			}
	// 			else {
	// 				decision = FORRAction(LEFT_TURN, rotIntensity-1);
	// 			}
	// 		}
	// 		else {
	// 			int intensity=0;
	// 			while(dist_to_next > move[intensity] and intensity < numMoves) {
	// 				intensity++;
	// 			}
	// 			if(intensity > 1)
	// 				intensity--;
	// 			if(move[intensity] > current_point.middle_distance){
	// 				intensity = 1;
	// 			}
	// 			decision = FORRAction(FORWARD, intensity);
	// 		}
	// 		return decision;
	// 	}
	// 	else{
	// 		cout << "All neighbors visited already, reset traveled grid" << endl;
	// 		traveled_grid.clear();
	// 		for(int i = 0; i < length; i++){
	// 			vector<int> col;
	// 			for(int j = 0; j < height; j ++){
	// 				col.push_back(0);
	// 			}
	// 			traveled_grid.push_back(col);
	// 		}
	// 		return FORRAction(FORWARD, 0);
	// 	}
	// }

	void findPathOnGrid(DecisionPoint current_point, DecisionPoint target_point){
		int current_x = (int)(current_point.point.getX());
		int current_y = (int)(current_point.point.getY());
		int target_x = (int)(target_point.point.getX());
		int target_y = (int)(target_point.point.getY());
		path_to_top_point.clear();
		cout << "Current point coordinates in grid " << current_x << " " << current_y << " target_point " << target_x << " " << target_y << endl;
		cout << "Current highway " << highway_grid[current_x][current_y] << " Target highway " << highway_grid[target_x][target_y] << endl;
		if(current_x == target_x and current_y == target_y){
			// vector<int> current;
			// current.push_back(current_x);
			// current.push_back(current_y);
			vector<double> current;
			current.push_back(target_point.point.getX());
			current.push_back(target_point.point.getY());
			path_to_top_point.push_back(current);
			return;
		}
		traveled_grid.clear();
		for(int i = 0; i < length; i++){
			vector<int> col;
			for(int j = 0; j < height; j ++){
				col.push_back(0);
			}
			traveled_grid.push_back(col);
		}
		traveled_grid[current_x][current_y] = 1;
		vector< vector<int> > added_points;
		vector< vector<int> > previous_cell;
		for(int i = current_x - 1; i < current_x + 2; i++){
			for(int j = current_y - 1; j < current_y + 2; j++){
				if(highway_grid[i][j] > -1 and traveled_grid[i][j] != 1){
					traveled_grid[i][j] = 1;
					vector<int> new_id;
					new_id.push_back(i);
					new_id.push_back(j);
					added_points.push_back(new_id);
					new_id.push_back(current_x);
					new_id.push_back(current_y);
					previous_cell.push_back(new_id);
					cout << "Neighbor " << i << " " << j << " Previous " << current_x << " " << current_y << endl;
				}
			}
		}
		cout << "Added points " << added_points.size() << endl;
		while(added_points.size() > 0){
			int new_x = added_points[0][0];
			int new_y = added_points[0][1];
			if(new_x == target_x and new_y == target_y){
				cout << "Found target point" << endl;
				break;
			}
			added_points.erase(added_points.begin());
			for(int i = new_x - 1; i < new_x + 2; i++){
				for(int j = new_y - 1; j < new_y + 2; j++){
					if(highway_grid[i][j] > -1 and traveled_grid[i][j] != 1){
						traveled_grid[i][j] = 1;
						vector<int> new_id;
						new_id.push_back(i);
						new_id.push_back(j);
						added_points.push_back(new_id);
						new_id.push_back(new_x);
						new_id.push_back(new_y);
						previous_cell.push_back(new_id);
						cout << "Neighbor " << i << " " << j << " Previous " << new_x << " " << new_y << endl;
					}
				}
			}
			cout << "Added points " << added_points.size() << endl;
		}
		cout << "Previous cell " << previous_cell.size() << endl;
		vector< vector<int> > path;
		if(traveled_grid[target_x][target_y] == 1){
			vector<int> target;
			target.push_back(target_x);
			target.push_back(target_y);
			path.insert(path.begin(), target);
			cout << "Path " << path.size() << " target " << target_x << " " << target_y << endl;
			bool found = true;
			while(found){
				int point_x = path[0][0];
				int point_y = path[0][1];
				found = false;
				cout << "Path " << path.size() << " point " << point_x << " " << point_y << endl; 
				for(int i = 0; i < previous_cell.size(); i++){
					cout << "Match " << previous_cell[i][0] << " " << previous_cell[i][1] << endl;
					if(previous_cell[i][0] == point_x and previous_cell[i][1] == point_y){
						vector<int> new_target;
						new_target.push_back(previous_cell[i][2]);
						new_target.push_back(previous_cell[i][3]);
						path.insert(path.begin(), new_target);
						cout << "Previous " << previous_cell[i][2] << " " << previous_cell[i][3] << endl;
						found = true;
						break;
					}
				}
				cout << path[0][0] << " " << current_x << " " << path[0][1] << " " << current_y << endl;
				if(path[0][0] == current_x and path[0][1] == current_y){
					break;
				}
			}
			// vector<int> current;
			// current.push_back(current_x);
			// current.push_back(current_y);
			// path.insert(path.begin(), current);
			vector< vector<double> > path_from_highway;
			cout << "Path " << path.size() << endl;
			for(int i = 0; i < path.size(); i++){
				cout << path[i][0] << " " << path[i][1] << " " << highway_grid[path[i][0]][path[i][1]] << endl;
				if(highway_grid[path[i][0]][path[i][1]] != -1){
					DecisionPoint closest = highways[highway_grid[path[i][0]][path[i][1]]].getClosestPointOnHighway(Position(path[i][0], path[i][1], 0));
					cout << closest.point.getX() << " " << closest.point.getY() << endl;
					vector<double> closest_point;
					closest_point.push_back(closest.point.getX());
					closest_point.push_back(closest.point.getY());
					path_from_highway.push_back(closest_point);
				}
			}
			// path_to_top_point = path;
			path_to_top_point = path_from_highway;
		}
	}

	void waypointAchieved(DecisionPoint current_position){
		cout << "In waypointAchieved" << endl;
		Position current_waypoint = Position(path_to_top_point[0][0], path_to_top_point[0][1], 0);
		if(current_position.point.getDistance(current_waypoint) <= 0.5){
			cout << "Waypoint Achieved, removing waypoint" << endl;
			path_to_top_point.erase(path_to_top_point.begin());
		}
	}

	FORRAction goTowardsPoint(DecisionPoint current_position, Position target_position){
		cout << "In goTowardsPoint" << endl;
		double distance_from_target = current_position.point.getDistance(target_position);
		cout << "Distance from target : " << distance_from_target << endl;

		// compute the angular difference between the direction to the target and the current robot direction
		double robot_direction = current_position.point.getTheta();
		double goal_direction = atan2((target_position.getY() - current_position.point.getY()), (target_position.getX() - current_position.point.getX()));
		double required_rotation = goal_direction - robot_direction;

		if(required_rotation > M_PI)
			required_rotation = required_rotation - (2*M_PI);
		if(required_rotation < -M_PI)
			required_rotation = required_rotation + (2*M_PI);
		cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
		// if the angular difference is greater than smallest turn possible 
		// pick the right turn to allign itself to the target

		FORRAction decision;
		int rotIntensity=0;
		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
			rotIntensity++;
		}
		cout << "Rotation Intensity : " << rotIntensity << endl;
		if (rotIntensity > 1) {
			if (required_rotation < 0){
				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
			}
			else {
				decision = FORRAction(LEFT_TURN, rotIntensity-1);
			}
		}
		else {
			decision = FORRAction(FORWARD, 1);
			int intensity=0;
			while(distance_from_target > move[intensity] and intensity < numMoves) {
				intensity++;
			}
			if(intensity > 1)
				intensity--;
			// cout << "Move Intensity : " << intensity << endl;

			while(move[intensity] > current_position.middle_distance_min){
				intensity--;
			}
			cout << "Move Intensity : " << intensity << endl;
			// decision = FORRAction(FORWARD, intensity);
		}
		double rotation_from_last = current_position.point.getTheta() - last_position.point.getTheta();
		if(rotation_from_last > M_PI)
			rotation_from_last = rotation_from_last - (2*M_PI);
		if(rotation_from_last < -M_PI)
			rotation_from_last = rotation_from_last + (2*M_PI);
		double distance_from_last = current_position.point.getDistance(last_position.point);
		cout << "distance_from_last " << distance_from_last << " rotation_from_last " << fabs(rotation_from_last) << endl;
		if(too_close == true){
			cout << "Very close to last point, and last point was close on left or right" << endl;
			decision = FORRAction(FORWARD, 1);
			too_close = false;
		}
		else if(current_position.left_distance < 1){
			cout << "Too close on left, turn right" << endl;
			decision = FORRAction(RIGHT_TURN, 3);
			too_close = true;
		}
		else if(current_position.right_distance < 1){
			cout << "Too close on right, turn left" << endl;
			decision = FORRAction(LEFT_TURN, 3);
			too_close = true;
		}
		cout << "Action choosen : " << decision.type << "," << decision.parameter << endl;
		last_position = current_position;
		return decision;
	}

	// FORRAction turnTowardsPoint(DecisionPoint current_position, Position target_position){
	// 	cout << "In turnTowardsPoint" << endl;
	// 	// compute the angular difference between the direction to the target and the current robot direction
	// 	double robot_direction = current_position.point.getTheta();
	// 	double goal_direction = target_position.getTheta();
	// 	double required_rotation = goal_direction - robot_direction;

	// 	if(required_rotation > M_PI)
	// 		required_rotation = required_rotation - (2*M_PI);
	// 	if(required_rotation < -M_PI)
	// 		required_rotation = required_rotation + (2*M_PI);
	// 	cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
	// 	// if the angular difference is greater than smallest turn possible 
	// 	// pick the right turn to allign itself to the target

	// 	FORRAction decision;
	// 	int rotIntensity=0;
	// 	while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
	// 		rotIntensity++;
	// 	}
	// 	cout << "Rotation Intensity : " << rotIntensity << endl;
	// 	if (rotIntensity > 1) {
	// 		if (required_rotation < 0){
	// 			decision = FORRAction(RIGHT_TURN, rotIntensity-1);
	// 		}
	// 		else {
	// 			decision = FORRAction(LEFT_TURN, rotIntensity-1);
	// 		}
	// 	}
	// 	else{
	// 		decision = FORRAction(FORWARD, 0);
	// 	}
	// 	cout << "Action choosen : " << decision.type << "," << decision.parameter << endl;
	// 	return decision;
	// }

	bool pointAlreadyInStack(DecisionPoint new_point){
		if(highway_grid[(int)(new_point.point.getX())][(int)(new_point.point.getY())] == -1){
			return false;
		}
		else{
			for(int i = 0; i < highway_stack.size(); i++){
				double dist = highway_stack[i].point.getDistance(new_point.point);
				double diff_view;
				if(new_point.direction == true){
					diff_view = fabs(highway_stack[i].farthest_view_right - new_point.farthest_view_right);
				}
				else{
					diff_view = fabs(highway_stack[i].farthest_view_left - new_point.farthest_view_left);
				}
				if(dist <= 1 and diff_view <= 20){
					return true;
				}
			}
			return false;
		}
	}


private:
	int length;
	int height;
	double distance_threshold;
	double move[300];  
	double rotate[300];
	int numMoves, numRotates;
	vector< vector<int> > highway_grid;
	vector<Highway> highways;
	// priority_queue<DecisionPoint> highway_queue;
	vector<DecisionPoint> highway_stack;
	DecisionPoint last_position;
	int last_highway;
	Position current_target;
	DecisionPoint top_point;
	vector< vector<int> > traveled_grid;
	vector< vector<double> > path_to_top_point;
	bool highways_complete;
	bool go_to_top_point;
	bool too_close;
};

#endif