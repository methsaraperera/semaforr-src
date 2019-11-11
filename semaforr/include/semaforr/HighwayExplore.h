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
	double middle_distance;
	DecisionPoint(): point(Position()), laser(sensor_msgs::LaserScan()) { }
	DecisionPoint(Position p, sensor_msgs::LaserScan ls){
		point = p;
		laser = ls;
		double start_angle = ls.angle_min;
		double increment = ls.angle_increment;
		double r_ang = p.getTheta();
		farthest_view_left = 0;
		farthest_view_middle = 329;
		farthest_view_right = 659;
		farthest_distance_left = 0;
		farthest_distance_middle = 0;
		farthest_distance_right = 0;
		middle_distance = 0;
		double max_value_left = 0;
		double max_value_middle = 0;
		double max_value_right = 0;
		for(int i = 0; i < ls.ranges.size(); i++){
			double angle = start_angle + r_ang;
			if(i <= 262){
				if(ls.ranges[i] > max_value_left){
					farthest_distance_left = ls.ranges[i];
					farthest_view_left = i;
					farthest_angle_left = angle;
					max_value_left = ls.ranges[i];
				}
			}
			else if(i >= 398){
				if(ls.ranges[i] > max_value_right){
					farthest_distance_right = ls.ranges[i];
					farthest_view_right = i;
					farthest_angle_right = angle;
					max_value_right = ls.ranges[i];
				}
			}
			else{
				middle_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_middle){
					farthest_distance_middle = ls.ranges[i];
					farthest_view_middle = i;
					farthest_angle_middle = angle;
					max_value_middle = ls.ranges[i];
				}
			}
			start_angle = start_angle + increment;
		}
		middle_distance = middle_distance / 135.0;
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
	};
	~HighwayExplorer(){};

	bool getHighwaysComplete(){return highways_complete;}

	FORRAction exploreDecision(Position current_point, sensor_msgs::LaserScan current_laser){
		DecisionPoint current_position = DecisionPoint(current_point, current_laser);
		cout << "current_position " << current_position.point.getX() << " " << current_position.point.getY() << " " << current_position.point.getTheta() << " " << current_position.middle_distance << " " << current_position.farthest_distance_left << " " << current_position.farthest_distance_middle << " " << current_position.farthest_distance_right << endl;
		if(highways.size() == 0){
			Highway initial_highway = Highway(current_position);
			highways.push_back(initial_highway);
			last_position = current_position;
			last_highway = 0;
		}
		cout << "last_position " << last_position.point.getX() << " " << last_position.point.getY() << " " << last_position.point.getTheta() << " " << last_highway << endl;
		if(current_position.middle_distance >= 1.0 and go_to_top_point == false){
			highways[last_highway].addPointToHighway(current_position);
			cout << "Current grid value " << highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] << endl;
			if(current_position.farthest_distance_left >= distance_threshold and highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1){
				// cout << "Adding to priority queue " << highway_queue.size() << " distance " << current_position.farthest_distance << " view " << current_position.farthest_view << endl;
				// highway_queue.push(current_position);
				cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_left << " view " << current_position.farthest_view_left << endl;
				highway_stack.insert(highway_stack.begin(), current_position);
			}
			if(current_position.farthest_distance_right >= distance_threshold and highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1){
				// cout << "Adding to priority queue " << highway_queue.size() << " distance " << current_position.farthest_distance << " view " << current_position.farthest_view << endl;
				// highway_queue.push(current_position);
				cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.farthest_distance_right << " view " << current_position.farthest_view_right << endl;
				highway_stack.insert(highway_stack.begin(), current_position);
			}
			highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] = last_highway;
			last_position = current_position;
			cout << "Highway grid" << endl;
			for(int i = (int)(current_position.point.getY())-10; i < (int)(current_position.point.getY())+10; i++){
				for(int j = (int)(current_position.point.getX())-10; j < (int)(current_position.point.getX())+10; j++){
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
			double current_direction = current_position.point.getTheta();
			double target_direction = current_position.farthest_angle_middle;
			double required_rotation = target_direction - current_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			cout << "current_direction " << current_direction << " target_direction " << target_direction << " required_rotation " << required_rotation << endl;
			FORRAction decision;
			int rotIntensity=0;
			while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
				rotIntensity++;
			}
			if (rotIntensity > 1 and current_position.farthest_distance_middle - current_position.middle_distance > distance_threshold) {
				if (required_rotation < 0){
					decision = FORRAction(RIGHT_TURN, 1);
				}
				else {
					decision = FORRAction(LEFT_TURN, 1);
				}
				return decision;
			}
			else{
				return FORRAction(FORWARD, 1);
			}
		}
		// else if(highway_queue.size() > 0){
		// 	DecisionPoint top_point = highway_queue.top();
		// 	highway_queue.pop();
		else if(highway_stack.size() > 0 and go_to_top_point == false){
			cout << "Going to top point on stack" << endl;
			top_point = highway_stack[0];
			highway_stack.erase(highway_stack.begin());
			Highway new_highway = Highway(top_point);
			highways.push_back(new_highway);
			last_position = current_position;
			last_highway = last_highway + 1;
			double dist_to_top_point = top_point.point.getDistance(current_position.point);
			cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.point.getTheta() << " top point views " << top_point.farthest_view_left << " " << top_point.farthest_view_right << endl;
			// if(dist_to_top_point <= 2){
			// 	cout << "Top point in range, turn towards stretch" << endl;
			// 	if(top_point.farthest_distance_right >= distance_threshold){
			// 		return FORRAction(RIGHT_TURN, 6);
			// 	}
			// 	else{
			// 		return FORRAction(LEFT_TURN, 6);
			// 	}
			// }
			// else{
			// 	cout << "Top point not in range, go to top point by following grid" << endl;
			// 	go_to_top_point = true;
			// 	return goToPointOnGrid(current_position, top_point);
			// }
			go_to_top_point = true;
			return goToPointOnGrid(current_position, top_point);
		}
		else if(go_to_top_point == true){
			cout << "Follow grid to top point" << endl;
			last_position = current_position;
			double dist_to_top_point = top_point.point.getDistance(current_position.point);
			cout << "Distance to top point " << dist_to_top_point << endl;
			// if(dist_to_top_point <= 2){
			// 	cout << "Top point in range, turn towards stretch" << endl;
			// 	if(top_point.farthest_distance_right >= distance_threshold){
			// 		return FORRAction(RIGHT_TURN, 6);
			// 	}
			// 	else{
			// 		return FORRAction(LEFT_TURN, 6);
			// 	}
			// 	go_to_top_point = false;
			// }
			// else{
			// 	cout << "Top point still not in range, go to top point by following grid" << endl;
			// 	return goToPointOnGrid(current_position, top_point);
			// }
			return goToPointOnGrid(current_position, top_point);
		}
		else{
			return FORRAction(FORWARD, 0);
		}
	}

	FORRAction goToPointOnGrid(DecisionPoint current_point, DecisionPoint target_point){
		int current_x = (int)(current_point.point.getX());
		int current_y = (int)(current_point.point.getY());
		int target_x = (int)(target_point.point.getX());
		int target_y = (int)(target_point.point.getY());
		cout << "Current point coordinates in grid" << current_x << " " << current_y << " target_point " << target_x << " " << target_y << endl;
		if((current_x == target_x and current_y == target_y) or current_point.point.getDistance(target_point.point) <= 1){
			double current_direction = current_point.point.getTheta();
			double target_direction = target_point.farthest_angle_middle;
			if(target_point.farthest_distance_left >= distance_threshold){
				target_direction = target_point.farthest_angle_left;
			}
			else if(target_point.farthest_distance_right >= distance_threshold){
				target_direction = target_point.farthest_angle_right;
			}
			double required_rotation = target_direction - current_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			cout << "current_direction " << current_direction << " target_direction " << target_direction << " required_rotation " << required_rotation << endl;
			FORRAction decision;
			int rotIntensity=0;
			while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
				rotIntensity++;
			}
			if (rotIntensity > 1) {
				if (required_rotation < 0){
					decision = FORRAction(RIGHT_TURN, rotIntensity-1);
				}
				else {
					decision = FORRAction(LEFT_TURN, rotIntensity-1);
				}
				return decision;
			}
			else{
				go_to_top_point = false;
				return FORRAction(FORWARD, 0);
			}
		}
		cout << "Highway grid" << endl;
		for(int i = current_y-5; i < current_y+5; i++){
			for(int j = current_x-5; j < current_x+5; j++){
				if(i >= 0 and j >= 0 and i < length and j < height){
					if(i == current_y and j == current_x){
						cout << "[" << highway_grid[j][i] << "] "; 
					}
					else{
						cout << highway_grid[j][i] << " ";
					}
				}
			}
			cout << endl;
		}
		cout << "Traveled grid" << endl;
		for(int i = current_y-5; i < current_y+5; i++){
			for(int j = current_x-5; j < current_x+5; j++){
				if(i >= 0 and j >= 0 and i < length and j < height){
					if(i == current_y and j == current_x){
						cout << "[" << traveled_grid[j][i] << "] "; 
					}
					else{
						cout << traveled_grid[j][i] << " ";
					}
				}
			}
			cout << endl;
		}
		traveled_grid[current_x][current_y] = 1;
		if(traveled_grid[current_x+1][current_y] == 1 and traveled_grid[current_x-1][current_y] == 1 and traveled_grid[current_x][current_y+1] == 1 and traveled_grid[current_x][current_y-1] == 1){
			cout << "All neighbors visited already, reset traveled grid" << endl;
			traveled_grid.clear();
			for(int i = 0; i < length; i++){
				vector<int> col;
				for(int j = 0; j < height; j ++){
					col.push_back(0);
				}
				traveled_grid.push_back(col);
			}
		}
		cout << "Current highway " << highway_grid[current_x][current_y] << " Neighbors [" << highway_grid[current_x+1][current_y] << ", " <<  traveled_grid[current_x+1][current_y] << "] [" << highway_grid[current_x-1][current_y] << ", " << traveled_grid[current_x-1][current_y] << "] [" << highway_grid[current_x][current_y+1] << ", " << traveled_grid[current_x][current_y+1] << "] [" << highway_grid[current_x][current_y-1] << ", " << traveled_grid[current_x][current_y-1] << "]" << endl;
		if(highway_grid[current_x+1][current_y] > -1 and traveled_grid[current_x+1][current_y] != 1){
			double dist_to_next = current_point.point.getDistance(Position(current_x+1.5, current_y, 0));
			double current_direction = current_point.point.getTheta();
			double angle_to_next = atan2((current_y - current_point.point.getY()), (current_x+1.5 - current_point.point.getX()));
			double required_rotation = angle_to_next - current_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			FORRAction decision;
			int rotIntensity=0;
			while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
				rotIntensity++;
			}
			if (rotIntensity > 1) {
				if (required_rotation < 0){
					decision = FORRAction(RIGHT_TURN, rotIntensity-1);
				}
				else {
					decision = FORRAction(LEFT_TURN, rotIntensity-1);
				}
			}
			else {
				int intensity=0;
				while(dist_to_next > move[intensity] and intensity < numMoves) {
					intensity++;
				}
				if(intensity > 1)
					intensity--;
				if(move[intensity] > current_point.middle_distance){
					intensity = 1;
				}
				decision = FORRAction(FORWARD, intensity);
			}
			return decision;
		}
		else if(highway_grid[current_x-1][current_y] > -1 and traveled_grid[current_x-1][current_y] != 1){
			double dist_to_next = current_point.point.getDistance(Position(current_x-1.5, current_y, 0));
			double current_direction = current_point.point.getTheta();
			double angle_to_next = atan2((current_y - current_point.point.getY()), (current_x-1.5 - current_point.point.getX()));
			double required_rotation = angle_to_next - current_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			FORRAction decision;
			int rotIntensity=0;
			while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
				rotIntensity++;
			}
			if (rotIntensity > 1) {
				if (required_rotation < 0){
					decision = FORRAction(RIGHT_TURN, rotIntensity-1);
				}
				else {
					decision = FORRAction(LEFT_TURN, rotIntensity-1);
				}
			}
			else {
				int intensity=0;
				while(dist_to_next > move[intensity] and intensity < numMoves) {
					intensity++;
				}
				if(intensity > 1)
					intensity--;
				if(move[intensity] > current_point.middle_distance){
					intensity = 1;
				}
				decision = FORRAction(FORWARD, intensity);
			}
			return decision;
		}
		else if(highway_grid[current_x][current_y+1] > -1 and traveled_grid[current_x][current_y+1] != 1){
			double dist_to_next = current_point.point.getDistance(Position(current_x, current_y+1.5, 0));
			double current_direction = current_point.point.getTheta();
			double angle_to_next = atan2((current_y+1.5 - current_point.point.getY()), (current_x - current_point.point.getX()));
			double required_rotation = angle_to_next - current_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			FORRAction decision;
			int rotIntensity=0;
			while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
				rotIntensity++;
			}
			if (rotIntensity > 1) {
				if (required_rotation < 0){
					decision = FORRAction(RIGHT_TURN, rotIntensity-1);
				}
				else {
					decision = FORRAction(LEFT_TURN, rotIntensity-1);
				}
			}
			else {
				int intensity=0;
				while(dist_to_next > move[intensity] and intensity < numMoves) {
					intensity++;
				}
				if(intensity > 1)
					intensity--;
				if(move[intensity] > current_point.middle_distance){
					intensity = 1;
				}
				decision = FORRAction(FORWARD, intensity);
			}
			return decision;
		}
		else if(highway_grid[current_x][current_y-1] > -1 and traveled_grid[current_x][current_y-1] != 1){
			double dist_to_next = current_point.point.getDistance(Position(current_x, current_y-1.5, 0));
			double current_direction = current_point.point.getTheta();
			double angle_to_next = atan2((current_y-1.5 - current_point.point.getY()), (current_x - current_point.point.getX()));
			double required_rotation = angle_to_next - current_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			FORRAction decision;
			int rotIntensity=0;
			while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
				rotIntensity++;
			}
			if (rotIntensity > 1) {
				if (required_rotation < 0){
					decision = FORRAction(RIGHT_TURN, rotIntensity-1);
				}
				else {
					decision = FORRAction(LEFT_TURN, rotIntensity-1);
				}
			}
			else {
				int intensity=0;
				while(dist_to_next > move[intensity] and intensity < numMoves) {
					intensity++;
				}
				if(intensity > 1)
					intensity--;
				if(move[intensity] > current_point.middle_distance){
					intensity = 1;
				}
				decision = FORRAction(FORWARD, intensity);
			}
			return decision;
		}
		else{
			cout << "All neighbors visited already, reset traveled grid" << endl;
			traveled_grid.clear();
			for(int i = 0; i < length; i++){
				vector<int> col;
				for(int j = 0; j < height; j ++){
					col.push_back(0);
				}
				traveled_grid.push_back(col);
			}
			return FORRAction(FORWARD, 0);
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
	DecisionPoint top_point;
	vector< vector<int> > traveled_grid;
	bool highways_complete;
	bool go_to_top_point;
};

#endif