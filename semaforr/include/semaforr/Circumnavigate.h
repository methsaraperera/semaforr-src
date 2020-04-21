#ifndef CIRCUMNAVIGATE_H
#define CIRCUMNAVIGATE_H

/**!
  * Circumnavigate.h
  * 
  * /author: Raj Korpan
  *
  *          Circumnavigate to get to goal
  */

#include <vector>
#include "FORRAction.h"
#include "Beliefs.h"

using namespace std;

struct DPoint{
	Position point;
	Position middle_point;
	DPoint(): point(Position()), middle_point(Position()) { }
	DPoint(Position p, Position m_p){
		point = p;
		middle_point = m_p;
	}
	bool operator==(const DPoint p) {
		return (point == p.point and middle_point == p.middle_point);
	}
};

class Circumnavigate{
public:
	Circumnavigate(int l, int h, double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax, Beliefs *b, PathPlanner *s){
		length = l;
		height = h;
		numMoves = moveArrMax;
		numRotates = rotateArrMax;
		for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
		for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
		beliefs = b;
		skeleton_planner = s;
		for(int i = 0; i < length; i++){
			vector<int> col;
			for(int j = 0; j < height; j ++){
				col.push_back(-1);
			}
			visited_grid.push_back(col);
		}
		followingCurrentDirection = false;
		goToStart = false;
		currentDirectionChanged = false;
		foundAlignmentPoint = false;
		currentDPoint = DPoint(Position(0,0,0),Position(0,0,0));
		alignmentPoint = Position(0,0,0);
		alignmentDirection = 0;
	};
	~Circumnavigate(){};

	void resetCircumnavigate(){
		visited_grid.clear();
		for(int i = 0; i < length; i++){
			vector<int> col;
			for(int j = 0; j < height; j ++){
				col.push_back(-1);
			}
			visited_grid.push_back(col);
		}
		followingCurrentDirection = false;
		goToStart = false;
		currentDirectionChanged = false;
		foundAlignmentPoint = false;
		currentDPoint = DPoint(Position(0,0,0),Position(0,0,0));
		alignmentPoint = Position(0,0,0);
		alignmentDirection = 0;
		east_stack.clear();
		west_stack.clear();
		north_stack.clear();
		south_stack.clear();
	}

	void addToStack(Position new_pose, sensor_msgs::LaserScan new_laser){
		double start_angle = new_laser.angle_min;
		double increment = new_laser.angle_increment;
		double r_x = new_pose.getX();
		double r_y = new_pose.getY();
		double r_ang = new_pose.getTheta();
		double min_distance = 1000;
		double left_x = 0, left_y = 0, right_x = 0, right_y = 0, middle_x = 0, middle_y = 0;
		for(int i = 0; i < new_laser.ranges.size(); i++){
			double angle = start_angle + r_ang;
			if(i >= 580 and i <= 620){
				left_x += r_x + new_laser.ranges[i]*cos(angle);
				left_y += r_y + new_laser.ranges[i]*sin(angle);
			}
			else if(i >= 40 and i <= 80){
				right_x += r_x + new_laser.ranges[i]*cos(angle);
				right_y += r_y + new_laser.ranges[i]*sin(angle);
			}
			else if(i >= 295 and i <= 365){
				middle_x += r_x + new_laser.ranges[i]*cos(angle);
				middle_y += r_y + new_laser.ranges[i]*sin(angle);
			}
			start_angle = start_angle + increment;
			if(new_laser.ranges[i] < min_distance){
				min_distance = new_laser.ranges[i];
			}
		}
		if(left_x < 0)
			left_x = 0;
		if(left_y < 0)
			left_y = 0;
		if(right_x < 0)
			right_x = 0;
		if(right_y < 0)
			right_y = 0;
		if(middle_x < 0)
			middle_x = 0;
		if(middle_y < 0)
			middle_y = 0;
		Position left_point = Position(left_x/ 41.0, left_y/ 41.0, 0);
		Position right_point = Position(right_x/ 41.0, right_y/ 41.0, 0);
		Position middle_point = Position(middle_x/ 71.0, middle_y/ 71.0, 0);
		if(new_pose.getTheta() >= -M_PI/18.0 and new_pose.getTheta() < M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to east " << middle_point.getX() << " " << middle_point.getY() << " added to north " << left_point.getX() << " " << left_point.getY() << " added to south " << right_point.getX() << " " << right_point.getY() << endl;
			if(visited_grid[(int)(middle_point.getX())][(int)(middle_point.getY())] < 0)
				east_stack.insert(east_stack.begin(), DPoint(new_pose, middle_point));
			if(visited_grid[(int)(left_point.getX())][(int)(left_point.getY())] < 0)
				north_stack.insert(north_stack.begin(), DPoint(new_pose, left_point));
			if(visited_grid[(int)(right_point.getX())][(int)(right_point.getY())] < 0)
				south_stack.insert(south_stack.begin(), DPoint(new_pose, right_point));
		}
		else if(new_pose.getTheta() >= M_PI/2.0-M_PI/18.0 and new_pose.getTheta() < M_PI/2.0+M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to north " << middle_point.getX() << " " << middle_point.getY() << " added to west " << left_point.getX() << " " << left_point.getY() << " added to east " << right_point.getX() << " " << right_point.getY() << endl;
			if(visited_grid[(int)(middle_point.getX())][(int)(middle_point.getY())] < 0)
				north_stack.insert(north_stack.begin(), DPoint(new_pose, middle_point));
			if(visited_grid[(int)(left_point.getX())][(int)(left_point.getY())] < 0)
				west_stack.insert(west_stack.begin(), DPoint(new_pose, left_point));
			if(visited_grid[(int)(right_point.getX())][(int)(right_point.getY())] < 0)
				east_stack.insert(east_stack.begin(), DPoint(new_pose, right_point));
		}
		else if(new_pose.getTheta() >= M_PI-M_PI/18.0 or new_pose.getTheta() < -M_PI+M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to west " << middle_point.getX() << " " << middle_point.getY() << " added to south " << left_point.getX() << " " << left_point.getY() << " added to north " << right_point.getX() << " " << right_point.getY() << endl;
			if(visited_grid[(int)(middle_point.getX())][(int)(middle_point.getY())] < 0)
				west_stack.insert(west_stack.begin(), DPoint(new_pose, middle_point));
			if(visited_grid[(int)(right_point.getX())][(int)(right_point.getY())] < 0)
				north_stack.insert(north_stack.begin(), DPoint(new_pose, right_point));
			if(visited_grid[(int)(left_point.getX())][(int)(left_point.getY())] < 0)
				south_stack.insert(south_stack.begin(), DPoint(new_pose, left_point));
		}
		else if(new_pose.getTheta() >= -M_PI/2.0-M_PI/18.0 and new_pose.getTheta() < -M_PI/2.0+M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to south " << middle_point.getX() << " " << middle_point.getY() << " added to east " << left_point.getX() << " " << left_point.getY() << " added to west " << right_point.getX() << " " << right_point.getY() << endl;
			if(visited_grid[(int)(middle_point.getX())][(int)(middle_point.getY())] < 0)
				south_stack.insert(south_stack.begin(), DPoint(new_pose, middle_point));
			if(visited_grid[(int)(right_point.getX())][(int)(right_point.getY())] < 0)
				west_stack.insert(west_stack.begin(), DPoint(new_pose, right_point));
			if(visited_grid[(int)(left_point.getX())][(int)(left_point.getY())] < 0)
				east_stack.insert(east_stack.begin(), DPoint(new_pose, left_point));
		}
		cout << "min_distance " << min_distance << endl;
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY())] = 1;
		visited_grid[(int)(new_pose.getX()+min_distance)][(int)(new_pose.getY())] = 1;
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY()+min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()-min_distance)][(int)(new_pose.getY())] = 1;
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY()-min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()+min_distance)][(int)(new_pose.getY()+min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()+min_distance)][(int)(new_pose.getY()-min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()-min_distance)][(int)(new_pose.getY()+min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()-min_distance)][(int)(new_pose.getY()-min_distance)] = 1;
		cout << "east_stack " << east_stack.size() << " west_stack " << west_stack.size() << " north_stack " << north_stack.size() << " south_stack " << south_stack.size() << endl;
		cout << "Visited grid" << endl;
		for(int i = 0; i < visited_grid[0].size(); i++){
			for(int j = 0; j < visited_grid.size(); j++){
				cout << visited_grid[j][i] << " ";
			}
			cout << endl;
		}
	}

	bool advisorCircumnavigate(FORRAction *decision){
		bool decisionMade = false;
		CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
		Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
		cout << "Target = " << task.get_x() << " " << task.get_y() << " Current Position = " << currentPosition.getX() << " " << currentPosition.getY() << " " << currentPosition.getTheta() << endl;
		CartesianPoint potentialAlignmentPoint1 = CartesianPoint(task.get_x(), currentPosition.getY());
		CartesianPoint potentialAlignmentPoint1 = CartesianPoint(currentPosition.getX(), task.get_y());
		bool can_see_1 = beliefs->getAgentState()->canSeePoint(potentialAlignmentPoint1, 25);
		bool can_see_2 = beliefs->getAgentState()->canSeePoint(potentialAlignmentPoint1, 25);
		if(can_see_1){
			foundAlignmentPoint = true;
			alignmentPoint = Position(potentialAlignmentPoint1.get_x(), potentialAlignmentPoint1.get_y(), 0);
		}
		else if(can_see_2){
			foundAlignmentPoint = true;
			alignmentPoint = Position(potentialAlignmentPoint2.get_x(), potentialAlignmentPoint2.get_y(), 0);
		}
		if(foundAlignmentPoint){
			set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
			double robot_direction = currentPosition.getTheta();
			double goal_direction = atan2((task.get_y() - currentPosition.getY()), (task.get_x() - currentPosition.getX()));
			double required_rotation = goal_direction - robot_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			if(currentPosition.get_distance(alignmentPoint) > 0.5){
				CartesianPoint align(alignmentPoint.getX(),alignmentPoint.getY());
				(*decision) = beliefs->getAgentState()->moveTowards(align);
				FORRAction forward = beliefs->getAgentState()->maxForwardAction();
				Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
				if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
					if(vetoed_actions->find(*decision) != vetoed_actions->end()){
						decisionMade = false;
					}
					else{
						cout << "Circumnavigate advisor to take decision" << endl;
						decisionMade = true;
					}
				}
			}
			else if(fabs(required_rotation) > 0.174532925){
				int rotIntensity=0;
				while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
					rotIntensity++;
				}
				int max_allowed = maxForwardAction().parameter;
				if (rotIntensity > 1) {
					if (required_rotation < 0){
						(*decision) = FORRAction(RIGHT_TURN, rotIntensity-1);
					}
					else {
						(*decision) = FORRAction(LEFT_TURN, rotIntensity-1);
					}
					if(vetoed_actions->find(*decision) != vetoed_actions->end()){
						decisionMade = false;
					}
					else{
						cout << "Circumnavigate advisor to take decision" << endl;
						decisionMade = true;
					}
				}
			}
			else{
				vector<CartesianPoint> currentLaserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
				vector < vector<int> > laserGrid;
				for(int i = 0; i < length; i++){
					vector<int> col;
					for(int j = 0; j < height; j ++){
						col.push_back(0);
					}
					laserGrid.push_back(col);
				}
				for(int i = 0; i < currentLaserEndpoints.size(); i++){
					laserGrid[(int)(currentLaserEndpoints[i].get_x())][(int)(currentLaserEndpoints[i].get_y())] = 1;
				}
				if(fabs(robot_direction) > M_PI/4 and fabs(robot_direction) < 3*M_PI/4){
					int row = (int)(currentLaserEndpoints[currentLaserEndpoints.size()].get_x());
					int start_col = (int)(currentLaserEndpoints[currentLaserEndpoints.size()].get_y());
					int end_col_r = start_col;
					int end_col_l = start_col;
					int value = laserGrid[row][start_col];
					while(value > 0){
						end_col_r = end_col_r + 1;
						value = laserGrid[row][end_col_r];
					}
					end_col_r = end_col_r - 1;
					value = laserGrid[row][start_col];
					while(value > 0){
						end_col_l = end_col_l - 1;
						value = laserGrid[row][end_col_l];
					}
					end_col_l = end_col_l + 1;
					if(robot_direction > 0){
						if(laserGrid[row-1][end_col_l] == 1 and laserGrid[row-1][end_col_r] == 1){
							// closed in
						}
					}
					else{
						if(laserGrid[row+1][end_col_l] == 1 and laserGrid[row+1][end_col_r] == 1){
							// closed in
						}
					}
				}
				else{
					int start_row = (int)(currentLaserEndpoints[currentLaserEndpoints.size()].get_x());
					int col = (int)(currentLaserEndpoints[currentLaserEndpoints.size()].get_y());
					int end_row_r = start_row;
					int end_row_l = start_row;
					int value = laserGrid[start_row][col];
					while(value > 0){
						end_row_r = end_row_r + 1;
						value = laserGrid[end_row_r][col];
					}
					end_row_r = end_row_r - 1;
					value = laserGrid[start_row][col];
					while(value > 0){
						end_row_l = end_row_l - 1;
						value = laserGrid[end_row_l][col];
					}
					end_row_l = end_row_l + 1;
					if(robot_direction > 0){
						if(laserGrid[end_row_l][col-1] == 1 and laserGrid[end_row_r][col-1] == 1){
							// closed in
						}
					}
					else{
						if(laserGrid[end_row_l][col+1] == 1 and laserGrid[end_row_r][col+1] == 1){
							// closed in
						}
					}
				}
			}
		}







		cout << "currentDPoint " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
		addToStack(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
		int currdir = beliefs->getAgentState()->getCurrentDirection();
		double x_diff = task.get_x() - currentPosition.getX();
		double y_diff = task.get_y() - currentPosition.getY();
		cout << "x_diff " << x_diff << " y_diff " << y_diff << endl;
		if(abs(x_diff) >= abs(y_diff) and abs(x_diff) > 0.5 and abs(y_diff) > 0.5){
			if(x_diff > 0.5){
				beliefs->getAgentState()->setCurrentDirection(1); // east
			}
			else if(x_diff < 0.5){
				beliefs->getAgentState()->setCurrentDirection(2); // west
			}
			// else{
			// 	foundAlignmentPoint = true;
			// 	alignmentPoint = currentPosition;
			// 	if(rand() / double(RAND_MAX) > 0.5){
			// 		beliefs->getAgentState()->setCurrentDirection(1);
			// 		alignmentDirection = 1;
			// 	}
			// 	else{
			// 		beliefs->getAgentState()->setCurrentDirection(2);
			// 		alignmentDirection = 2;
			// 	}
			// }
			if(y_diff > 0.5){
				beliefs->getAgentState()->setDesiredDirection(3); // north
			}
			else if(y_diff < 0.5){
				beliefs->getAgentState()->setDesiredDirection(4); // south
			}
			// else{
			// 	foundAlignmentPoint = true;
			// 	alignmentPoint = currentPosition;
			// 	if(rand() / double(RAND_MAX) > 0.5){
			// 		beliefs->getAgentState()->setDesiredDirection(3);
			// 	}
			// 	else{
			// 		beliefs->getAgentState()->setDesiredDirection(4);
			// 	}
			// }
		}
		else if(abs(x_diff) < abs(y_diff) and abs(x_diff) > 0.5 and abs(y_diff) > 0.5){
			if(x_diff > 0.5){
				beliefs->getAgentState()->setDesiredDirection(1);
			}
			else if(x_diff < 0.5){
				beliefs->getAgentState()->setDesiredDirection(2);
			}
			// else{
			// 	foundAlignmentPoint = true;
			// 	alignmentPoint = currentPosition;
			// 	if(rand() / double(RAND_MAX) > 0.5){
			// 		beliefs->getAgentState()->setDesiredDirection(1);
			// 	}
			// 	else{
			// 		beliefs->getAgentState()->setDesiredDirection(2);
			// 	}
			// }
			if(y_diff > 0.5){
				beliefs->getAgentState()->setCurrentDirection(3);
			}
			else if(y_diff < 0.5){
				beliefs->getAgentState()->setCurrentDirection(4);
			}
			// else{
			// 	foundAlignmentPoint = true;
			// 	alignmentPoint = currentPosition;
			// 	if(rand() / double(RAND_MAX) > 0.5){
			// 		beliefs->getAgentState()->setCurrentDirection(3);
			// 		alignmentDirection = 3;
			// 	}
			// 	else{
			// 		beliefs->getAgentState()->setCurrentDirection(4);
			// 		alignmentDirection = 4;
			// 	}
			// }
		}
		else if(abs(x_diff) > 0.5 and abs(y_diff) <= 0.5){
			if(x_diff > 0.5){
				beliefs->getAgentState()->setDesiredDirection(1);
			}
			else if(x_diff < 0.5){
				beliefs->getAgentState()->setDesiredDirection(2);
			}
			foundAlignmentPoint = true;
			alignmentPoint = currentPosition;
			if(rand() / double(RAND_MAX) > 0.5){
				beliefs->getAgentState()->setCurrentDirection(3);
				alignmentDirection = 3;
			}
			else{
				beliefs->getAgentState()->setCurrentDirection(4);
				alignmentDirection = 4;
			}
		}
		else if(abs(x_diff) <= 0.5 and abs(y_diff) > 0.5){
			foundAlignmentPoint = true;
			alignmentPoint = currentPosition;
			if(rand() / double(RAND_MAX) > 0.5){
				beliefs->getAgentState()->setCurrentDirection(1);
				alignmentDirection = 1;
			}
			else{
				beliefs->getAgentState()->setCurrentDirection(2);
				alignmentDirection = 2;
			}
			if(y_diff > 0.5){
				beliefs->getAgentState()->setDesiredDirection(3); // north
			}
			else if(y_diff < 0.5){
				beliefs->getAgentState()->setDesiredDirection(4); // south
			}
		}
		else{
			if(rand() / double(RAND_MAX) > 0.5){
				if(rand() / double(RAND_MAX) > 0.5){
					beliefs->getAgentState()->setCurrentDirection(1);
				}
				else{
					beliefs->getAgentState()->setCurrentDirection(2);
				}
				if(rand() / double(RAND_MAX) > 0.5){
					beliefs->getAgentState()->setDesiredDirection(3);
				}
				else{
					beliefs->getAgentState()->setDesiredDirection(4);
				}
			}
			else{
				if(rand() / double(RAND_MAX) > 0.5){
					beliefs->getAgentState()->setCurrentDirection(3);
				}
				else{
					beliefs->getAgentState()->setCurrentDirection(4);
				}
				if(rand() / double(RAND_MAX) > 0.5){
					beliefs->getAgentState()->setDesiredDirection(1);
				}
				else{
					beliefs->getAgentState()->setDesiredDirection(2);
				}
			}
			if(abs(x_diff) <= 0.5 or abs(y_diff) <= 0.5){
				foundAlignmentPoint = true;
				alignmentPoint = currentPosition;
				alignmentDirection = beliefs->getAgentState()->getCurrentDirection();
			}
		}
		cout << "Current Direction " << beliefs->getAgentState()->getCurrentDirection() << " Desired Direction " << beliefs->getAgentState()->getDesiredDirection() << endl;
		cout << "foundAlignmentPoint " << foundAlignmentPoint << " alignmentPoint " << alignmentPoint.getX() << " " << alignmentPoint.getY() << endl;
		if(currdir != beliefs->getAgentState()->getCurrentDirection()){
			currentDirectionChanged = true;
		}
		else{
			currentDirectionChanged = false;
		}
		if(currentPosition.getDistance(currentDPoint.middle_point) <= 0.75 or beliefs->getAgentState()->maxForwardAction().parameter == 0){
			followingCurrentDirection = false;
		}
		cout << "Previous Current Direction " << currdir << " currentDirectionChanged " << currentDirectionChanged << endl;
		cout << "IsPLanComplete " << beliefs->getAgentState()->getCurrentTask()->getIsPlanComplete() << " followingCurrentDirection " << followingCurrentDirection << endl;
		set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
		if(beliefs->getAgentState()->getCurrentTask()->getIsPlanComplete() == true and (followingCurrentDirection == false or currentDirectionChanged == true)){
			if(beliefs->getAgentState()->getCurrentDirection() == 1){
				cout << "east_stack " << east_stack.size() << endl;
				if(east_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0 and east_stack.size() > 0){
						new_current = east_stack[0];
						cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
						east_stack.erase(east_stack.begin());
						end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
						cout << "end_label " << end_label << endl;
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
						followingCurrentDirection = true;
						goToStart = true;
					}
				}
			}
			else if(beliefs->getAgentState()->getCurrentDirection() == 2){
				cout << "west_stack " << west_stack.size() << endl;
				if(west_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0 and west_stack.size() > 0){
						new_current = west_stack[0];
						cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
						west_stack.erase(west_stack.begin());
						end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
						cout << "end_label " << end_label << endl;
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
						followingCurrentDirection = true;
						goToStart = true;
					}
				}
			}
			else if(beliefs->getAgentState()->getCurrentDirection() == 3){
				cout << "north_stack " << north_stack.size() << endl;
				if(north_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0 and north_stack.size() > 0){
						new_current = north_stack[0];
						cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
						north_stack.erase(north_stack.begin());
						end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
						cout << "end_label " << end_label << endl;
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
						followingCurrentDirection = true;
						goToStart = true;
					}
				}
			}
			else if(beliefs->getAgentState()->getCurrentDirection() == 4){
				cout << "south_stack " << south_stack.size() << endl;
				if(south_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0 and south_stack.size() > 0){
						new_current = south_stack[0];
						cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
						south_stack.erase(south_stack.begin());
						end_label = visited_grid[(int)(currentDPoint.middle_point.getX())][(int)(currentDPoint.middle_point.getY())];
						cout << "end_label " << end_label << endl;
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
						followingCurrentDirection = true;
						goToStart = true;
					}
				}
			}
			if(followingCurrentDirection == false){
				if(beliefs->getAgentState()->getDesiredDirection() == 1){
					cout << "east_stack " << east_stack.size() << endl;
					if(east_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0 and east_stack.size() > 0){
							new_current = east_stack[0];
							cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
							east_stack.erase(east_stack.begin());
							end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
							cout << "end_label " << end_label << endl;
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
							followingCurrentDirection = true;
							goToStart = true;
						}
					}
				}
				else if(beliefs->getAgentState()->getDesiredDirection() == 2){
					cout << "west_stack " << west_stack.size() << endl;
					if(west_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0 and west_stack.size() > 0){
							new_current = west_stack[0];
							cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
							west_stack.erase(west_stack.begin());
							end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
							cout << "end_label " << end_label << endl;
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
							followingCurrentDirection = true;
							goToStart = true;
						}
					}
				}
				else if(beliefs->getAgentState()->getDesiredDirection() == 3){
					cout << "north_stack " << north_stack.size() << endl;
					if(north_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0 and north_stack.size() > 0){
							new_current = north_stack[0];
							cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
							north_stack.erase(north_stack.begin());
							end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
							cout << "end_label " << end_label << endl;
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
							followingCurrentDirection = true;
							goToStart = true;
						}
					}
				}
				else if(beliefs->getAgentState()->getDesiredDirection() == 4){
					cout << "south_stack " << south_stack.size() << endl;
					if(south_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0 and south_stack.size() > 0){
							new_current = south_stack[0];
							cout << "Potential new_current " << new_current.point.getX() << " " << new_current.point.getY() << endl;
							south_stack.erase(south_stack.begin());
							end_label = visited_grid[(int)(currentDPoint.middle_point.getX())][(int)(currentDPoint.middle_point.getY())];
							cout << "end_label " << end_label << endl;
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
							followingCurrentDirection = true;
							goToStart = true;
						}
					}
				}
			}
			cout << "followingCurrentDirection " << followingCurrentDirection << " goToStart " << goToStart << endl;
			if(followingCurrentDirection == false and foundAlignmentPoint == true){
				currentDPoint = DPoint(alignmentPoint, alignmentPoint);
				cout << "DPoint selected " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
				followingCurrentDirection = true;
				goToStart = true;
			}
			if(followingCurrentDirection == true){
				//plan route to current point and then follow it to end or until the other direction is detected (need to switch directions if going in desired directions)
				if(goToStart == true){
					if(currentPosition.getDistance(currentDPoint.point) <= 0.75){
						goToStart = false;
						cout << "Close to start of new circumnavigate point" << endl;
						cout << "DPoint " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
						CartesianPoint task(currentDPoint.middle_point.getX(),currentDPoint.middle_point.getY());
						(*decision) = beliefs->getAgentState()->moveTowards(task);
						FORRAction forward = beliefs->getAgentState()->maxForwardAction();
						Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
						if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
							if(vetoed_actions->find(*decision) != vetoed_actions->end()){
								decisionMade = false;
							}
							else{
								cout << "Circumnavigate advisor to take decision" << endl;
								decisionMade = true;
							}
						}
					}
					else if(beliefs->getAgentState()->getCurrentTask()->getIsPlanActive() == false){
						cout << "Not close to start of circumnavigate point" << endl;
						Task* currentTask = beliefs->getAgentState()->getCurrentTask();
						vector<Position> *pos_hist = currentTask->getPositionHistory();
						vector< vector<CartesianPoint> > *laser_hist = currentTask->getLaserHistory();
						vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
						cout << "regions " << regions.size() << endl;
						vector<CartesianPoint> trace;
						for(int i = 0 ; i < pos_hist->size() ; i++){
							trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
						}
						vector< vector<CartesianPoint> > laser_trace;
						for(int i = 0 ; i < laser_hist->size() ; i++){
							laser_trace.push_back((*laser_hist)[i]);
						}
						cout << "trace " << trace.size() << " laser_trace " << laser_trace.size() << endl;
						int cutoff = 0;
						for(int i = trace.size()-1; i > 0; i--){
							// double radius = 10000;
							// for(int j = 0; j< laser_trace[i].size(); j++){
							// 	double range = laser_trace[i][j].get_distance(trace[i]);
							// 	if (range < radius and range >= 0.1){
							// 		radius = range;
							// 	}
							// }
							// FORRRegion current_region = FORRRegion(trace[i], laser_trace[i], radius);
							for(int j = 0; j < regions.size(); j++){
								// if(regions[j].inRegion(trace[i]) or regions[j].doIntersect(current_region)){
								if(regions[j].inRegion(trace[i])){
									cutoff = i;
									break;
								}
							}
							if(cutoff > 0){
								break;
							}
						}
						cout << "cutoff " << cutoff << endl;
						vector< Position > *position_trace = new vector<Position>();
						vector< vector<CartesianPoint> > *laser_hist_trace = new vector< vector<CartesianPoint> >();
						vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
						vector< vector<CartesianPoint> > exit_traces = beliefs->getAgentState()->getInitialExitTraces();
						all_trace.push_back(trace);
						for(int i = 0; i < exit_traces.size(); i++){
							all_trace.insert(all_trace.begin(), exit_traces[i]);
						}
						vector<CartesianPoint> new_trace;
						for(int i = cutoff ; i < pos_hist->size() ; i++){
							new_trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
							position_trace->push_back((*pos_hist)[i]);
							laser_hist_trace->push_back((*laser_hist)[i]);
						}
						cout << "new_trace " << new_trace.size() << " position_trace " << position_trace->size() << " laser_hist_trace " << laser_hist_trace->size() << endl;
						beliefs->getSpatialModel()->getRegionList()->learnRegionsAndExits(position_trace, laser_hist_trace, all_trace);
						// beliefs->getSpatialModel()->getRegionList()->learnRegions(position_trace, laser_hist_trace);
						cout << "Regions Updated" << endl;
						// beliefs->getSpatialModel()->getRegionList()->clearAllExits();
						// beliefs->getSpatialModel()->getRegionList()->learnExits(all_trace);
						cout << "Exits Updated" << endl;
						regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
						vector<FORRRegion> new_regions;
						for(int i = 0; i < new_trace.size(); i++){
							for(int j = 0; j < regions.size(); j++){
								if(regions[j].inRegion(new_trace[i])){
									new_regions.push_back(regions[j]);
								}
							}
						}
						cout << "new_regions " << new_regions.size() << endl;
						beliefs->getSpatialModel()->getDoors()->clearAllDoors();
						beliefs->getSpatialModel()->getDoors()->learnDoors(beliefs->getSpatialModel()->getRegionList()->getRegions());
						cout << "Doors Learned" << endl;
						int index_val = skeleton_planner->getGraph()->getMaxInd()+1;
						cout << "start index " << index_val << endl;
						for(int i = 0 ; i < new_regions.size(); i++){
							int x = (int)(new_regions[i].getCenter().get_x()*100);
							int y = (int)(new_regions[i].getCenter().get_y()*100);
							cout << "Region " << new_regions[i].getCenter().get_x() << " " << new_regions[i].getCenter().get_y() << " " << x << " " << y << endl;
							vector<FORRExit> exits = new_regions[i].getMinExits();
							cout << "Exits " << exits.size() << endl;
							if(exits.size() > 0){
								bool success = skeleton_planner->getGraph()->addNode(x, y, index_val);
								if(success){
									index_val++;
								}
								for(int j = 0; j < exits.size() ; j++){
									int ex = (int)(exits[j].getExitPoint().get_x()*100);
									int ey = (int)(exits[j].getExitPoint().get_y()*100);
									cout << "Exit " << exits[j].getExitPoint().get_x() << " " << exits[j].getExitPoint().get_y() << " " << ex << " " << ey << endl;
									success = skeleton_planner->getGraph()->addNode(ex, ey, index_val);
									if(success){
										index_val++;
									}
									int mx = (int)(exits[j].getMidPoint().get_x()*100);
									int my = (int)(exits[j].getMidPoint().get_y()*100);
									cout << "Midpoint " << exits[j].getMidPoint().get_x() << " " << exits[j].getMidPoint().get_y() << " " << mx << " " << my << endl;
									success = skeleton_planner->getGraph()->addNode(mx, my, index_val);
									if(success){
										index_val++;
									}
								}
							}
						}
						for(int i = 0 ; i < new_regions.size(); i++){
							int region_id = skeleton_planner->getGraph()->getNodeID((int)(new_regions[i].getCenter().get_x()*100), (int)(new_regions[i].getCenter().get_y()*100));
							if(region_id != -1){
								vector<FORRExit> exits = new_regions[i].getMinExits();
								for(int j = 0; j < exits.size() ; j++){
									int index_val = skeleton_planner->getGraph()->getNodeID((int)(exits[j].getExitPoint().get_x()*100), (int)(exits[j].getExitPoint().get_y()*100));
									if(index_val != -1){
										cout << "Edge from " << region_id << " to " << index_val << " Distance " << new_regions[i].getRadius()*100 << endl;
										skeleton_planner->getGraph()->addEdge(region_id, index_val, new_regions[i].getRadius()*100);
										int mid_index_val = skeleton_planner->getGraph()->getNodeID((int)(exits[j].getMidPoint().get_x()*100), (int)(exits[j].getMidPoint().get_y()*100));
										if(mid_index_val != -1){
											cout << "Edge from " << index_val << " to " << mid_index_val << " Distance " << (exits[j].getExitDistance()*100)/2.0 << endl;
											skeleton_planner->getGraph()->addEdge(index_val, mid_index_val, (exits[j].getExitDistance()*100)/2.0);
											int tx = (int)(exits[j].getExitRegionPoint().get_x()*100);
											int ty = (int)(exits[j].getExitRegionPoint().get_y()*100);
											int end_index_val = skeleton_planner->getGraph()->getNodeID(tx, ty);
											if(end_index_val != -1){
												cout << "Edge from " << mid_index_val << " to " << end_index_val << " Distance " << (exits[j].getExitDistance()*100)/2.0 << endl;
												skeleton_planner->getGraph()->addEdge(mid_index_val, end_index_val, (exits[j].getExitDistance()*100)/2.0);
											}
										}
									}
								}
							}
						}

						Node s(1, currentPosition.getX()*100, currentPosition.getY()*100);
						skeleton_planner->setSource(s);
						Node t(1, currentDPoint.middle_point.getX()*100, currentDPoint.middle_point.getY()*100);
						skeleton_planner->setTarget(t);
						cout << "plan generation status" << skeleton_planner->calcPath(true) << endl;
						list<int> waypointInd = skeleton_planner->getPath();
						skeleton_planner->resetPath();
						cout << "plan generation complete" << endl;
						Graph *navGraph = skeleton_planner->getGraph();
						vector<CartesianPoint> waypoints;
						list<int>::iterator it;
						for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
							cout << "node " << (*it) << endl;
							double x = navGraph->getNode(*it).getX()/100.0;
							double y = navGraph->getNode(*it).getY()/100.0;
							cout << x << " " << y << endl;
							CartesianPoint waypoint(x,y);
							waypoints.push_back(waypoint);
						}
						for(int i = waypoints.size()-1; i > 0; i--){
							beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], true);
						}
					}
				}
				else{
					cout << "Move towards middle_point" << endl;
					CartesianPoint task(currentDPoint.middle_point.getX(),currentDPoint.middle_point.getY());
					cout << "DPoint " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
					(*decision) = beliefs->getAgentState()->moveTowards(task);
					FORRAction forward = beliefs->getAgentState()->maxForwardAction();
					Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
					if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
						if(vetoed_actions->find(*decision) != vetoed_actions->end()){
							decisionMade = false;
						}
						else{
							cout << "Circumnavigate advisor to take decision" << endl;
							decisionMade = true;
						}
					}
				}
			}
		}
		else if(followingCurrentDirection == true){
			cout << "Move towards middle_point" << endl;
			CartesianPoint task(currentDPoint.middle_point.getX(),currentDPoint.middle_point.getY());
			cout << "DPoint " << currentDPoint.point.getX() << " " << currentDPoint.point.getY() << " " << currentDPoint.middle_point.getX() << " " << currentDPoint.middle_point.getY() << endl;
			(*decision) = beliefs->getAgentState()->moveTowards(task);
			FORRAction forward = beliefs->getAgentState()->maxForwardAction();
			Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
			if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
				if(vetoed_actions->find(*decision) != vetoed_actions->end()){
					decisionMade = false;
				}
				else{
					cout << "Circumnavigate advisor to take decision" << endl;
					decisionMade = true;
				}
			}
		}
		cout << "decisionMade " << decisionMade << endl;
		return decisionMade;
	}

private:
	int length;
	int height;
	double move[300];  
	double rotate[300];
	int numMoves, numRotates;
	vector< vector<int> > visited_grid;
	Beliefs *beliefs;
	PathPlanner *skeleton_planner;
	// CHANGE TO PRIORITY QUEUES
	vector<DPoint> north_stack;
	vector<DPoint> south_stack;
	vector<DPoint> west_stack;
	vector<DPoint> east_stack;
	DPoint currentDPoint;
	Position alignmentPoint;
	bool followingCurrentDirection;
	bool goToStart;
	bool currentDirectionChanged;
	bool foundAlignmentPoint;
	int alignmentDirection;
};

#endif