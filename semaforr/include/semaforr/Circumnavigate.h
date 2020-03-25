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
	Circumnavigate(int l, int h, Beliefs *b){
		length = l;
		height = h;
		beliefs = b;
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			visited_grid.push_back(col);
		}
		followingCurrentDirection = false;
	};
	~Circumnavigate(){};

	void resetCircumnavigate(){
		visited_grid.clear();
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			visited_grid.push_back(col);
		}
		followingCurrentDirection = false;
		currentDPoint = DPoint(Position(0,0,0),Position(0,0,0));
	}

	void addToStack(Position new_pose, sensor_msgs::LaserScan new_laser){
		double start_angle = new_laser.angle_min;
		double increment = new_laser.angle_increment;
		double r_x = new_pose.getX();
		double r_y = new_pose.getY();
		double r_ang = new_pose.getTheta();
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
			east_stack.insert(east_stack.begin(), DPoint(new_pose, middle_point));
			north_stack.insert(north_stack.begin(), DPoint(new_pose, left_point));
			south_stack.insert(south_stack.begin(), DPoint(new_pose, right_point));
		}
		else if(new_pose.getTheta() >= M_PI/2.0-M_PI/18.0 and new_pose.getTheta() < M_PI/2.0+M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to north " << middle_point.getX() << " " << middle_point.getY() << " added to west " << left_point.getX() << " " << left_point.getY() << " added to east " << right_point.getX() << " " << right_point.getY() << endl;
			north_stack.insert(north_stack.begin(), DPoint(new_pose, middle_point));
			west_stack.insert(west_stack.begin(), DPoint(new_pose, left_point));
			east_stack.insert(east_stack.begin(), DPoint(new_pose, right_point));
		}
		else if(new_pose.getTheta() >= M_PI-M_PI/18.0 or new_pose.getTheta() < -M_PI+M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to west " << middle_point.getX() << " " << middle_point.getY() << " added to south " << left_point.getX() << " " << left_point.getY() << " added to north " << right_point.getX() << " " << right_point.getY() << endl;
			west_stack.insert(west_stack.begin(), DPoint(new_pose, middle_point));
			north_stack.insert(north_stack.begin(), DPoint(new_pose, right_point));
			south_stack.insert(south_stack.begin(), DPoint(new_pose, left_point));
		}
		else if(new_pose.getTheta() >= -M_PI/2.0-M_PI/18.0 and new_pose.getTheta() < -M_PI/2.0+M_PI/18.0){
			cout << "Current Position " << r_x << " " << r_y << " " << r_ang << " added to south " << middle_point.getX() << " " << middle_point.getY() << " added to east " << left_point.getX() << " " << left_point.getY() << " added to west " << right_point.getX() << " " << right_point.getY() << endl;
			south_stack.insert(south_stack.begin(), DPoint(new_pose, middle_point));
			west_stack.insert(west_stack.begin(), DPoint(new_pose, right_point));
			east_stack.insert(east_stack.begin(), DPoint(new_pose, left_point));
		}
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY())] = 1;
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
		addToStack(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
		double x_diff = task.get_x() - currentPosition.getX();
		double y_diff = task.get_y() - currentPosition.getY();
		if(abs(x_diff) > abs(y_diff)){
			if(x_diff > 0){
				beliefs->getAgentState()->setCurrentDirection(1); // east
			}
			else{
				beliefs->getAgentState()->setCurrentDirection(2); // west
			}
			if(y_diff > 0){
				beliefs->getAgentState()->setDesiredDirection(3); // north
			}
			else{
				beliefs->getAgentState()->setDesiredDirection(4); // south
			}
		}
		else{
			if(x_diff > 0){
				beliefs->getAgentState()->setDesiredDirection(1);
			}
			else{
				beliefs->getAgentState()->setDesiredDirection(2);
			}
			if(y_diff > 0){
				beliefs->getAgentState()->setCurrentDirection(3);
			}
			else{
				beliefs->getAgentState()->setCurrentDirection(4);
			}
		}
		cout << "Current Direction " << beliefs->getAgentState()->getCurrentDirection() << " Desired Direction " << beliefs->getAgentState()->getDesiredDirection() << endl;
		if(beliefs->getAgentState()->getCurrentTask()->getIsPlanComplete() == true and followingCurrentDirection == false){
			if(beliefs->getAgentState()->getCurrentDirection() == 1){
				if(east_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0){
						new_current = east_stack[0];
						// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
						east_stack.erase(east_stack.begin());
						end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						followingCurrentDirection = true;
					}
				}
			}
			else if(beliefs->getAgentState()->getCurrentDirection() == 2){
				if(west_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0){
						new_current = west_stack[0];
						// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
						west_stack.erase(west_stack.begin());
						end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						followingCurrentDirection = true;
					}
				}
			}
			else if(beliefs->getAgentState()->getCurrentDirection() == 3){
				if(north_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0){
						new_current = north_stack[0];
						// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
						north_stack.erase(north_stack.begin());
						end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						followingCurrentDirection = true;
					}
				}
			}
			else if(beliefs->getAgentState()->getCurrentDirection() == 4){
				if(south_stack.size() > 0){
					DPoint new_current;
					int end_label = 0;
					while(end_label >= 0){
						new_current = south_stack[0];
						// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
						south_stack.erase(south_stack.begin());
						end_label = visited_grid[(int)(currentDPoint.middle_point.getX())][(int)(currentDPoint.middle_point.getY())];
					}
					if(!(new_current == currentDPoint)){
						currentDPoint = new_current;
						followingCurrentDirection = true;
					}
				}
			}
			if(followingCurrentDirection == false){
				if(beliefs->getAgentState()->getDesiredDirection() == 1){
					if(east_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0){
							new_current = east_stack[0];
							// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
							east_stack.erase(east_stack.begin());
							end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							followingCurrentDirection = true;
						}
					}
				}
				else if(beliefs->getAgentState()->getDesiredDirection() == 2){
					if(west_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0){
							new_current = west_stack[0];
							// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
							west_stack.erase(west_stack.begin());
							end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							followingCurrentDirection = true;
						}
					}
				}
				else if(beliefs->getAgentState()->getDesiredDirection() == 3){
					if(north_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0){
							new_current = north_stack[0];
							// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
							north_stack.erase(north_stack.begin());
							end_label = visited_grid[(int)(new_current.middle_point.getX())][(int)(new_current.middle_point.getY())];
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							followingCurrentDirection = true;
						}
					}
				}
				else if(beliefs->getAgentState()->getDesiredDirection() == 4){
					if(south_stack.size() > 0){
						DPoint new_current;
						int end_label = 0;
						while(end_label >= 0){
							new_current = south_stack[0];
							// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
							south_stack.erase(south_stack.begin());
							end_label = visited_grid[(int)(currentDPoint.middle_point.getX())][(int)(currentDPoint.middle_point.getY())];
						}
						if(!(new_current == currentDPoint)){
							currentDPoint = new_current;
							followingCurrentDirection = true;
						}
					}
				}
			}
			if(followingCurrentDirection == true){
				//plan route to current point and then follow it to end or until the other direction is detected (need to switch directions if going in desired directions)
			}
		}
		cout << "decisionMade " << decisionMade << endl;
		return decisionMade;
	}

private:
	int length;
	int height;
	vector< vector<int> > visited_grid;
	Beliefs *beliefs;
	vector<DPoint> north_stack;
	vector<DPoint> south_stack;
	vector<DPoint> west_stack;
	vector<DPoint> east_stack;
	DPoint currentDPoint;
	bool followingCurrentDirection;
	bool currentDirectionChanged;
};

#endif