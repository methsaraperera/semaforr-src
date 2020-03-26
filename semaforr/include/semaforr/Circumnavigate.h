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
		goToStart = false;
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
						goToStart = true;
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
						goToStart = true;
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
						goToStart = true;
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
						goToStart = true;
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
							goToStart = true;
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
							goToStart = true;
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
							goToStart = true;
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
							goToStart = true;
						}
					}
				}
			}
			if(followingCurrentDirection == true){
				//plan route to current point and then follow it to end or until the other direction is detected (need to switch directions if going in desired directions)
				if(goToStart == true){
					if(currentPosition.getDistance(currentDPoint.point) < 0.5){
						goToStart = false;
						cout << "Close to start of new circumnavigate point" << endl;
						CartesianPoint task(currentDPoint.middle_point.getX(),currentDPoint.middle_point.getY());
						(*decision) = beliefs->getAgentState()->moveTowards(task);
						FORRAction forward = beliefs->getAgentState()->maxForwardAction();
						Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
						if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
							cout << "Circumnavigate advisor to take decision" << endl;
							decisionMade = true;
						}
					}
					else if(beliefs->getAgentState()->getCurrentTask()->getIsPlanActive() == false){
						Task* completedTask = beliefs->getAgentState()->getCurrentTask();
						vector<Position> *pos_hist = completedTask->getPositionHistory();
						vector< vector<CartesianPoint> > *laser_hist = completedTask->getLaserHistory();
						vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
						vector<CartesianPoint> trace;
						for(int i = 0 ; i < pos_hist->size() ; i++){
							trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
						}
						vector< vector<CartesianPoint> > laser_trace;
						for(int i = 0 ; i < laser_hist->size() ; i++){
							laser_trace.push_back((*laser_hist)[i]);
						}
						int cutoff = -1;
						for(int i = trace.size(); i > 0; i--){
							double radius = 10000;
							for(int j = 0; j< laser_trace[i].size(); j++){
								double range = laser_trace[i][j].get_distance(trace[i]);
								if (range < radius and range >= 0.1){
									radius = range;
								}
							}
							FORRRegion current_region = FORRRegion(trace[i], laser_trace[i], radius);
							for(int j = 0; j < regions.size(); j++){
								if(regions[j].inRegion(trace[i]) or regions[j].doIntersect(current_region)){
									cutoff = i;
									break;
								}
							}
							if(cutoff >= 0){
								break;
							}
						}
						vector< vector<CartesianPoint> > all_trace;
						vector<CartesianPoint> new_trace;
						for(int i = cutoff ; i < pos_hist->size() ; i++){
							trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
						}
						all_trace.push_back(trace);
						vector< Position > *position_trace = new vector<Position>();
						vector< vector<CartesianPoint> > *laser_hist_trace = new vector< vector<CartesianPoint> >();

						beliefs->getSpatialModel()->getRegionList()->learnRegions(pos_hist, laser_hist);
						cout << "Regions Updated" << endl;
						beliefs->getSpatialModel()->getRegionList()->learnExits(all_trace);
						cout << "Exits Updated" << endl;

					}
				}
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
	bool goToStart;
	bool currentDirectionChanged;
};

#endif