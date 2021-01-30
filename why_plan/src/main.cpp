/* \mainpage SemaFORR Explanation
 * \brief Explains plans of a robot.
 *
 * \author Raj Korpan.
 *
 * \version SEMAFORR Explanation 1.0
 *
 *
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <map>
#include <algorithm>
#include <cmath>        //for atan2 and M_PI
#include <sys/time.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

using namespace std;

class Explanation
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "plan_explanations" topic
	ros::Publisher plan_explanations_pub_;
	//! We will be publishing to the "plan_explanations_log" topic
	ros::Publisher plan_explanations_log_pub_;
	//! We will be listening to \decision_log, \crowd_density, \plan, and \original_plan topics
	ros::Subscriber sub_decision_log_;
	ros::Subscriber sub_crowd_density_;
	ros::Subscriber sub_crowd_risk_;
	ros::Subscriber sub_plan_;
	ros::Subscriber sub_original_plan_;
	// Current log
	string current_log;
	// Current crowd density
	nav_msgs::OccupancyGrid current_crowd_density;
	// Current crowd risk
	nav_msgs::OccupancyGrid current_crowd_risk;
	// Current plans
	nav_msgs::Path current_plan;
	nav_msgs::Path current_original_plan;
	// Message received
	bool log_message_received;
	bool density_message_received;
	bool risk_message_received;
	bool plan_message_received;
	bool orig_plan_message_received;
	// Stats on plans
	double planDistance=0, originalPlanDistance=0, planCost=0, originalPlanCost=0, planCrowdDensity=0, originalPlanCrowdDensity=0, planCrowdRisk=0, originalPlanCrowdRisk=0;
	// thresholds and their associated phrases
	map <string, vector < pair<double, string> > > thresholds;
	// objectives and their associated phrases
	map <string, vector <string> > objectives;
	// plan directions phrases
	map <int, string> directions_phrases;
	// plan distances phrases
	map <double, string> distances_phrases;
	// // distance intervals with their associated phrases
	// vector <double> distanceThreshold;
	// vector <string> distancePhrase;
	// // density intervals with their associated phrases
	// vector <double> densityThreshold;
	// vector <string> densityPhrase;
	// // risk intervals with their associated phrases
	// vector <double> riskThreshold;
	// vector <string> riskPhrase;
	// // flow intervals with their associated phrases
	// vector <double> flowThreshold;
	// vector <string> flowPhrase;
	// // convey intervals with their associated phrases
	// vector <double> conveyThreshold;
	// vector <string> conveyPhrase;
	// // hallway intervals with their associated phrases
	// vector <double> hallwayThreshold;
	// vector <string> hallwayPhrase;
	// // region intervals with their associated phrases
	// vector <double> regionThreshold;
	// vector <string> regionPhrase;
	// // trail intervals with their associated phrases
	// vector <double> trailThreshold;
	// vector <string> trailPhrase;
	// // skeleton intervals with their associated phrases
	// vector <double> skeletonThreshold;
	// vector <string> skeletonPhrase;
	// // highway intervals with their associated phrases
	// vector <double> highwayThreshold;
	// vector <string> highwayPhrase;
	// other parameters
	double targetX, targetY, robotY, robotX;
	string selected_planner;
	vector<string> alternative_planners;
	string alt_planner;
	bool sameplan;
	double computationTimeSec=0.0;
	int densityNum = 5;
	int riskNum = 5;
	int costNum = 5;
	int distanceNum = 5;

public:
	//! ROS node initialization
	Explanation(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the explanations topic
		plan_explanations_pub_ = nh_.advertise<std_msgs::String>("plan_explanations", 1);
		plan_explanations_log_pub_ = nh_.advertise<std_msgs::String>("plan_explanations_log", 1);
		sub_decision_log_ = nh.subscribe("decision_log", 1000, &Explanation::updateLog, this);
		sub_crowd_density_ = nh.subscribe("crowd_density", 1000, &Explanation::updateCrowdDensity, this);
		sub_crowd_risk_ = nh.subscribe("crowd_risk", 1000, &Explanation::updateCrowdRisk, this);
		sub_plan_ = nh.subscribe("plan", 1000, &Explanation::updatePlan, this);
		sub_original_plan_ = nh.subscribe("original_plan", 1000, &Explanation::updateOriginalPlan, this);
		log_message_received = false;
		density_message_received = false;
		risk_message_received = false;
		plan_message_received = false;
		orig_plan_message_received = false;
	}

	void updateLog(const std_msgs::String & log){
		log_message_received = true;
		current_log = log.data;
		//ROS_INFO_STREAM("Recieved log data: " << current_log << endl);
	}

	void updateCrowdDensity(const nav_msgs::OccupancyGrid & crowd_density){
		density_message_received = true;
		current_crowd_density = crowd_density;
		//ROS_INFO_STREAM("Recieved crowd density data: " << current_crowd_density << endl);
	}

	void updateCrowdRisk(const nav_msgs::OccupancyGrid & crowd_risk){
		risk_message_received = true;
		current_crowd_risk = crowd_risk;
		//ROS_INFO_STREAM("Recieved crowd density data: " << current_crowd_risk << endl);
	}

	void updatePlan(const nav_msgs::Path & plan){
		if (plan.poses.size() > 0) {
			plan_message_received = true;
			current_plan = plan;
			//ROS_INFO_STREAM("Recieved plan data: " << current_plan << endl);
		}
	}

	void updateOriginalPlan(const nav_msgs::Path & original_plan){
		if (original_plan.poses.size() > 0) {
			orig_plan_message_received = true;
			current_original_plan = original_plan;
			//ROS_INFO_STREAM("Recieved orig plan data: " << current_original_plan << endl);
		}
	}

	void initialize(string text_config){
		string fileLine;
		ifstream file(text_config.c_str());
		ROS_DEBUG_STREAM("Reading text_config_file:" << text_config);
		if(!file.is_open()){
			ROS_DEBUG("Unable to locate or read text config file!");
		}

		while(getline(file, fileLine)){
			//cout << "Inside while in tasks" << endl;
			if(fileLine[0] == '#'){  // skip comment lines
				continue;
			}
			else if (fileLine.find("threshold") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
				for(int i=1; i < vstrings.size(); i+=7){
					vector < pair<double, string> > thresh;
					thresh.push_back(pair<double, string>(atof(vstrings[i+1].c_str()),vstrings[i+2]));
					thresh.push_back(pair<double, string>(atof(vstrings[i+3].c_str()),vstrings[i+4]));
					thresh.push_back(pair<double, string>(atof(vstrings[i+5].c_str()),vstrings[i+6]));
					thresholds.insert(pair< string, vector < pair<double, string> > >(vstrings[i], thresh));
					//ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << " " << vstrings[i+2] << " " << vstrings[i+3] << " " << vstrings[i+4] << " " << vstrings[i+5] << " " << vstrings[i+6] << endl);
				}
			}
			else if (fileLine.find("objphrases") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
				for(int i=1; i < vstrings.size(); i+=4){
					vector <string> > objs;
					objs.push_back(vstrings[i+1]);
					objs.push_back(vstrings[i+2]);
					objs.push_back(vstrings[i+3]);
					objectives.insert(pair< string, vector <string> >(vstrings[i], objs));
					//ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << " " << vstrings[i+2] << " " << vstrings[i+3] << endl);
				}
			}
			else if (fileLine.find("plandirections") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
				for(int i=1; i < vstrings.size(); i+=2){
					directions_phrases.insert(pair< int, string >(atoi(vstrings[i].c_str()), vstrings[i+1]));
					//ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			else if (fileLine.find("plandistances") != string::npos){
				vector<string> vstrings = parseText(fileLine, '\t');
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
				for(int i=1; i < vstrings.size(); i+=2){
					distances_phrases.insert(pair< double, string >(atof(vstrings[i].c_str()), vstrings[i+1]));
					//ROS_DEBUG_STREAM("File text:" << vstrings[i] << " " << vstrings[i+1] << endl);
				}
			}
			// else if (fileLine.find("distance") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		distanceThreshold.push_back(atof(vstrings[i].c_str()));
			// 		distancePhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("density") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		densityThreshold.push_back(atof(vstrings[i].c_str()));
			// 		densityPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("risk") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		riskThreshold.push_back(atof(vstrings[i].c_str()));
			// 		riskPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("flow") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		flowThreshold.push_back(atof(vstrings[i].c_str()));
			// 		flowPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("conveys") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		conveyThreshold.push_back(atof(vstrings[i].c_str()));
			// 		conveyPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("hallwayer") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		hallwayThreshold.push_back(atof(vstrings[i].c_str()));
			// 		hallwayPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("spatial") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		regionThreshold.push_back(atof(vstrings[i].c_str()));
			// 		regionPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("trailer") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		trailThreshold.push_back(atof(vstrings[i].c_str()));
			// 		trailPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("skeleton") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		skeletonThreshold.push_back(atof(vstrings[i].c_str()));
			// 		skeletonPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
			// else if (fileLine.find("hallwayskel") != string::npos){
			// 	vector<string> vstrings = parseText(fileLine, '\t');
			// 	for(int i=1; i < vstrings.size(); i+=2){
			// 		highwayThreshold.push_back(atof(vstrings[i].c_str()));
			// 		highwayPhrase.push_back(vstrings[i+1]);
			// 		//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
			// 	}
			// 	//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			// }
		}
		ros::spinOnce();
	}
	
	void run(){
		std_msgs::String explanationString;
		ros::Rate rate(30.0);
		timeval cv;
		double start_timecv, end_timecv;
		while(nh_.ok()) {
			//while(log_message_received == false or plan_message_received == false or orig_plan_message_received == false or (density_message_received == false and risk_message_received == false)){
			while(log_message_received == false or plan_message_received == false or orig_plan_message_received == false){
				//ROS_DEBUG("Waiting for all messages");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			ROS_INFO_STREAM("Messages received");
			gettimeofday(&cv,NULL);
			start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			vector<string> parsed_log = parseText(current_log, '\t');
			targetX = atof(parsed_log[4].c_str());
			targetY = atof(parsed_log[5].c_str());
			robotX = atof(parsed_log[6].c_str());
			robotY = atof(parsed_log[7].c_str());
			selected_planner = parseText(parsed_log[25], '>')[0];
			alternative_planners = parseText(parsed_log[25], '>');
			alternative_planners.erase(alternative_planners.begin());
			savePlanCosts();
			alt_planner = 'distance';
			if(selected_planner == 'skeleton'){
				if(alternative_planners[0] == 'hallwayskel'){
					selected_planner = 'skeletonhall';
					alt_planner = 'hallwayskel';
				}
			}
			else if(selected_planner == 'hallwayskel'){
				alt_planner = 'skeletonhall';
			}
			//ROS_INFO_STREAM("Before compute plan distances");
			//computePlanDistances();
			//ROS_INFO_STREAM("Before compute plan densities");
			//computePlanDensities();
			//computePlanRisks();
			//ROS_INFO_STREAM("Before compare plans");
			if(selected_planner == 'distance'){
				explanationString.data = "I decided to go this way because I agree that we should take the shortest route.\nI think your way is better.\nYour way is the best way to go.\nI'm really sure because this is the shortest way.";
			}
			else if (comparePlans()) {
				explanationString.data = "I decided to go this way because I think it is just as " + objectivePhrase(alt_planner, 0) + " and equally " + objectivePhrase(selected_planner, 0) + ".\nI think both plans are equally good.\nWe could go that way since it's a bit " + objectivePhrase(alt_planner, 1) + " but it could also be a bit " + objectivePhrase(selected_planner, 2) + ".\nI'm only somewhat sure because even though my plan is a bit " + objectivePhrase(selected_planner, 1) + ", it is also a bit " + objectivePhrase(alt_planner, 2) + " than your plan.";
			}
			else if ((planDistance - originalPlanDistance) <= 0) {
				explanationString.data = "I think my way is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "I think my way is better because it's " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "We could go that way since it's a bit " + objectivePhrase(alt_planner, 1) + " but it could also be " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 2) + ".\n" + "I'm " + computeCostConf() + ".";
			}
			else {
				explanationString.data = "Although there may be another way that is " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 1) + ", I think my way is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "I think my way is better because it's " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ".\n" + "We could go that way since it's " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 1) + " but it could also be " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 2) + ".\n" + "I'm " + computeCostConf() + ".";
			}
			if(selected_planner == 'hallwayskel' or selected_planner == 'skeletonhall'){
				// give explanation for them
				vector<double> current_plan_angles;
				vector<double> current_plan_distances;
				for(int i = 0; i < current_plan.poses.size()-1; i++){
					current_plan_angles.push_back(atan2(current_plan.poses[i].pose.position.y - current_plan.poses[i+1].pose.position.y, current_plan.poses[i].pose.position.x - current_plan.poses[i+1].pose.position.x));
					current_plan_distances.push_back(computeDistance(current_plan.poses[i].pose.position.x, current_plan.poses[i].pose.position.y, current_plan.poses[i+1].pose.position.x, current_plan.poses[i+1].pose.position.y));
				}
				vector<double> alt_plan_angles;
				vector<double> alt_plan_distances;
				for(int i = 0; i < current_original_plan.poses.size()-1; i++){
					alt_plan_angles.push_back(atan2(current_original_plan.poses[i].pose.position.y - current_original_plan.poses[i+1].pose.position.y, current_original_plan.poses[i].pose.position.x - current_original_plan.poses[i+1].pose.position.x));
					alt_plan_distances.push_back(computeDistance(current_original_plan.poses[i].pose.position.x, current_original_plan.poses[i].pose.position.y, current_original_plan.poses[i+1].pose.position.x, current_original_plan.poses[i+1].pose.position.y));
				}
				vector<int> current_plan_directions;
				for(int i = 0; i < current_plan_angles.size(); i++){
					if(current_plan_angles[i] >= -M_PI/8.0 and current_plan_angles[i] < M_PI/8.0){
						current_plan_directions.push_back(5);
					}
					else if(current_plan_angles[i] >= M_PI/8.0 and current_plan_angles[i] < 3.0*M_PI/8.0){
						current_plan_directions.push_back(6);
					}
					else if(current_plan_angles[i] >= 3.0*M_PI/8.0 and current_plan_angles[i] < 5.0*M_PI/8.0){
						current_plan_directions.push_back(7);
					}
					else if(current_plan_angles[i] >= 5.0*M_PI/8.0 and current_plan_angles[i] < 7.0*M_PI/8.0){
						current_plan_directions.push_back(8);
					}
					else if(current_plan_angles[i] >= 7.0*M_PI/8.0 or current_plan_angles[i] < -7.0*M_PI/8.0){
						current_plan_directions.push_back(1);
					}
					else if(current_plan_angles[i] >= -7.0*M_PI/8.0 and current_plan_angles[i] < -5.0*M_PI/8.0){
						current_plan_directions.push_back(2);
					}
					else if(current_plan_angles[i] >= -5.0*M_PI/8.0 and current_plan_angles[i] < -3.0*M_PI/8.0){
						current_plan_directions.push_back(3);
					}
					else if(current_plan_angles[i] >= -3.0*M_PI/8.0 and current_plan_angles[i] < -M_PI/8.0){
						current_plan_directions.push_back(4);
					}
				}
				vector<int> alt_plan_directions;
				for(int i = 0; i < alt_plan_angles.size(); i++){
					if(alt_plan_angles[i] >= -M_PI/8.0 and alt_plan_angles[i] < M_PI/8.0){
						alt_plan_directions.push_back(5);
					}
					else if(alt_plan_angles[i] >= M_PI/8.0 and alt_plan_angles[i] < 3.0*M_PI/8.0){
						alt_plan_directions.push_back(6);
					}
					else if(alt_plan_angles[i] >= 3.0*M_PI/8.0 and alt_plan_angles[i] < 5.0*M_PI/8.0){
						alt_plan_directions.push_back(7);
					}
					else if(alt_plan_angles[i] >= 5.0*M_PI/8.0 and alt_plan_angles[i] < 7.0*M_PI/8.0){
						alt_plan_directions.push_back(8);
					}
					else if(alt_plan_angles[i] >= 7.0*M_PI/8.0 or alt_plan_angles[i] < -7.0*M_PI/8.0){
						alt_plan_directions.push_back(1);
					}
					else if(alt_plan_angles[i] >= -7.0*M_PI/8.0 and alt_plan_angles[i] < -5.0*M_PI/8.0){
						alt_plan_directions.push_back(2);
					}
					else if(alt_plan_angles[i] >= -5.0*M_PI/8.0 and alt_plan_angles[i] < -3.0*M_PI/8.0){
						alt_plan_directions.push_back(3);
					}
					else if(alt_plan_angles[i] >= -3.0*M_PI/8.0 and alt_plan_angles[i] < -M_PI/8.0){
						alt_plan_directions.push_back(4);
					}
				}
				vector<int> current_plan_direction_changes;
				for(int i = 0; i < current_plan_directions.size()-1; i++){
					if(current_plan_directions[i+1] - current_plan_directions[i] < 0){
						current_plan_direction_changes.push_back(current_plan_directions[i+1] - current_plan_directions[i] + 8);
					}
					else{
						current_plan_direction_changes.push_back(current_plan_directions[i+1] - current_plan_directions[i]);
					}
				}
				vector<int> alt_plan_direction_changes;
				for(int i = 0; i < alt_plan_directions.size()-1; i++){
					if(alt_plan_directions[i+1] - alt_plan_directions[i] < 0){
						alt_plan_direction_changes.push_back(alt_plan_directions[i+1] - alt_plan_directions[i] + 8);
					}
					else{
						alt_plan_direction_changes.push_back(alt_plan_directions[i+1] - alt_plan_directions[i]);
					}
				}
			}
			gettimeofday(&cv,NULL);
			end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			computationTimeSec = (end_timecv-start_timecv);
			ROS_INFO_STREAM("After Plan explanation: " << explanationString.data);
			//ROS_INFO_STREAM("Before log data");
			logExplanationData();
			//send the explanation
			plan_explanations_pub_.publish(explanationString);
			log_message_received = false;
			density_message_received = false;
			risk_message_received = false;
			plan_message_received = false;
			orig_plan_message_received = false;
			//ROS_INFO_STREAM("Before clear stats");
			clearStats();
			//ROS_INFO_STREAM("Loop completed");
			//wait for some time
			rate.sleep();
			// Sense input 
			ros::spinOnce();
		}
	}

	void computePlanDistances(){
		//ROS_INFO_STREAM("Inside compute plan distances");
		//planDistance = computeDistance(robotX, robotY, current_plan.poses[0].pose.position.x, current_plan.poses[0].pose.position.y);
		//ROS_INFO_STREAM("Initial plan distance: " << planDistance);
		for(int i = 0; i < current_plan.poses.size()-1; i++){
			planDistance += computeDistance(current_plan.poses[i].pose.position.x, current_plan.poses[i].pose.position.y, current_plan.poses[i+1].pose.position.x, current_plan.poses[i+1].pose.position.y);
		}
		//ROS_INFO_STREAM("Plan distance after loop: " << planDistance);
		planDistance += computeDistance(current_plan.poses[current_plan.poses.size()-1].pose.position.x, current_plan.poses[current_plan.poses.size()-1].pose.position.y, targetX, targetY);
		//ROS_INFO_STREAM("Final plan distance: " << planDistance);

		//originalPlanDistance = computeDistance(robotX, robotY, current_original_plan.poses[0].pose.position.x, current_original_plan.poses[0].pose.position.y);
		//ROS_INFO_STREAM("Initial orig plan distance: " << originalPlanDistance);
		for(int i = 0; i < current_original_plan.poses.size()-1; i++){
			originalPlanDistance += computeDistance(current_original_plan.poses[i].pose.position.x, current_original_plan.poses[i].pose.position.y, current_original_plan.poses[i+1].pose.position.x, current_original_plan.poses[i+1].pose.position.y);
		}
		//ROS_INFO_STREAM("Orig Plan distance after loop: " << originalPlanDistance);
		originalPlanDistance += computeDistance(current_original_plan.poses[current_original_plan.poses.size()-1].pose.position.x, current_original_plan.poses[current_original_plan.poses.size()-1].pose.position.y, targetX, targetY);
		//ROS_INFO_STREAM("Final orig plan distance: " << originalPlanDistance);
	}

	void savePlanCosts(){
		//ROS_INFO_STREAM("Inside save plan distances");
		vector<string> vstrings1 = parseText(current_plan.header.frame_id, '\t');
		planCost = atof(vstrings1[0].c_str()); // cost is for selected plan
		planDistance = atof(vstrings1[1].c_str()); // distance is for alternative plan
		ROS_INFO_STREAM("Plan distance: " << planDistance << " Plan cost = " << planCost);
		vector<string> vstrings2 = parseText(current_original_plan.header.frame_id, '\t');
		originalPlanCost = atof(vstrings2[0].c_str());
		originalPlanDistance = atof(vstrings2[1].c_str());
		ROS_INFO_STREAM("Orig Plan distance: " << originalPlanDistance << " Orig Plan cost = " << originalPlanCost);
	}

	double computeDistance(double x1, double y1, double x2, double y2){
		//ROS_INFO_STREAM("Inside compute distance");
		double distance = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
		return distance;
	}

	void computePlanDensities(){
		ROS_INFO_STREAM("Inside compute plan densities");
		//planCrowdDensity = atof(parseText(current_log)[25].c_str(), '\t');
		//ROS_INFO_STREAM("Plan density: " << planCrowdDensity);
		//originalPlanCrowdDensity = atof(parseText(current_log)[27].c_str(), '\t');
		//ROS_INFO_STREAM("Orig plan density: " << originalPlanCrowdDensity);

		vector< vector<int> > densities;
		int granularity = current_crowd_density.info.resolution;
		int boxes_height = current_crowd_density.info.height;
		int boxes_width = current_crowd_density.info.width;
		int map_height = boxes_height * granularity;
		int map_width = boxes_width * granularity;
		ROS_INFO_STREAM("granularity: " << granularity << " boxes_height: " << boxes_height << " boxes_width: " << boxes_width << " map_height: " << map_height << " map_width: " << map_width);
		int start = 0;
		//vector<signed char> density_data = current_crowd_density.data;
		for(int i = 0; i < boxes_width; i++){
			//ROS_INFO_STREAM("start: " << start);
			vector<int> col;
			for(int j = start; j < current_crowd_density.data.size(); j+=boxes_width){
				//ROS_INFO_STREAM("j: " << j << " density: " << (int)(unsigned char)current_crowd_density.data[j]);
				col.push_back((int)(unsigned char)current_crowd_density.data[j]);
			}
			densities.push_back(col);
			start++;
		}

		ROS_INFO_STREAM("Initial plan density: " << planCrowdDensity);
		for(int i = 0; i < current_plan.poses.size()-1; i++){
			planCrowdDensity += getGridValue(current_plan.poses[i].pose.position.x, current_plan.poses[i].pose.position.y, map_width, map_height, boxes_width, boxes_height, densities);
		}
		ROS_INFO_STREAM("Plan density after loop: " << planCrowdDensity);

		ROS_INFO_STREAM("Initial orig plan density: " << originalPlanCrowdDensity);
		for(int i = 0; i < current_original_plan.poses.size()-1; i++){
			originalPlanCrowdDensity += getGridValue(current_original_plan.poses[i].pose.position.x, current_original_plan.poses[i].pose.position.y, map_width, map_height, boxes_width, boxes_height, densities);
		}
		ROS_INFO_STREAM("Plan orig density after loop: " << originalPlanCrowdDensity);
	}

	void computePlanRisks(){
		ROS_INFO_STREAM("Inside compute plan risks");
		vector< vector<int> > risks;
		int granularity = current_crowd_risk.info.resolution;
		int boxes_height = current_crowd_risk.info.height;
		int boxes_width = current_crowd_risk.info.width;
		int map_height = boxes_height * granularity;
		int map_width = boxes_width * granularity;
		ROS_INFO_STREAM("granularity: " << granularity << " boxes_height: " << boxes_height << " boxes_width: " << boxes_width << " map_height: " << map_height << " map_width: " << map_width);
		int start = 0;
		//vector<signed char> risk_data = current_crowd_risk.data;
		for(int i = 0; i < boxes_width; i++){
			//ROS_INFO_STREAM("start: " << start);
			vector<int> col;
			for(int j = start; j < current_crowd_risk.data.size(); j+=boxes_width){
				//ROS_INFO_STREAM("j: " << j << " risk: " << (int)(unsigned char)current_crowd_risk.data[j]);
				col.push_back((int)(unsigned char)current_crowd_risk.data[j]);
			}
			risks.push_back(col);
			start++;
		}

		ROS_INFO_STREAM("Initial plan risk: " << planCrowdRisk);
		for(int i = 0; i < current_plan.poses.size()-1; i++){
			planCrowdRisk += getGridValue(current_plan.poses[i].pose.position.x, current_plan.poses[i].pose.position.y, map_width, map_height, boxes_width, boxes_height, risks);
		}
		ROS_INFO_STREAM("Plan risk after loop: " << planCrowdRisk);

		ROS_INFO_STREAM("Initial orig plan risk: " << originalPlanCrowdRisk);
		for(int i = 0; i < current_original_plan.poses.size()-1; i++){
			originalPlanCrowdRisk += getGridValue(current_original_plan.poses[i].pose.position.x, current_original_plan.poses[i].pose.position.y, map_width, map_height, boxes_width, boxes_height, risks);
		}
		ROS_INFO_STREAM("Plan orig risk after loop: " << originalPlanCrowdRisk);
	}

	int getGridValue(double map_x, double map_y, int map_width, int map_height, int boxes_width, int boxes_height, vector< vector<int> > densities){
		if(map_x < 0) map_x=0;
		if(map_x > map_width) map_x=map_width;
		if(map_y < 0) map_y=0;
		if(map_y > map_height) map_y=map_height;
		pair<int,int> grid_coords = make_pair((int)((map_x/(map_width*1.0)) * boxes_width), (int)((map_y/(map_height * 1.0)) * boxes_height));
		return densities[grid_coords.first][grid_coords.second];
	}

	bool comparePlans(){
		sameplan = 1;
		//ROS_INFO_STREAM("Comparing plans...");
		if (planDistance != originalPlanDistance){
			sameplan = 0;
			ROS_INFO_STREAM("Plan distances are different");
			return sameplan;
		}
		else if (planCrowdDensity != originalPlanCrowdDensity or planCrowdRisk != originalPlanCrowdRisk or planCost != originalPlanCost) {
			sameplan = 0;
			ROS_INFO_STREAM("Plan densities or risks or costs are different");
			return sameplan;
		}
		/*else if (current_plan.poses.size() != current_original_plan.poses.size()) {
			sameplan = 0;
			ROS_INFO_STREAM("Plan number of poses are different");
			return sameplan;
		}
		else {
			for(int i = 0; i < current_plan.poses.size()-1; i++){
				if (current_plan.poses[i].pose.position.x != current_original_plan.poses[i].pose.position.x or current_plan.poses[i].pose.position.y != current_original_plan.poses[i].pose.position.y) {
					sameplan = 0;
					ROS_INFO_STREAM("One of the plan poses is different");
					return sameplan;
				}
			}
		}*/
		ROS_INFO_STREAM("Plans are the same");
		return sameplan;
	}

	// string densityDifftoPhrase(){
	// 	//ROS_INFO_STREAM("Inside density diff to phrase");
	// 	string phrase;
	// 	for (int i = densityThreshold.size()-1; i >= 0; --i) {
	// 		if ((planCrowdDensity - originalPlanCrowdDensity) <= densityThreshold[i]) {
	// 			phrase = densityPhrase[i];
	// 			densityNum = i;
	// 		}
	// 	}
	// 	ROS_INFO_STREAM((planCrowdDensity - originalPlanCrowdDensity) << " " << phrase << " " << densityNum);
	// 	return phrase;
	// }

	// string riskDifftoPhrase(){
	// 	//ROS_INFO_STREAM("Inside risk diff to phrase");
	// 	string phrase;
	// 	for (int i = densityThreshold.size()-1; i >= 0; --i) {
	// 		if ((planCrowdRisk - originalPlanCrowdRisk) <= densityThreshold[i]) {
	// 			phrase = densityPhrase[i];
	// 			riskNum = i;
	// 		}
	// 	}
	// 	ROS_INFO_STREAM((planCrowdRisk - originalPlanCrowdRisk) << " " << phrase << " " << riskNum);
	// 	return phrase;
	// }

	string costDifftoPhrase(string plannerName){
		//ROS_INFO_STREAM("Inside cost diff to phrase");
		vector < pair<double, string> > plannerThreshold = thresholds[plannerName];
		string phrase;
		for (int i = plannerThreshold.size()-1; i >= 0; --i) {
			if ((planCost - originalPlanCost) <= plannerThreshold[i].first) {
				phrase = plannerThreshold[i].second;
				costNum = i;
			}
		}
		//ROS_INFO_STREAM((planCost - originalPlanCost) << " " << phrase << " " << costNum);
		return phrase;
	}

	string distanceDiffToPhrase(string plannerName){
		//ROS_INFO_STREAM("Inside distance diff to phrase");
		vector < pair<double, string> > plannerThreshold = thresholds[plannerName];
		string phrase;
		for (int i = plannerThreshold.size()-1; i >= 0; --i) {
			if ((planDistance - originalPlanDistance) <= plannerThreshold[i]) {
				phrase = plannerThreshold[i];
				distanceNum = i;
			}
		}
		//ROS_INFO_STREAM((planDistance - originalPlanDistance) << " " << phrase << " " << distanceNum);
		return phrase;
	}

	string objectivePhrase(string plannerName, int type){
		//ROS_INFO_STREAM("Inside objectivePhrase");
		vector <string> plannerObjective = objectives[plannerName];
		string phrase = plannerObjective[type];
		//ROS_INFO_STREAM(type << " " << phrase);
		return phrase;
	}

	// string computeConf(){
	// 	ROS_INFO_STREAM("Inside compute conf");
	// 	string phrase, tempphrase;
	// 	if (densityNum == 5) tempphrase = densityDifftoPhrase();
	// 	if (distanceNum == 5) tempphrase = distanceDiffToPhrase();
	// 	ROS_INFO_STREAM((planCrowdDensity - originalPlanCrowdDensity) << " " << densityNum << " " << (planDistance - originalPlanDistance) << " " << distanceNum);
	// 	if ((densityNum == 0 and distanceNum == 2) or (densityNum == 1 and distanceNum == 1) or (densityNum == 2 and distanceNum == 0)){
	// 		phrase = "only somewhat sure because even though my plan is " + densityDifftoPhrase() + " less crowded, it is also " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	else if((densityNum == 1 and distanceNum == 2) or (densityNum == 2 and distanceNum == 2) or (densityNum == 2 and distanceNum == 1)){
	// 		phrase = "not sure because my plan is " + distanceDiffToPhrase() + " longer than your plan and only " + densityDifftoPhrase() + " less crowded";
	// 	}
	// 	else if((densityNum == 0 and distanceNum == 1) or (densityNum == 0 and distanceNum == 0) or (densityNum == 1 and distanceNum == 0)){
	// 		phrase = "really sure because my plan is " + densityDifftoPhrase() + " less crowded and only " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	return phrase;
	// }

	// string computeRiskConf(){
	// 	ROS_INFO_STREAM("Inside compute risk conf");
	// 	string phrase, tempphrase;
	// 	if (riskNum == 5) tempphrase = riskDifftoPhrase();
	// 	if (distanceNum == 5) tempphrase = distanceDiffToPhrase();
	// 	ROS_INFO_STREAM((planCrowdRisk - originalPlanCrowdRisk) << " " << riskNum << " " << (planDistance - originalPlanDistance) << " " << distanceNum);
	// 	if ((riskNum == 0 and distanceNum == 2) or (riskNum == 1 and distanceNum == 1) or (riskNum == 2 and distanceNum == 0)){
	// 		phrase = "only somewhat sure because even though my plan is " + riskDifftoPhrase() + " less risky, it is also " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	else if((riskNum == 1 and distanceNum == 2) or (riskNum == 2 and distanceNum == 2) or (riskNum == 2 and distanceNum == 1)){
	// 		phrase = "not sure because my plan is " + distanceDiffToPhrase() + " longer than your plan and only " + riskDifftoPhrase() + " less risky";
	// 	}
	// 	else if((riskNum == 0 and distanceNum == 1) or (riskNum == 0 and distanceNum == 0) or (riskNum == 1 and distanceNum == 0)){
	// 		phrase = "really sure because my plan is " + riskDifftoPhrase() + " less risky and only " + distanceDiffToPhrase() + " longer than your plan";
	// 	}
	// 	return phrase;
	// }

	string computeCostConf(){
		//ROS_INFO_STREAM("Inside compute cost conf");
		string phrase, tempphrase;
		if (costNum == 5) tempphrase = costDifftoPhrase(selected_planner);
		if (distanceNum == 5) tempphrase = distanceDiffToPhrase(alt_planner);
		//ROS_INFO_STREAM((planCost - originalPlanCost) << " " << costNum << " " << (planDistance - originalPlanDistance) << " " << distanceNum);
		if ((costNum == 0 and distanceNum == 2) or (costNum == 1 and distanceNum == 1) or (costNum == 2 and distanceNum == 0)){
			phrase = "only somewhat sure because even though my plan is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + ", it is also " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 2) + " than your plan";
		}
		else if((costNum == 1 and distanceNum == 2) or (costNum == 2 and distanceNum == 2) or (costNum == 2 and distanceNum == 1)){
			phrase = "not sure because my plan is " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 1) + " than your plan and only " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 2) + " than your plan";
		}
		else if((costNum == 0 and distanceNum == 1) or (costNum == 0 and distanceNum == 0) or (costNum == 1 and distanceNum == 0)){
			phrase = "really sure because my plan is " + costDifftoPhrase(selected_planner) + " " + objectivePhrase(selected_planner, 1) + " and only " + distanceDiffToPhrase(alt_planner) + " " + objectivePhrase(alt_planner, 2) + " than your plan";
		}
		return phrase;
	}

	vector<string> parseText(string text, char delim){
		//ROS_INFO_STREAM("Inside parse text");
		vector<string> vstrings;
		stringstream ss;
		ss.str(text);
		string item;
		// char delim = '\t';
		while (getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		return vstrings;
	}
	
	void clearStats() {
		//ROS_INFO_STREAM("Inside clear stats");
		planDistance=0, originalPlanDistance=0, planCrowdDensity=0, originalPlanCrowdDensity=0, planCrowdRisk=0, originalPlanCrowdRisk=0, planCost=0, originalPlanCost=0;
		targetX=0, targetY=0, robotY=0, robotX=0;
		sameplan=1;
		computationTimeSec=0.0;
		densityNum=5, distanceNum=5, riskNum=5, costNum=5;
		string selected_planner;
		alternative_planners.clear();
		alt_planner = '';
	}
	
	void logExplanationData() {
		//ROS_INFO_STREAM("Inside log explanation data");
		std_msgs::String logData;
		vector<string> vstrings = parseText(current_log, '\t');

		stringstream output;
		output << atof(vstrings[0].c_str()) << "\t" << atof(vstrings[1].c_str()) << "\t" << atof(vstrings[2].c_str()) << "\t" << computationTimeSec << "\t" << sameplan << "\t" << alt_planner << "\t" << planDistance << "\t" << originalPlanDistance << "\t" << (planDistance - originalPlanDistance) << "\t" << selected_planner << "\t" << planCost << "\t" << originalPlanCost << "\t" << (planCost - originalPlanCost);
		
		logData.data = output.str();
		plan_explanations_log_pub_.publish(logData);
	}
};

// Main file : Load configuration file

int main(int argc, char **argv) {

	//init the ROS node
	ros::init(argc, argv, "why_plan");
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	string path = ros::package::getPath("why_plan");
	string text_config = path + "/config/text.conf";
	Explanation explain(nh);
	explain.initialize(text_config);
	ROS_INFO("Plan Explanation Initialized");
	explain.run();

	return 0;
}
