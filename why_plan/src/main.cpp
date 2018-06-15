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
	double planLength=0, originalPlanLength=0, planCrowdDensity=0, originalPlanCrowdDensity=0, planCrowdRisk=0, originalPlanCrowdRisk=0;
	// length intervals with their associated phrases
	std::vector <double> lengthThreshold;
	std::vector <std::string> lengthPhrase;
	// density intervals with their associated phrases
	std::vector <double> densityThreshold;
	std::vector <std::string> densityPhrase;
	// other parameters
	double targetX, targetY, robotY, robotX;
	bool sameplan;
	double computationTimeSec=0.0;
	int densityNum = 5;
	int riskNum = 5;
	int lengthNum = 5;

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
		std::ifstream file(text_config.c_str());
		ROS_DEBUG_STREAM("Reading text_config_file:" << text_config);
		if(!file.is_open()){
			ROS_DEBUG("Unable to locate or read text config file!");
		}

		while(getline(file, fileLine)){
			//cout << "Inside while in tasks" << endl;
			if(fileLine[0] == '#')  // skip comment lines
				continue;
			else if (fileLine.find("length") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					lengthThreshold.push_back(atof(vstrings[i].c_str()));
					lengthPhrase.push_back(vstrings[i+1]);
					//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("density") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					densityThreshold.push_back(atof(vstrings[i].c_str()));
					densityPhrase.push_back(vstrings[i+1]);
					//ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
		}
		ros::spinOnce();
	}
	
	void run(){
		std_msgs::String explanationString;
		ros::Rate rate(30.0);
		timeval cv;
		double start_timecv, end_timecv;
		while(nh_.ok()) {
			while(log_message_received == false or plan_message_received == false or orig_plan_message_received == false or (density_message_received == false and risk_message_received == false)){
				//ROS_DEBUG("Waiting for all messages");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			ROS_INFO_STREAM("Messages received");
			gettimeofday(&cv,NULL);
			start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			targetX = atof(parseText(current_log)[4].c_str());
			targetY = atof(parseText(current_log)[5].c_str());
			robotX = atof(parseText(current_log)[6].c_str());
			robotY = atof(parseText(current_log)[7].c_str());
			//ROS_INFO_STREAM("Before compute plan lengths");
			computePlanLengths();
			//ROS_INFO_STREAM("Before compute plan densities");
			computePlanDensities();
			computePlanRisks();
			ROS_INFO_STREAM("Before compare plans");
			if (comparePlans()) {
				//explanationString.data = "I decided to go this way because I think it is just as short and not that crowded.\nI think both plans are equally good.\nWe could go that way since it's a bit shorter but it could also be a bit more crowded.\nI'm only somewhat sure because even though my plan is a bit less crowded, it is also a bit longer than your plan.";
				explanationString.data = "I decided to go this way because I think it is just as short and not that risky.\nI think both plans are equally good.\nWe could go that way since it's a bit shorter but it could also be a bit more risky.\nI'm only somewhat sure because even though my plan is a bit less risky, it is also a bit longer than your plan.";
			}
			else if ((planLength - originalPlanLength) <= 0) {
				//ROS_INFO_STREAM("Before density diff to phrase");
				//explanationString.data = "I think my way is " + densityDifftoPhrase() + " less crowded.\n" + "I think my way is better because it's " + densityDifftoPhrase() + " less crowded.\n" + "We could go that way since it's a bit shorter but it could also be " + densityDifftoPhrase() + " more crowded.\n" + "I'm " + computeConf() + ".";
				ROS_INFO_STREAM("Before risk diff to phrase");
				explanationString.data = "I think my way is " + riskDifftoPhrase() + " less risky.\n" + "I think my way is better because it's " + riskDifftoPhrase() + " less risky.\n" + "We could go that way since it's a bit shorter but it could also be " + riskDifftoPhrase() + " more risky.\n" + "I'm " + computeRiskConf() + ".";
			}
			else {
				ROS_INFO_STREAM("Before length diff to phrase");
				//explanationString.data = "Although there may be " + lengthDiffToPhrase() + " shorter way, I think my way is " + densityDifftoPhrase() + " less crowded.\n" + "I think my way is better because it's " + densityDifftoPhrase() + " less crowded.\n" + "We could go that way since it's " + lengthDiffToPhrase() + " shorter but it could also be " + densityDifftoPhrase() + " more crowded.\n" + "I'm " + computeConf() + ".";
				explanationString.data = "Although there may be " + lengthDiffToPhrase() + " shorter way, I think my way is " + riskDifftoPhrase() + " less risky.\n" + "I think my way is better because it's " + riskDifftoPhrase() + " less risky.\n" + "We could go that way since it's " + lengthDiffToPhrase() + " shorter but it could also be " + riskDifftoPhrase() + " more risky.\n" + "I'm " + computeRiskConf() + ".";
			}
			ROS_INFO_STREAM("After explanation: " << explanationString.data);
			gettimeofday(&cv,NULL);
			end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			computationTimeSec = (end_timecv-start_timecv);
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

	void computePlanLengths(){
		ROS_INFO_STREAM("Inside compute plan lengths");
		//planLength = computeDistance(robotX, robotY, current_plan.poses[0].pose.position.x, current_plan.poses[0].pose.position.y);
		ROS_INFO_STREAM("Initial plan length: " << planLength);
		for(int i = 0; i < current_plan.poses.size()-1; i++){
			planLength += computeDistance(current_plan.poses[i].pose.position.x, current_plan.poses[i].pose.position.y, current_plan.poses[i+1].pose.position.x, current_plan.poses[i+1].pose.position.y);
		}
		ROS_INFO_STREAM("Plan length after loop: " << planLength);
		planLength += computeDistance(current_plan.poses[current_plan.poses.size()-1].pose.position.x, current_plan.poses[current_plan.poses.size()-1].pose.position.y, targetX, targetY);
		ROS_INFO_STREAM("Final plan length: " << planLength);

		//originalPlanLength = computeDistance(robotX, robotY, current_original_plan.poses[0].pose.position.x, current_original_plan.poses[0].pose.position.y);
		ROS_INFO_STREAM("Initial orig plan length: " << originalPlanLength);
		for(int i = 0; i < current_original_plan.poses.size()-1; i++){
			originalPlanLength += computeDistance(current_original_plan.poses[i].pose.position.x, current_original_plan.poses[i].pose.position.y, current_original_plan.poses[i+1].pose.position.x, current_original_plan.poses[i+1].pose.position.y);
		}
		ROS_INFO_STREAM("Orig Plan length after loop: " << originalPlanLength);
		originalPlanLength += computeDistance(current_original_plan.poses[current_original_plan.poses.size()-1].pose.position.x, current_original_plan.poses[current_original_plan.poses.size()-1].pose.position.y, targetX, targetY);
		ROS_INFO_STREAM("Final orig plan length: " << originalPlanLength);
	}

	double computeDistance(double x1, double y1, double x2, double y2){
		//ROS_INFO_STREAM("Inside compute distance");
		double distance = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
		return distance;
	}

	void computePlanDensities(){
		ROS_INFO_STREAM("Inside compute plan densities");
		//planCrowdDensity = atof(parseText(current_log)[25].c_str());
		//ROS_INFO_STREAM("Plan density: " << planCrowdDensity);
		//originalPlanCrowdDensity = atof(parseText(current_log)[27].c_str());
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
		ROS_INFO_STREAM("Comparing plans...");
		if (planLength != originalPlanLength){
			sameplan = 0;
			ROS_INFO_STREAM("Plan lengths are different");
			return sameplan;
		}
		else if (planCrowdDensity != originalPlanCrowdDensity or planCrowdRisk != originalPlanCrowdRisk) {
			sameplan = 0;
			ROS_INFO_STREAM("Plan densities or risks are different");
			return sameplan;
		}
		else if (current_plan.poses.size() != current_original_plan.poses.size()) {
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
		}
		ROS_INFO_STREAM("Plans are the same");
		return sameplan;
	}

	std::string densityDifftoPhrase(){
		//ROS_INFO_STREAM("Inside density diff to phrase");
		std::string phrase;
		for (int i = densityThreshold.size()-1; i >= 0; --i) {
			if ((planCrowdDensity - originalPlanCrowdDensity) <= densityThreshold[i]) {
				phrase = densityPhrase[i];
				densityNum = i;
			}
		}
		ROS_INFO_STREAM((planCrowdDensity - originalPlanCrowdDensity) << " " << phrase << " " << densityNum);
		return phrase;
	}

	std::string riskDifftoPhrase(){
		//ROS_INFO_STREAM("Inside risk diff to phrase");
		std::string phrase;
		for (int i = densityThreshold.size()-1; i >= 0; --i) {
			if ((planCrowdRisk - originalPlanCrowdRisk) <= densityThreshold[i]) {
				phrase = densityPhrase[i];
				riskNum = i;
			}
		}
		ROS_INFO_STREAM((planCrowdDensity - originalPlanCrowdDensity) << " " << phrase << " " << riskNum);
		return phrase;
	}

	std::string lengthDiffToPhrase(){
		//ROS_INFO_STREAM("Inside length diff to phrase");
		std::string phrase;
		for (int i = lengthThreshold.size()-1; i >= 0; --i) {
			if ((planLength - originalPlanLength) <= lengthThreshold[i]) {
				phrase = lengthPhrase[i];
				lengthNum = i;
			}
		}
		ROS_INFO_STREAM((planLength - originalPlanLength) << " " << phrase << " " << lengthNum);
		return phrase;
	}

	std::string computeConf(){
		ROS_INFO_STREAM("Inside compute conf");
		std::string phrase, tempphrase;
		if (densityNum == 5) tempphrase = densityDifftoPhrase();
		if (lengthNum == 5) tempphrase = lengthDiffToPhrase();
		ROS_INFO_STREAM((planCrowdDensity - originalPlanCrowdDensity) << " " << densityNum << " " << (planLength - originalPlanLength) << " " << lengthNum);
		if ((densityNum == 0 and lengthNum == 2) or (densityNum == 1 and lengthNum == 1) or (densityNum == 2 and lengthNum == 0)){
			phrase = "only somewhat sure because even though my plan is " + densityDifftoPhrase() + " less crowded, it is also " + lengthDiffToPhrase() + " longer than your plan";
		}
		else if((densityNum == 1 and lengthNum == 2) or (densityNum == 2 and lengthNum == 2) or (densityNum == 2 and lengthNum == 1)){
			phrase = "not sure because my plan is " + lengthDiffToPhrase() + " longer than your plan and only " + densityDifftoPhrase() + " less crowded";
		}
		else if((densityNum == 0 and lengthNum == 1) or (densityNum == 0 and lengthNum == 0) or (densityNum == 1 and lengthNum == 0)){
			phrase = "really sure because my plan is " + densityDifftoPhrase() + " less crowded and only " + lengthDiffToPhrase() + " longer than your plan";
		}
		return phrase;
	}

	std::string computeRiskConf(){
		ROS_INFO_STREAM("Inside compute risk conf");
		std::string phrase, tempphrase;
		if (riskNum == 5) tempphrase = riskDifftoPhrase();
		if (lengthNum == 5) tempphrase = lengthDiffToPhrase();
		ROS_INFO_STREAM((planCrowdRisk - originalPlanCrowdRisk) << " " << riskNum << " " << (planLength - originalPlanLength) << " " << lengthNum);
		if ((riskNum == 0 and lengthNum == 2) or (riskNum == 1 and lengthNum == 1) or (riskNum == 2 and lengthNum == 0)){
			phrase = "only somewhat sure because even though my plan is " + riskDifftoPhrase() + " less risky, it is also " + lengthDiffToPhrase() + " longer than your plan";
		}
		else if((riskNum == 1 and lengthNum == 2) or (riskNum == 2 and lengthNum == 2) or (riskNum == 2 and lengthNum == 1)){
			phrase = "not sure because my plan is " + lengthDiffToPhrase() + " longer than your plan and only " + riskDifftoPhrase() + " less risky";
		}
		else if((riskNum == 0 and lengthNum == 1) or (riskNum == 0 and lengthNum == 0) or (riskNum == 1 and lengthNum == 0)){
			phrase = "really sure because my plan is " + riskDifftoPhrase() + " less risky and only " + lengthDiffToPhrase() + " longer than your plan";
		}
		return phrase;
	}

	std::vector<std::string> parseText(string text){
		//ROS_INFO_STREAM("Inside parse text");
		std::vector<std::string> vstrings;
		std::stringstream ss;
		ss.str(text);
		std::string item;
		char delim = '\t';
		while (std::getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		return vstrings;
	}
	
	void clearStats() {
		//ROS_INFO_STREAM("Inside clear stats");
		planLength=0, originalPlanLength=0, planCrowdDensity=0, originalPlanCrowdDensity=0;
		targetX=0, targetY=0, robotY=0, robotX=0;
		sameplan=1;
		computationTimeSec=0.0;
		densityNum=5, lengthNum=5, riskNum=5;
	}
	
	void logExplanationData() {
		//ROS_INFO_STREAM("Inside log explanation data");
		std_msgs::String logData;
		
		std::stringstream output;
		output << atof(parseText(current_log)[0].c_str()) << "\t" << atof(parseText(current_log)[1].c_str()) << "\t" << atof(parseText(current_log)[2].c_str()) << "\t" << computationTimeSec << "\t" << sameplan << "\t" << planLength << "\t" << originalPlanLength << "\t" << (planLength - originalPlanLength) << "\t" << planCrowdDensity << "\t" << originalPlanCrowdDensity << "\t" << (planCrowdDensity - originalPlanCrowdDensity);
		
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
