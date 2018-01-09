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
	//! We will be listening to \decision_log, \plan, and \original_plan topics
	ros::Subscriber sub_decisionLog_;
	ros::Subscriber sub_plan_;
	ros::Subscriber sub_original_plan_;
	// Current log
	string current_log;
	// Current plans
	nav_msgs::Path current_plan;
	nav_msgs::Path current_original_plan;
	// Message received
	bool init_message_received;
	// Stats on plans
	double planLength, originalPlanLength, planCrowdDensity, originalPlanCrowdDensity;
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

public:
	//! ROS node initialization
	Explanation(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the explanations topic
		plan_explanations_pub_ = nh_.advertise<std_msgs::String>("plan_explanations", 1);
		plan_explanations_log_pub_ = nh_.advertise<std_msgs::String>("plan_explanations_log", 1);
		sub_decisionLog_ = nh.subscribe("decision_log", 1000, &Explanation::updateLog, this);
		sub_plan_ = nh.subscribe("plan", 1000, &Explanation::updatePlan, this);
		sub_original_plan_ = nh.subscribe("original_plan", 1000, &Explanation::updateOriginalPlan, this);
		init_message_received = false;
	}

	void updateLog(const std_msgs::String & log){
		init_message_received = true;
		current_log = log.data;
		//ROS_INFO_STREAM("Recieved log data: " << current_log << endl);
	}

	void updatePlan(const nav_msgs::Path & plan){
		init_message_received = true;
		current_plan = plan;
		//ROS_INFO_STREAM("Recieved log data: " << current_log << endl);
	}

	void updateOriginalPlan(const nav_msgs::Path & original_plan){
		init_message_received = true;
		current_original_plan = original_plan;
		//ROS_INFO_STREAM("Recieved log data: " << current_log << endl);
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
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
				}
				//ROS_DEBUG_STREAM("File text:" << vstrings[0]);
			}
			else if (fileLine.find("density") != std::string::npos){
				std::vector<std::string> vstrings = parseText(fileLine);
				for(int i=1; i < vstrings.size(); i+=2){
					densityThreshold.push_back(atof(vstrings[i].c_str()));
					densityPhrase.push_back(vstrings[i+1]);
					ROS_DEBUG_STREAM("File text:" << vstrings[i+1] << " " << vstrings[i] << endl);
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
			while(init_message_received == false){
				ROS_DEBUG("Waiting for first message");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			gettimeofday(&cv,NULL);
			start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			targetX = atof(parseText(current_log)[4].c_str());
			targetY = atof(parseText(current_log)[5].c_str());
			robotX = atof(parseText(current_log)[6].c_str());
			robotY = atof(parseText(current_log)[7].c_str());

			computePlanLengths();
			computePlanDensities();
			if (comparePlans()) {
				explanationString.data = "I decided to go this way because I think it is just as short and not that crowded.";
			}
			else if ((planLength - originalPlanLength) <= 0) {
				explanationString.data = "I think my way is " + densityDifftoPhrase() + " less crowded.";
			}
			else {
				explanationString.data = "Although there may be a " + lengthDiffToPhrase() + " shorter way, I think my way is " + densityDifftoPhrase() + " less crowded.";
			}
			

			gettimeofday(&cv,NULL);
			end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
			computationTimeSec = (end_timecv-start_timecv);
			logExplanationData();
			//send the explanation
			plan_explanations_pub_.publish(explanationString);
			init_message_received = false;
			clearStats();
			//wait for some time
			rate.sleep();
			// Sense input 
			ros::spinOnce();
		}
	}

	void computePlanLengths(){
		planLength = distance(robotX, robotY, current_plan.poses[0].pose.position.x, current_plan.poses[0].pose.position.y);
		ROS_INFO_STREAM("Initial plan length: " << planLength);
		for(int i = 0; i < current_plan.poses.size()-1; i++){
			planLength += distance(current_plan.poses[i].pose.position.x, current_plan.poses[i].pose.position.y, current_plan.poses[i+1].pose.position.x, current_plan.poses[i+1].pose.position.y);
		}
		ROS_INFO_STREAM("Plan length after loop: " << planLength);
		planLength += distance(current_plan.poses[current_plan.poses.size()-1].pose.position.x, current_plan.poses[current_plan.poses.size()-1].pose.position.y, targetX, targetY);
		ROS_INFO_STREAM("Final plan length: " << planLength);

		originalPlanLength = distance(robotX, robotY, current_original_plan.poses[0].pose.position.x, current_original_plan.poses[0].pose.position.y);
		ROS_INFO_STREAM("Initial orig plan length: " << originalPlanLength);
		for(int i = 0; i < current_original_plan.poses.size()-1; i++){
			originalPlanLength += distance(current_original_plan.poses[i].pose.position.x, current_original_plan.poses[i].pose.position.y, current_original_plan.poses[i+1].pose.position.x, current_original_plan.poses[i+1].pose.position.y);
		}
		ROS_INFO_STREAM("Orig Plan length after loop: " << originalPlanLength);
		originalPlanLength += distance(current_original_plan.poses[current_original_plan.poses.size()-1].pose.position.x, current_original_plan.poses[current_original_plan.poses.size()-1].pose.position.y, targetX, targetY);
		ROS_INFO_STREAM("Final orig plan length: " << originalPlanLength);
	}

	double computeDistance(double x1, double y1, double x2, double y2){
		double distance = sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
		return distance;
	}

	void computePlanDensities(){
		planCrowdDensity = atof(parseText(current_log)[26].c_str());
		ROS_INFO_STREAM("Plan density: " << planCrowdDensity);
		originalPlanCrowdDensity = atof(parseText(current_log)[28].c_str());
		ROS_INFO_STREAM("Orig plan density: " << originalPlanCrowdDensity);
	}

	bool comparePlans(){
		sameplan = 1;
		if (planLength != originalPlanLength){
			sameplan = 0;
			return sameplan;
		}
		else if (planCrowdDensity != originalPlanCrowdDensity) {
			sameplan = 0;
			return sameplan;
		}
		else if (current_plan.poses.size() != current_original_plan.poses.size()) {
			sameplan = 0;
			return sameplan;
		}
		else {
			for(int i = 0; i < current_plan.poses.size()-1; i++){
				if (current_plan.poses[i].pose.position.x != current_original_plan.poses[i].pose.position.x or current_plan.poses[i].pose.position.y != current_original_plan.poses[i].pose.position.y) {
					sameplan = 0;
					return sameplan;
				}
			}
		}
		return sameplan;
	}

	std::string densityDifftoPhrase(){
		return "Test";
	}

	std::string lengthDiffToPhrase(){
		return "Test";
	}

	std::vector<std::string> parseText(string text){
		std::vector<std::string> vstrings;
		std::stringstream ss;
		ss.str(text);
		std::string item;
		char delim = '\t';
		while (std::getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		//std::stringstream ss(text);
		//std::istream_iterator<std::string> begin(ss);
		//std::istream_iterator<std::string> end;
		//std::vector<std::string> vstrings(begin, end);
		//ROS_DEBUG_STREAM("Log text:" << vstrings[0]);
		return vstrings;
	}
	
	void clearStats() {
		planLength=0, originalPlanLength=0, planCrowdDensity=0, originalPlanCrowdDensity=0;
		targetX=0, targetY=0, robotY=0, robotX=0;
		sameplan=1;
		computationTimeSec=0.0;
	}
	
	void logExplanationData() {
		std_msgs::String logData;
		
		std::stringstream output;
		output << sameplan << "\t" << planLength << "\t" << originalPlanLength << "\t" << planCrowdDensity << "\t" << originalPlanCrowdDensity;
		
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
