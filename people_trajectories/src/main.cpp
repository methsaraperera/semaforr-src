/* \mainpage People Trajectories
 * \brief Simulates people's trajectories from data.
 *
 * \author Raj Korpan.
 *
 * \version People Trajectories 1.0
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
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

using namespace std;

class Simulator
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to /pose, /laserscan and /crowd_pose and /crowd_pose_all topics
	ros::Publisher pose_pub_;
	ros::Publisher laser_pub_;
	ros::Publisher crowd_pose_pub_;
	ros::Publisher crowd_pose_all_pub_;
	//! We will be listening to \cmd_vel topic
	ros::Subscriber sub_cmd_vel_;
	// Message received
	bool cmd_vel_received;
	// other parameters
	vector<double> peopleX;
	vector<double> peopleY;
	vector<double> peopleTheta;
	vector<vector<double> > peopleLaserScans;

public:
	//! ROS node initialization
	Simulator(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publishers for the topics
		pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
		laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("base_scan", 1);
		crowd_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("crowd_pose", 1);
		crowd_pose_all_pub_ = nh_.advertise<geometry_msgs::PoseArray>("crowd_pose_all", 1);
		sub_cmd_vel_ = nh.subscribe("cmd_vel", 1000, &Simulator::updateCmdVel, this);
		cmd_vel_received = false;
	}

	void updateCmdVel(const geometry_msgs::Twist & cmd_vel){
		cmd_vel_received = true;
		//ROS_INFO_STREAM("Recieved cmd_vel data");
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
			else{
				std::vector<std::string> vstrings = parseText(fileLine, '\t');
				//ROS_DEBUG_STREAM("File text:" << vstrings[0] << " " << vstrings[1] << " " << vstrings[2] << " " << vstrings[3]);
				peopleX.push_back(atof(vstrings[0].c_str()));
				peopleY.push_back(atof(vstrings[1].c_str()));
				peopleTheta.push_back(atof(vstrings[2].c_str()));
				std::vector<std::string> lstrings = parseText(vstrings[3], ';');
				for(int i=0; i < lstrings.size(); i++){
					vector<double> laserScanData;
					laserScanData.push_back(atof(lstrings[i].c_str()));
					peopleLaserScans.push_back(laserScanData);
					//ROS_DEBUG_STREAM("File text:" << lstrings[i] << endl);
				}
			}
		}
		ros::spinOnce();
	}
	
	void run(){
		ros::Rate rate(30.0);
		while(nh_.ok()) {
			while(cmd_vel_received == false){
				//ROS_DEBUG("Waiting for message");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			ROS_INFO_STREAM("Cmd_vel message received");
			geometry_msgs::PoseStamped pose;
			sensor_msgs::LaserScan laser;
			geometry_msgs::PoseArray crowd;
			geometry_msgs::PoseArray crowd_all;
			//send the next person's data
			pose_pub_.publish(pose);
			laser_pub_.publish(laser);
			crowd_pose_pub_.publish(crowd);
			crowd_pose_all_pub_.publish(crowd_all);
			cmd_vel_received = false;
			//ROS_INFO_STREAM("Loop completed");
			//wait for some time
			rate.sleep();
			// Sense input 
			ros::spinOnce();
		}
	}

	std::vector<std::string> parseText(string text, char delim){
		//ROS_INFO_STREAM("Inside parse text");
		std::vector<std::string> vstrings;
		std::stringstream ss;
		ss.str(text);
		std::string item;
		while (std::getline(ss, item, delim)) {
			vstrings.push_back(item);
		}
		return vstrings;
	}
	
};

// Main file : Load configuration file

int main(int argc, char **argv) {

	//init the ROS node
	ros::init(argc, argv, "people_trajectories");
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	string path = ros::package::getPath("people_trajectories");
	string text_config = path + "/config/text.conf";
	Simulator generator(nh);
	generator.initialize(text_config);
	ROS_INFO("People Trajectory Generation Initialized");
	generator.run();

	return 0;
}
