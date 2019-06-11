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
#include <map>
#include <cmath>
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
	bool rot_received;
	// other parameters
	vector<double> peopleX;
	vector<double> peopleY;
	vector<double> peopleTheta;
	vector<vector<double> > peopleLaserScans;
	vector<int> peopleFrame;
	map<int,vector<vector<double> > > pointsByFrame;
	vector<vector<double> > targets;

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
		rot_received = false;
		cmd_vel_received = false;
	}

	void updateCmdVel(const geometry_msgs::Twist & cmd_vel){
		if(cmd_vel.linear.x == 0.01){
			rot_received = true;
		}
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
			if(fileLine[0] == '#'){
				std::vector<std::string> vstrings = parseText(fileLine, '\t');
				if(vstrings.size() > 10){
					vector<double> target;
					target.push_back(atof(vstrings[1].c_str()));
					target.push_back(atof(vstrings[2].c_str()));
					targets.push_back(target);
				}
			}
			else{
				std::vector<std::string> vstrings = parseText(fileLine, '\t');
				ROS_DEBUG_STREAM("File text: 0 " << vstrings[0] << " 1 " << vstrings[1] << " 2 " << vstrings[2] << " 3 " << vstrings[3]);
				peopleX.push_back(atof(vstrings[0].c_str()));
				peopleY.push_back(atof(vstrings[1].c_str()));
				peopleFrame.push_back(atof(vstrings[2].c_str()));
				peopleTheta.push_back(atof(vstrings[3].c_str())/180.0*M_PI);
				vector<double> pose;
				pose.push_back(atof(vstrings[0].c_str()));
				pose.push_back(atof(vstrings[1].c_str()));
				pose.push_back(atof(vstrings[3].c_str()));
				pointsByFrame[atof(vstrings[2].c_str())].push_back(pose);
				std::vector<std::string> lstrings = parseText(vstrings[6], ';');
				vector<double> laserScanData;
				for(int i=0; i < lstrings.size(); i++){
					laserScanData.push_back(atof(lstrings[i].c_str()));
					//ROS_DEBUG_STREAM("File text:" << lstrings[i]);
				}
				peopleLaserScans.push_back(laserScanData);
			}
		}
		ros::spinOnce();
	}
	
	void run(bool first){
		ros::Rate rate(30.0);
		while(nh_.ok()) {
			while(cmd_vel_received == false and first == false){
				ROS_DEBUG("Waiting for cmd_vel");
				//wait for some time
				rate.sleep();
				// Sense input 
				ros::spinOnce();
			}
			ROS_INFO_STREAM("Cmd_vel message received");

			// Calculate Pose
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header.frame_id = "map";
			poseStamped.header.stamp = ros::Time::now();
			poseStamped.pose.position.x = peopleX[0];
			poseStamped.pose.position.y = peopleY[0];
			poseStamped.pose.position.z = 0;
			poseStamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, peopleTheta[0]);

			// Calculate Laser Scan
			sensor_msgs::LaserScan ls;
			ls.header.stamp = ros::Time::now();
			ls.header.frame_id = "base_scan";
			for(int i = 0;i < 660;i++){
				ls.ranges.push_back(0);
			}
			for(int i = 0;i < 660;i++){
				ls.ranges[i] = peopleLaserScans[0][i];
			}
			ls.angle_min = -110.0/180.0*M_PI;
			ls.angle_max = 110.0/180.0*M_PI;
			ls.angle_increment = (1.0/3.0)/180.0*M_PI;
			ls.range_max = 25.0;

			//Calculate Crowd All Array
			geometry_msgs::PoseArray crowd_all;
			for(int i = 0; i < pointsByFrame[peopleFrame[0]].size(); i++){
				geometry_msgs::Pose pose;
				pose.position.x = pointsByFrame[peopleFrame[0]][i][0];
				pose.position.y = pointsByFrame[peopleFrame[0]][i][1];
				pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, pointsByFrame[peopleFrame[0]][i][2]);
				if(pose.position.x != peopleX[0] and pose.position.y != peopleY[0]){
					crowd_all.poses.push_back(pose);
				}
				/*double distance = sqrt((pose.position.x - peopleX[0])*(pose.position.x - peopleX[0])+(pose.position.y - peopleY[0])*(pose.position.y - peopleY[0]));
				if(distance < 25){
					double rangeBeginning = peopleTheta[0] + ls.angle_min;
					if(rangeBeginning < -M_PI){
						rangeBeginning = rangeBeginning + 2*M_PI;
					}
					for(int j = 0;j < 660;j++){
						double combinedAngle = rangeBeginning + j*ls.angle_increment;
						if(combinedAngle > M_PI){
							combinedAngle = combinedAngle - 2*M_PI;
						}
						if()
					}
				}*/
			}
			crowd_all.header.stamp = ros::Time::now();
			crowd_all.header.frame_id = "map";

			//Calculate Crowd Array
			geometry_msgs::PoseArray crowd;
			for(int i = 0; i < crowd_all.poses.size(); i++){
				//double distance = sqrt((crowd_all.poses[i].position.x - peopleX[0])*(crowd_all.poses[i].position.x - peopleX[0])+(crowd_all.poses[i].position.y - peopleY[0])*(crowd_all.poses[i].position.y - peopleY[0]));
				double minDistance = 1000;
				double rangeBeginning = peopleTheta[0] + ls.angle_min;
				if(rangeBeginning < -M_PI){
					rangeBeginning = rangeBeginning + 2*M_PI;
				}
				for(int j = 0;j < 660;j++){
					double combinedAngle = rangeBeginning + j*ls.angle_increment;
					if(combinedAngle > M_PI){
						combinedAngle = combinedAngle - 2*M_PI;
					}
					double slope, b, standardA, standardB, standardC;
					bool vertical_line = false;
					if(combinedAngle == -M_PI/2 || combinedAngle == M_PI/2){
						vertical_line = true;
					}
					if(!vertical_line){
						slope = tan(combinedAngle);
						b = peopleY[0] - slope*peopleX[0];
						standardA = -slope;
						standardB = 1;
						standardC = b;
					}
					else{
						standardA = 1;
						standardB = 0;
						standardC = -peopleX[0];
					}
					double numerator = abs(standardA*crowd_all.poses[i].position.x + standardB*crowd_all.poses[i].position.y + standardC);
					double denominator = sqrt(standardA*standardA + standardB*standardB);
					double distanceFromCenter = numerator/denominator;
					if(distanceFromCenter < minDistance){
						minDistance = distanceFromCenter;
					}
				}
				if (minDistance <= 0.25){
					crowd.poses.push_back(crowd_all.poses[i]);
				}
			}
			crowd.header.stamp = ros::Time::now();
			crowd.header.frame_id = "map";

			//send the next person's data
			crowd_pose_pub_.publish(crowd);
			crowd_pose_all_pub_.publish(crowd_all);
			pose_pub_.publish(poseStamped);
			laser_pub_.publish(ls);

			if((peopleX[0]==targets[0][0] and peopleY[0]==targets[0][1])){
				ROS_INFO_STREAM("Target reached");
				peopleX.erase(peopleX.begin());
				peopleY.erase(peopleY.begin());
				peopleTheta.erase(peopleTheta.begin());
				peopleLaserScans.erase(peopleLaserScans.begin());
				peopleFrame.erase(peopleFrame.begin());
				targets.erase(targets.begin());
			}
			else if(rot_received == false){
				ROS_INFO_STREAM("Forward move detected");
				peopleX.erase(peopleX.begin());
				peopleY.erase(peopleY.begin());
				peopleTheta.erase(peopleTheta.begin());
				peopleLaserScans.erase(peopleLaserScans.begin());
				peopleFrame.erase(peopleFrame.begin());
			}

			cmd_vel_received = false;
			rot_received = false;
			ROS_INFO_STREAM("People loop completed");
			//wait for some time
			rate.sleep();
			// Sense input 
			ros::spinOnce();
			first = false;
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
	generator.run(true);

	return 0;
}
