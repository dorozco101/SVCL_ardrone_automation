#pragma once
 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ESTIMATIONNODE_H
#define __ESTIMATIONNODE_H
 

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "svcl_ardrone_automation/filter_state.h"
#include "svcl_ardrone_automation/tracker.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <dynamic_reconfigure/server.h>
#include "svcl_ardrone_automation/StateestimationParamsConfig.h"
#include "TooN/se3.h"


class DroneKalmanFilter;
class MapView;
class PTAMWrapper;

struct EstimationNode
{
private:
	// comm with drone
	ros::Subscriber navdata_sub; // drone navdata
	ros::Subscriber vel_sub; // to co-read contro commands sent from other thread
	ros::Subscriber vid_sub;
	ros::Subscriber tracker_sub;
	ros::Time lastNavStamp;

	//SVCL Circle finder/tracker
	boost::array<float, 4ul> landMark;
	boost::array<float, 4ul> loc;
	boost::array<float, 2ul> yaw;

	// comm with ptam
	//ros::Subscriber slam_info_sub; // ptam info (tracking quality) etc.
	//tf::TransformListener tf_sub;
	ros::Subscriber svcl_ardrone_automation_sub;
	ros::Publisher svcl_ardrone_automation_pub;
	static pthread_mutex_t svcl_ardrone_automation_CS;

	// output
	ros::Publisher dronepose_pub;

	ros::NodeHandle nh_;

	tf::TransformBroadcaster tf_broadcaster;

	// parameters
	// every [publishFreq]ms, the node publishes the drones predicted position [predTime]ms into the future.
	// this pose can then be used to steer the drone. obviously, the larger [predTime], the worse the estimate.
	// this pose is published on /tf, and simultaneously further info is published on /ardrone/predictedPose
	ros::Duration predTime;
	int publishFreq;

	std::string navdata_channel;
	std::string control_channel;
	std::string output_channel;
	std::string video_channel;
	std::string command_channel;
	std::string tracker_channel;



	// for navdata time-smoothing
	long lastDroneTS;
	long lastRosTS;
	long droneRosTSOffset;


	// save last navinfo received for forwarding...
	ardrone_autonomy::Navdata lastNavdataReceived;

public:
	// filter
	DroneKalmanFilter* filter;
	PTAMWrapper* ptamWrapper;
	MapView* mapView;
	std::string packagePath;

	EstimationNode();
	~EstimationNode();


	// ROS message callbacks
	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
	void velCb(const geometry_msgs::TwistConstPtr velPtr);
	void vidCb(const sensor_msgs::ImageConstPtr img);
	void comCb(const std_msgs::StringConstPtr str);
	void trackerCb(const svcl_ardrone_automation::tracker& msg);
	void dynConfCb(svcl_ardrone_automation::StateestimationParamsConfig &config, uint32_t level);

	//SVCL code
	inline void getCircle(boost::array<float, 4ul> &m_landMark, boost::array<float, 4ul> &m_loc, boost::array<float, 2ul> &m_yaw)
	{m_landMark = landMark; m_loc = loc; m_yaw = yaw;}

	// main pose-estimation loop
	void Loop();

	// writes a string message to "/svcl_ardrone_automation/com".
	// is thread-safe (can be called by any thread, but may block till other calling thread finishes)
	void publishCommand(std::string c);
	void reSendInfo();

	void publishTf(TooN::SE3<>, ros::Time stamp, int seq, std::string system);

	// logging stuff
	// logging stuff
	std::ofstream* logfileIMU;
	std::ofstream* logfilePTAM;
	std::ofstream* logfileFilter;
	std::ofstream* logfilePTAMRaw;
	static pthread_mutex_t logIMU_CS;
	static pthread_mutex_t logPTAM_CS;
	static pthread_mutex_t logFilter_CS;
	static pthread_mutex_t logPTAMRaw_CS;
	long currentLogID;
	long startedLogClock;

	void toogleLogging();	// switches logging on or off.
	std::string calibFile;
	int arDroneVersion;


};
#endif /* __ESTIMATIONNODE_H */
