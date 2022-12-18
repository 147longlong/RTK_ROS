/*------------------------------------------------------------------------------
* gnss_data.h : header of navlib ros server
*
*          Copyright (C) 2020 by Cheng Chi, All rights reserved.
*
* notes   :
*     
*
* version : $Revision:$ $Date:$
* history : 2020/04/03 1.0  new
*-----------------------------------------------------------------------------*/
#ifndef GNSS_DATA_H
#define GNSS_DATA_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <gnss_observations/Observation.h>
#include <gnss_observations/EpochData.h>
#include <signal.h>

#include <rtklib.h>

using namespace std;

#define SIZE_STREAM 65536

class gnss_data_t {

public:

	gnss_data_t(ros::NodeHandle& nh);
	~gnss_data_t();
};

bool                    post_proc;					// post processing or real-time
ros::Publisher          obs_pub;    				// observations publisher
ros::Publisher          rtk_pub;                    // rtklib solution publisher
vector<ros::Publisher>  stream_pub; 				// gnss stream publisher
vector<ros::Subscriber> stream_sub; 				// gnss stream subscriber
unsigned char 			stream[3][SIZE_STREAM];		// raw stream
unsigned int 	        nstream[3];					// size of buff
bool					stream_updated[3];			// flags for rtk server
double					current_position[2];		// current spp position (lat, long)

#define STREAM_CALLBACK(n)												\
void stream_callback_##n(const std_msgs::String::ConstPtr& msg)			\
{																		\
	if (!stream_updated[n]) {											\
	    nstream[n] = msg->data.size();									\
	    for (unsigned ii = 0; ii < nstream[n]; ii++) {					\
	        stream[n][ii] = (unsigned char)msg->data[ii];				\
	    } 																\
	}																	\
	else {																\
		nstream[n] += msg->data.size();									\
	    for (unsigned ii = nstream[n]-msg->data.size();				    \
	    	 ii < nstream[n]; ii++) {									\
	        stream[n][ii] = (unsigned char)msg->data[ii];				\
	    } 																\
	}																	\
    stream_updated[n] = true;											\
}
STREAM_CALLBACK(0)
STREAM_CALLBACK(1)
STREAM_CALLBACK(2)

#define stream_callback(n) stream_callback_##n

gnss_data_t* gnss_data;

void shutdown_handler(int sig);

#endif /*GNSS_DATA_H*/

