/*****************************************************************************/
/*                            ASTUPS – simplified_ros                        */
/*****************************************************************************/
/*
* Author : Julien Lechalupé - lechalupe.julien@gmail.com
* Creation Date : 2013
* License : BSD-3-Clause
*/

#ifndef ROSPUBLISHER_HPP_
#define ROSPUBLISHER_HPP_

#include <ros/ros.h>
#include <string>
#include <node_manager.hpp>

template <class T>
class RosPublisher
{
protected:
	ros::Publisher _pub;

public:

	/**
	 *	Constructor.
	 *	@param nd Application NodeHandler
	 *	@param bufferSize Size of the sending buffer
	 *	@param publisherName Name of the publisher topic
	 */
	RosPublisher(const std::string& publisherName, int bufferSize = 64)
	{
		_pub = NodeManager::getInstance().advertise<T>(publisherName.c_str(),bufferSize);
	}


	/**
	 *	Destructor
	 */
	~RosPublisher()
	{
		_pub.shutdown();
	}

	/**
	 *	Publishing routine.
	 *	Publish message on the ros network under the name of the topic publisherName
	 *	@param message Data to send
	 */
	void publish(const T& message) const
	{
		_pub.publish(message);
	}
};

#endif //ROSPUBLISHER_HPP_
