/*****************************************************************************/
/*                            ASTUPS – simplified_ros                        */
/*****************************************************************************/
/*
* Author : Julien Lechalupé - lechalupe.julien@gmail.com
* Creation Date : 2013
* License : BSD-3-Clause
*/

#ifndef ROSSUBSCRIBER_HPP_
#define ROSSUBSCRIBER_HPP_

#include <ros/ros.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <node_manager.hpp>

template <class T>
class RosSubscriber
{
private:
	/**
	 *	Private Method.
	 *	Listen callback, called when piece of data received
	 */
	void listenCallback(const boost::shared_ptr<T>& ptrData)
	{
		if ( ptrData != NULL )
		{
			_data = (*ptrData);
			_is_available = true;
		}
	}

protected:
	ros::Subscriber _sub;
	T _data;
	bool _is_available;

public:
	/**
	 *	Constructor.
	 *	@param bufferSize Size of the sending buffer
	 *	@param publisherName Name of the subscriber topic
	 *	@param callback Function pointer on the callback function
	 */
	RosSubscriber(const std::string& subscriberName,int bufferSize = 64, void(* callback)(const boost::shared_ptr<T>&) = NULL)
	{
		if ( callback == NULL )
		{
			_sub = NodeManager::getInstance().subscribe(subscriberName,bufferSize,&RosSubscriber::listenCallback,this);
		}
		else
		{
			_sub = NodeManager::getInstance().subscribe(subscriberName,bufferSize,callback);
		}

		_is_available = false;
	}


	/**
	 *	Destructor.
	 */
	~RosSubscriber()
	{
		_sub.shutdown();
	}

	/**
	 *	Getter.
	 *	Get the data received and filtered by the function selector
	 *	@return Data received
	 */
	T getData() throw(std::string)
	{
		if ( !_is_available )
		{
			throw(std::string("Error: no data available"));
		}

		_is_available = false;

		return _data;
	}

	/**
	 *	Getter.
	 *	Get boolean representing data availability
	 *	@return True if data available else otherwise
	 */
	bool isDataAvailable() const
	{
		return _is_available;
	}

	/**
	 *	Static method.
	 *	Call all the subscriber callbacks via ROS
	 *	callAllCallbacks => (isDataAvailable <- if (dataReceived) true else false )
	 */
	static void executeCallbacks()
	{
		ros::spinOnce();
	}
};

#endif //ROSSUBSCRIBER_HPP_
