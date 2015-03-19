/*****************************************************************************/
/*                            ASTUPS – simplified_ros                        */
/*****************************************************************************/
/*
* Author : Julien Lechalupé - lechalupe.julien@gmail.com
* Creation Date : 2013
* License : BSD-3-Clause
*/

#ifndef NODEMANAGER_HPP_
#define NODEMANAGER_HPP_

class NodeManager
{
private:
	static ros::NodeHandle* _handle;

	/**
	*	Constructor.
	*	Prevents unwanted constructions (singleton)
	*/
	NodeManager()
	{
		_handle = NULL;
	}

public:
	/**
	*	Getters.
	*	Get the global reference of the NodeHandle
	*	Build automaticaly if the reference does not exist
	*/
	static ros::NodeHandle& getInstance()
	{
		if ( _handle == NULL )
		{
			_handle = new ros::NodeHandle("~"); //The tilde is necessary for services
		}

		return (*_handle);
	}

	/**
	*	Destructor.
	*/
	~NodeManager()
	{
		if ( _handle != NULL )
		{
			delete _handle;
		}
	}
};

ros::NodeHandle* NodeManager::_handle = NULL;

#endif //NODEMANAGER_HPP_
