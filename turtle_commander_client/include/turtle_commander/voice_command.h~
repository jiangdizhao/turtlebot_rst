#ifndef VOICE_COMMAND_H_
#define VOICE_COMMAND_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <bondcpp/bond.h>
#include <mutex>
#include <turtle_commander_messages/StateMessage.h>

namespace tcm = turtle_commander_messages;
class TurtleCommander; // forward declaration to allow processing events

class Voice_command
{
	public: 
	        struct Goal
                {
                 std::string name;
                 bool docking;
                };
		Voice_command() {}
		void Initialize(int turtlebot_no, TurtleCommander& turtle_commander);
		boost::optional<Voice_command::Goal> isGoalAvailable();

	protected:
		void voiceServiceCallback(const tcm::StateMessage::ConstPtr& msg);
	private:
	        Goal _goal;
		int _turtlebot_no = 1; 
		bool _initialized = false;
		bool _new_goal_flag = false;
                std::mutex _goal_mutex;
		tcm::StateMessage state_msg;
		ros::NodeHandle n= ros::NodeHandle("~");
		ros::Subscriber sub;
		TurtleCommander* _turtle_commander = nullptr;
		
};
#endif
