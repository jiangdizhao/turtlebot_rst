#include <turtle_commander/voice_command.h>
#include <turtle_commander/types.h>
#include <turtle_commander/events.h>
#include <turtle_commander/turtle_commander.h>
#include <ros/subscribe_options.h>
#include <boost/bind.hpp>
#include <turtle_commander_messages/StateMessage.h>

void Voice_command::Initialize(int turtlebot_no, TurtleCommander& turtle_commander)
{    
    _turtlebot_no = turtlebot_no;
    
    _turtle_commander = &turtle_commander;
    
    
    // initialize subscriber
    sub = n.subscribe("/turtle_commander_master/command",20,&Voice_command::voiceServiceCallback,this);
      
    _initialized = true;
}

void Voice_command::voiceServiceCallback(const turtle_commander_messages::StateMessage::ConstPtr& msg)
{
    if(msg->turtlebot_number == _turtlebot_no)
    {
        if(msg->state1 == "GOAL")
        {      
            ROS_INFO_STREAM("Goal '" << msg->state2 << "' received from voice_commander.");
            std::lock_guard<std::mutex> l(_goal_mutex);
	    _goal.name = msg->state2;
            _goal.docking = false;
            _new_goal_flag = true;
              
        }
        else if(msg->state1 == "DOCKING_GOAL")
        {
            ROS_INFO_STREAM("Docking station '" << msg->state2 << "' received from voice_commander.");
            std::lock_guard<std::mutex> l(_goal_mutex);
	    _goal.name = msg->state2;
            _goal.docking = true;
            _new_goal_flag = true;
            
        }
    }   
       
}

boost::optional<Voice_command::Goal> Voice_command::isGoalAvailable()
{
    std::lock_guard<std::mutex> l(_goal_mutex);
    
    if (!_new_goal_flag)
        return boost::none;
    
    // reset new goal flag
    _new_goal_flag = false;
    
    return _goal;
}
