
#include <turtle_commander_master/remote_mode.h>
#include <turtle_commander_master/docking.h>
#include <turtle_commander_master/register.h>
#include <turtle_commander_master/idle.h>

// definition of the reaction for the Event EvSendGoal
sc::result RemoteMode::react( const EvSendGoal& ev )
{
    return transit<sendNewGoal>(&Robot::setCurrentGoal, ev);
}

// definition of the reaction for the Event EvGoalReceived
sc::result RemoteMode::react( const EvGoalReceived& ev )
{
    outermost_context().commonData().goal_received = true;
    return transit<travelToGoal>();
}

// definition of the reaction for the Event EvDocking
sc::result RemoteMode::react( const EvDocking& ev )
{
    return transit<Docking>(&Robot::setActualDocking, ev);
}

// constructor for the state getStarted (default state)
getStarted::getStarted(my_context ctx) : my_base(ctx){};

// constructor for the state sendNewGoal
sendNewGoal::sendNewGoal(my_context ctx) : my_base(ctx)
{
  outermost_context().commonData().goal_reached = false;
  outermost_context().commonData().waiting_to_travel = false;
  
  if (outermost_context().sendClientCommand("GOAL"))
  {    
    ROS_INFO_STREAM("Turtlebot_"<< outermost_context().commonData().turtlebot_no <<": Send Goal to the turtlebot: " << outermost_context().commonData().current_goal);
    outermost_context().commonData().goal_received = true;
    post_event(EvGoalReceived());
  }
  else 
  {
    outermost_context().commonData().goal_received = false;
    ROS_WARN("sendNewGoal() failed");
  }
};

// constructor for the state travelToGoals
travelToGoal::travelToGoal(my_context ctx) : my_base(ctx)
{
  ROS_INFO_STREAM("Turtlebot_"<< outermost_context().commonData().turtlebot_no<<": Start Traveling to: " << outermost_context().commonData().current_goal);
};

// constructor for the state goalReached
goalReached::goalReached(my_context ctx) : my_base(ctx)
{
  outermost_context().commonData().goal_reached = true;
  outermost_context()._reached_goals.push_back(outermost_context().commonData().current_goal);
  ROS_INFO_STREAM("Turtlebot_"<< outermost_context().commonData().turtlebot_no << ": Reached at Goal! Waiting for next one!");

  ros::Duration(2).sleep();
};

// constructor for the state waitToTravel
// this state means that somebody else is driving to the new goal
waitToTravel::waitToTravel(my_context ctx) : my_base(ctx)
{
  outermost_context().commonData().goal_reached = false;
  outermost_context().commonData().waiting_to_travel = true;
  ROS_INFO_STREAM("Turtlebot_" << outermost_context().commonData().turtlebot_no << ": I have to wait for another turtlebot. (" << outermost_context().commonData().current_goal << ")" );
};
