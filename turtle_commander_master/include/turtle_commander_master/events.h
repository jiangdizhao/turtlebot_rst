
#ifndef EVENTS_H_
#define EVENTS_H_

#include <turtle_commander_master/types.h>

  struct EvRegistration 	: sc::event< EvRegistration > {};
  struct EvUnregistered         : sc::event< EvUnregistered > {};
  struct EvStartRemote 		: sc::event< EvStartRemote > {};
  struct EvWaitForTravel 	: sc::event< EvWaitForTravel > {};
  struct EvGoalNotFree 		: sc::event< EvGoalNotFree > {};
  struct EvGoalSent 		: sc::event< EvGoalSent > {};
  struct EvGoalReceived 	: sc::event< EvGoalReceived > {};  
  struct EvGoalNOTReceived 	: sc::event< EvGoalNOTReceived > {};
  struct EvTraveling 		: sc::event< EvTraveling > {};
  struct EvGoalReached 		: sc::event< EvGoalReached > {};
  struct EvSendNextGoal 	: sc::event< EvSendNextGoal > {};
  struct EvStuck 		: sc::event< EvStuck > {};
  struct EvIntermediate 	: sc::event< EvIntermediate> {};
  
  struct EvSendGoal 		: sc::event< EvSendGoal > 
  {
      EvSendGoal(const std::string& goal_name);
      const std::string& getGoal() const { return _goal_name; } ;
      
  private:
    std::string _goal_name;
  };  
  
  struct EvDocking 		: sc::event< EvDocking > 
  {
      EvDocking(const std::string& docking_station);
      const std::string& getDocking() const { return _docking_name; };
      
      private:
	std::string _docking_name;

  };
  
#endif