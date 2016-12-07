
/* 
 * definition of the events with overloaded constructor
*/
#include <turtle_commander_master/events.h>



EvSendGoal::EvSendGoal(const std::string& goal_name) : _goal_name(goal_name) { };

EvDocking::EvDocking(const std::string& docking_name) : _docking_name(docking_name) { };
