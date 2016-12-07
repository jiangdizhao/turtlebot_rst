
/* 
 * Turtlebot drives to the docking and wait (for charging or because
 * he is'nt allowed to drive!
*/
#include <turtle_commander_master/docking.h>


Docking::Docking(my_context ctx) : my_base(ctx)
{
  // deactivate the Robot (delete actual_goal, _dest_list
  outermost_context().deactivateRobot();
  // send the docking goal to the client
  outermost_context().sendClientCommand("DOCKING_GOAL");  
  ROS_INFO_STREAM("Turtlebot_" << outermost_context().commonData().turtlebot_no << ": I'm docking to " << outermost_context().commonData().current_docking);
  // TODO: check if docking goal is received!
}