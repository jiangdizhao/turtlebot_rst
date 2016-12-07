
/* 
 * Register should open the possibility to expand the state machine
 * with an register function at the beginning or to change the mode
 * into random mode
*/

#include <turtle_commander_master/register.h>
#include <turtle_commander_master/idle.h>

Registration::Registration(my_context ctx) : my_base(ctx)
{
  outermost_context().commonData().active = true;  
  ROS_INFO_STREAM("Turtlebot_" << outermost_context().commonData().turtlebot_no << ": is activated");
  outermost_context()._dest_list.clear();
  outermost_context()._reached_goals.clear();
  outermost_context().commonData().total_riding_time = 0;
  
  // start bond with client (method will do nothing if already bonded)
  outermost_context().startBond();

  post_event(EvStartRemote());
}
