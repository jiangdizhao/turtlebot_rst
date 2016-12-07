
#ifndef DOCKING_H_
#define DOCKING_H_

#include <turtle_commander_master/robot.h>
#include <turtle_commander_master/events.h>
#include <turtle_commander_master/remote_mode.h>
#include <turtle_commander_master/register.h>
#include <turtle_commander_master/idle.h>

struct Docking : sc::state< Docking, Robot >
{
  using reactions = mpl::list< 	sc::transition< EvRegistration, Registration >,
			        sc::transition< EvStuck, Stuck >>;  
  Docking(my_context ctx);      
};


#endif //DOCKING_H_