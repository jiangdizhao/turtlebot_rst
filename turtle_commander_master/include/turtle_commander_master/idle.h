
#ifndef IDLE_H_
#define IDLE_H_

#include <turtle_commander_master/robot.h>
#include <turtle_commander_master/events.h>
#include <turtle_commander_master/remote_mode.h>

// default state of the robot
// only way is to the register state
struct Idle : sc::state< Idle, Robot >
{
  using reactions = mpl::list< 	sc::custom_reaction< EvRegistration > >;  
  
  sc::result react( const EvRegistration& ev );
  
  Idle(my_context ctx) : my_base(ctx) {};     
};


#endif