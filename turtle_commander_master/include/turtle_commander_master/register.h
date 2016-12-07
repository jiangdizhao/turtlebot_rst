
#ifndef REGISTER_H_
#define REGISTER_H_

#include <turtle_commander_master/events.h>
#include <turtle_commander_master/robot.h>
#include <turtle_commander_master/remote_mode.h>


struct getStarted;
struct Idle;
struct Registration : sc::state< Registration, Robot>
{
  struct EvChangeMode : sc::event< EvChangeMode > {};
  
  using reactions = mpl::list<  sc::transition< EvStartRemote , getStarted>,
                                sc::transition< EvUnregistered, Idle > >;
  
  Registration(my_context ctx);      
};


#endif