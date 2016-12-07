
#ifndef REMOTEMODE_H_
#define REMOTEMODE_H_


#include <turtle_commander_master/robot.h>
#include <turtle_commander_master/register.h>

struct Registration;
struct getStarted;
struct sendNewGoal;
struct waitForReceive;
struct travelToGoal;
struct goalReached;
struct waitToTravel;
struct RemoteModePause; // default state (not used!)
struct RemoteMode : sc::state< RemoteMode, Robot, RemoteModePause >
{
  friend class getStarted;
  friend class sendNewGoal;
  friend class travelToGoal;
  friend class goalReached;
  friend class waitToTravel;
  

  using reactions = mpl::list< sc::custom_reaction< EvSendGoal >,
			       sc::custom_reaction< EvGoalReceived >,
			       sc::custom_reaction< EvDocking >,
			       sc::transition< EvRegistration, Registration >,
                               sc::transition< EvUnregistered, Idle >,
			       sc::transition< EvSendNextGoal, sendNewGoal >,
			       sc::transition< EvGoalReached, goalReached >,
			       sc::transition< EvGoalNOTReceived, sendNewGoal >,
			       sc::transition< EvGoalNotFree, waitToTravel>,
			       sc::transition< EvStuck, Stuck >>;
  			       
  RemoteMode(my_context ctx): my_base(ctx){};
  
  // declaration of the reactions
  sc::result react( const EvSendGoal& ev );
  sc::result react( const EvGoalReceived& ev );  
  sc::result react( const EvDocking& ev );
  
};

// definition of the state getStarted
struct getStarted : sc::state<getStarted, RemoteMode>
{
    getStarted(my_context ctx);
};

//definition of the state sendNewGoal
struct sendNewGoal : sc::state<sendNewGoal, RemoteMode>
{
    sendNewGoal(my_context ctx);
};


//definition of the state sendNextGoal
struct sendNextGoal : sc::state<sendNextGoal, RemoteMode>
{
    sendNextGoal(my_context ctx);
};

//definition of the state travelToGoal
struct travelToGoal : sc::state<travelToGoal, RemoteMode>
{
    travelToGoal(my_context ctx);
};

//definition of the state goalReached
struct goalReached : sc::state<goalReached, RemoteMode>
{
    goalReached(my_context ctx);
};

// definition of the state waitToTravel
struct waitToTravel : sc::state<waitToTravel, RemoteMode>
{
    waitToTravel(my_context ctx);
};
#endif

