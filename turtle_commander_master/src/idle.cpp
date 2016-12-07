
/* 
 * Idle is the default state without a registration
*/
#include <turtle_commander_master/idle.h>
#include <turtle_commander_master/register.h>


// transition for the event EvStartRemote
sc::result Idle::react( const EvRegistration& ev )
{  
  return transit<Registration>();
}