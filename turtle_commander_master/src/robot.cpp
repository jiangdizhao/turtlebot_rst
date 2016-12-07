
#include <turtle_commander_master/robot.h>
#include <turtle_commander_master/register.h>
#include <turtle_commander_master/idle.h>

// Constructor of the robot, set turtlebot_number and save a pointer to the service client
Robot::Robot(int turtlebot_no, bool remote_mode)
{
  commonData().turtlebot_no = turtlebot_no;
  
  // specify service for client commands
  std::string client_service = "/robot_" + std::to_string(turtlebot_no) + "/turtle_commander_client/cmd_from_master"; // TODO param
  _cmd_client = _nhandle.serviceClient<tcm::CommandService>(client_service); 
}

void Robot::onClientBoundFormedCallback()
{
    ROS_INFO("Turtlebot_%d: bond to client established.", commonData().turtlebot_no);
}

void Robot::onClientBoundBrokenCallback()
{
   deactivateRobot();
   
   post_event( EvUnregistered() );
   
   ROS_INFO("Turtlebot_%d: bond to client broken.", commonData().turtlebot_no); 
}


bool Robot::setNextGoal() 
{ 
  if(!_dest_list.empty())
  { 
    std::string new_goal = _dest_list.front();
    _dest_list.pop_front();
    _common_data.current_goal = new_goal;
    return true; 
  } 
  else 
  {
    return false;
  }  
}

void Robot::deactivateRobot() 
{ 
  _common_data.active = false; 
  _common_data.current_goal = "";
  _dest_list.clear();
  
};

bool Robot::sendClientCommand(const std::string& cmd)
{
    turtle_commander_messages::CommandService srv;
    srv.request.turtlebot_number = commonData().turtlebot_no;
    srv.request.command = cmd;
    
    if(cmd == "GOAL") srv.request.argument = commonData().current_goal;
    if(cmd == "DOCKING_GOAL" ) srv.request.argument = commonData().current_docking;
    
    return _cmd_client.call(srv) && srv.response.success;
}

void Robot::startBond()
{
    if (_client_bond && !_client_bond->isBroken()) // skip starting if already bonded
        return;
        
    // initialize client bond
    // we initialize our bond here instead of in the constructor since we want to create a new bond 
    // whenever the master restarts. If we would not create a new bond, the old process UUID is cached
    // (unfortunately there is no reset method in bondcpp).
    _client_bond = std::unique_ptr<bond::Bond>(new bond::Bond("/turtle_commander_master/client_bond", 
                                                              std::to_string(commonData().turtlebot_no),
                                                              boost::bind(&Robot::onClientBoundBrokenCallback, this),
                                                              boost::bind(&Robot::onClientBoundFormedCallback, this) 
                                                )); 
    _client_bond->setHeartbeatPeriod(4);
    _client_bond->setHeartbeatTimeout(10);

    _client_bond->start();
}


// Constructor of the state stuck
Stuck::Stuck(my_context ctx) : my_base(ctx)
{
  outermost_context().deactivateRobot();
  ROS_INFO_STREAM("Turtlebot_" << outermost_context().commonData().turtlebot_no << ": Get stuck. Please help me!");
}

// transition for the event EvStartRemote
sc::result Stuck::react( const EvRegistration& ev )
{
  return transit<Registration>();
}