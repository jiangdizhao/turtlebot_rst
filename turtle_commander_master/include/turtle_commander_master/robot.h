#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include <turtle_commander_master/types.h>
#include <turtle_commander_master/events.h>
#include <bondcpp/bond.h>


//define state machine
struct Idle;	//defualt state
struct Robot : sc::state_machine< Robot, Idle>
{
    
  Robot(int turtlebot_no, bool remote_mode);
  
  
  struct CommonData
  {
    bool active = false;			// shows if the turtlebot is active now
    int turtlebot_no = 0;			// number of the turtlebot (important for the messages)
    double total_riding_time;			// riding time of this section, will be reseted by a new registration
    ros::Time time_flag;			// hold the time of registration and sending goals
    bool remote_traveling = false;		// bit for the remote traveling (in this case alway one)
    bool waiting_to_travel = false;		// bit to show, that the robot has to wait
    std::string current_goal = "";		// the actual traveling goal
    std::string current_docking = "";		// the actual docking goal
    bool goal_reached = false;			// shows if the turtlebot reached at the goal
    bool goal_received = false;			// shows if the goal was sent to the turtlebot
  };  
  
  ros::ServiceClient _cmd_client;
  std::unique_ptr<bond::Bond> _client_bond;
  
  CommonData& commonData() {return _common_data;}
  
  /**
   * Set the event traveling goal to the actual one
   */
  void setCurrentGoal(const EvSendGoal& ev) { _common_data.current_goal = ev.getGoal(); };
  
   /**
   * Set the event docking goal to the actual one
   */
  void setActualDocking(const EvDocking& ev) { _common_data.current_docking = ev.getDocking(); };
  
  /**
   * Check if _dest_list contains any other goals, to continue traveling
   * If we there is a new goal, write it to actual goal
   * 
   * @return	bool		Shows if a new goal has been set
   */
  bool setNextGoal();
  
  /**
   * Deactivate the turtlebot, because it get stuck or is docking
   * Clears the actual goal and the _dest_list
   */
  void deactivateRobot();
  
  /**
   * Send a new command to the client (robot) and receive acknowledge
   */
  bool sendClientCommand(const std::string& cmd);
  
  /**
   * Publish a new command for this robot
   */
  void publishCommand(std::string cmd);
  

  void startBond();
  
  void onClientBoundFormedCallback();
  
  void onClientBoundBrokenCallback();
  
  
  /**
   * This list contains the next traveling goals
   * Something like a to-do list
   */
  std::list <std::string> _dest_list;
  
  /**
   * This list contains the reached goals in this traveling section
   */
  std::list <std::string> _reached_goals;
  
  
private:
  CommonData _common_data;
  ros::NodeHandle _nhandle = ros::NodeHandle("~");
};


using RobotPtr = std::unique_ptr<Robot>;

/**
 * State Stuck
 * The robot can't move anyway!
 */
struct Stuck : sc::state< Stuck, Robot >
{
  using reactions = mpl::list< sc::custom_reaction< EvRegistration >,
                               sc::transition< EvUnregistered, Idle> >;
  
  Stuck(my_context ctx);
  
  sc::result react(const EvRegistration& ev );
};

#endif //ROBOT