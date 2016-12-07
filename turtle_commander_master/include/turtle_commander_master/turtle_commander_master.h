#ifndef TURTLE_COMMANDER_MASTER_H
#define TURTLE_COMMANDER_MASTER_H

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <limits>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <unordered_map>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>

#include <turtle_commander_messages/EvaluationData.h>
#include <turtle_commander_messages/CommandService.h>
#include <turtle_commander_master/robot.h>
#include <turtle_commander_master/idle.h>
#include <turtle_commander_master/export_data.h>

class TurtleCommanderMaster
{
public:
  // variable declaration
  ros::NodeHandle _nhandle = ros::NodeHandle("~");                              //!< handle to the turtle commander master node
  ros::ServiceServer _cmd_srv;                                                  //!< subscriber to process command requests by the client
  ros::Subscriber _sub_evalData;                                                //!< subscriber to get state of the turtlebots
  ros::Timer _timer;                                                            //!< timer for a cyclic method  
  std::deque<RobotPtr> _robots;                                                 //!< store robot statemachines
  std::list <std::string> _dest_list;                                           //!< list of the active goals
  std::unordered_map<std::string, geometry_msgs::Pose> _nav_goals;              //!< list of the navigation goals
  std::unordered_map<std::string, geometry_msgs::Pose> _docking_goals;          //!< list of the docking goals
  std::list <std::string> _free_docking_st;                                     //!< list of the free docking stations
  std::mutex _goals_mutex;                                                      //!< needed for the YAML Import  
  
  EvaluationSettings& evaluationSettings() {return _evaluation_settings;}
  DataExport _export_data;
  
  // function decalation
  TurtleCommanderMaster();
  
  /**
   * Start the master (blocking call including ros spinner)
   */
  void start();
  
  /**
   * Get a list from the ros parameter server into a unordered map
   * 
   * @param 	nhandle 	Nodehandle to locate the ros parameter server
   * @param 	param 		Name of the parameter on the ros parameter server
   * @param	map_out		Output map, where the list would be saved
   * 
   * @return	bool		Shows if an error occured
   */
  bool parsePoseListFromYaml(ros::NodeHandle* nhandle, const std::string& param, std::unordered_map<std::string, geometry_msgs::Pose>& map_out);
  /**
   * Find a random goal out of _nav_goals, which is different to actual goals of all robots.
   * 
   * @return	string		Name of the chosen random goal
   */
  std::string getRandomGoal();
  /**
   * Find a free docking station next to the last goal of the given robot.
   * 
   * @param	robot_idx	Number of the robot, who want to go docking
   */
  void getNextDockingStation(int robot_idx);
  
  /**
   * Erase all inactive robots from the list of available robots
   * 
   * @remarks this method also defines
   */
  void clearInactiveRobots();
    
  /**
   * Check if some robot needs a new goal or if it didn't answer to the last message
   */
  void processRobotGoals();
  
  /**
   * Find the robot in the list of currently available robots
   * @param turtlebot_no    Number of the robot
   * @returns index of the robot in the _robots container; returns -1 if the robot is not found.
   */
  int findRobot(int turtlebot_no);

  /**
   * Callback for incoming command messages from the robot clients
   * 
   * @param	req		CommandService Request
   * @param	res		CommandService Response
   */
  bool commandServiceCallback(turtle_commander_messages::CommandService::Request& req,
                              turtle_commander_messages::CommandService::Response& res);
    /**
   * Callback for incoming EvaluationData messages from the robots
   * 
   * @param	msg		Evaluation Data Message, which contains new data from a robot
   */
  void evaluationDataCallback(const turtle_commander_messages::EvaluationData::ConstPtr& msg);

  /**
   * Transfer the goal list to the next active robot.
   * 
   * @param	robot             Pointer to the robot object to be activated
   * @param	start_location    Location where the robot starts
   */
  void activateTurtlebot(Robot* robot, const std::string& start_location);
  /**
   * Check if 'element' is part of the given 'list'
   * 
   * @param	robot_idx	Index of the robot in _robots, which get deactivated
   */  
  void deactivateTurtlebot(int robot_idx);
  /**
   * Check if 'element' is part of the given 'list'
   * 
   * @param	list		List which gets scaned
   * @param	element		Searched Element
   */
  bool isElementOfList(std::list<std::string>* list, const std::string& element);
  /**
   * Print the goal list into the console.
   * 
   * @param	list		Printed list
   */
  void printGoalList(std::list<std::string>* list);
  
private:
  EvaluationSettings _evaluation_settings;       //!< Evaluationsetting (see types)
  ros::Time _global_time_flag;                  //!< Flag to hold the time between exporting data
  ros::Duration _duration_skipping;             //!< waiting duration before skipping a taken goal  
};

#endif