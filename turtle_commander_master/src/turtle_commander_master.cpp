#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <turtle_commander_master/turtle_commander_master.h>

TurtleCommanderMaster::TurtleCommanderMaster()
{   
  int time_before_skipping = 130;
  // get the waiting time before skipping a goal from the rosparam server
  _nhandle.param("time_before_skipping", time_before_skipping, time_before_skipping);
  
  // get the duration time of the timer callback from the rosparam server
  int timer_callback_duration = 5;		// time in seconds of the timer callback 	(default = 5 sec)
  _nhandle.param("timer_callback_duration", timer_callback_duration, timer_callback_duration);
  
  // convert get skipping time
  _duration_skipping = ros::Duration(time_before_skipping);
  
  //get Evaluation Settings
  std::string work_path = "/tmp/";
  _nhandle.param("work_path", work_path, work_path);
  _evaluation_settings.configuration_file_routes = work_path + "config_routes.txt";
  _evaluation_settings.configuration_file_planner = work_path + "config_planner.txt";
  _evaluation_settings.evaluation_file = work_path + "evaluation_data.mat";
  
  _nhandle.param("number_v_bins", _evaluation_settings.number_v_bins, _evaluation_settings.number_v_bins);
  _nhandle.param("number_omega_bins", _evaluation_settings.number_omega_bins, _evaluation_settings.number_omega_bins);
  _nhandle.param("v_max", _evaluation_settings.v_max, _evaluation_settings.v_max);
  _nhandle.param("omega_max", _evaluation_settings.omega_max, _evaluation_settings.omega_max);
    
  // load goals out of the rosparam
  if(!parsePoseListFromYaml(&_nhandle, "nav_goals", _nav_goals))
  {
    ROS_ERROR_STREAM("Couldn't load navigation goals from YAML!"); 
  }  
  
  // load goals out of the rosparam
  if(!parsePoseListFromYaml(&_nhandle, "docking_goals", _docking_goals))
  {
    ROS_ERROR_STREAM("Couldn't load goals from YAML!"); 
  }
  
  // get a list of the available docking goals
  for (const auto& list_it : _docking_goals)
  {
    _free_docking_st.push_back(list_it.first);
  }
  
    // define publisher for the goals
//   _pub_goal = _nhandle.advertise<turtle_commander_messages::StateMessage>("/turtle_commander_master/command" ,30);
  
  // define services
  _cmd_srv = _nhandle.advertiseService("/turtle_commander_master/cmd_to_master",  &TurtleCommanderMaster::commandServiceCallback, this);
  
  // define subscriber for the evaluationData
  _sub_evalData = _nhandle.subscribe("/turtle_commander_master/evaluation_data", 30, &TurtleCommanderMaster::evaluationDataCallback, this);
        
  // initialize the export_data class
  _export_data.init(&evaluationSettings(), &_nav_goals);
  
  // clear the Destination list
  _dest_list.clear();
  // save starting time
  _global_time_flag = ros::Time::now();
  
  ROS_INFO("TurtleCommander master initialized.");
}

void TurtleCommanderMaster::start()
{
    ros::Rate r(10);
    while(ros::ok())
    {
        clearInactiveRobots();
        processRobotGoals();
        ros::spinOnce();
        r.sleep();
    }
}


void TurtleCommanderMaster::clearInactiveRobots()
{
    if (_robots.empty())
        return;
    
    bool new_leader = false;
    
    auto robot = _robots.begin();
    while ( robot != _robots.end() )
    {
        if (!*robot)
        {
            ROS_INFO_STREAM("Unknown turtlebot erased from the list of known robots.");
            if (robot == _robots.begin())
                new_leader = true;
            robot = _robots.erase(robot);
            continue;
        }
        
        if (!robot->get()->commonData().active)
        {
            ROS_INFO_STREAM("Turtlebot_" << robot->get()->commonData().turtlebot_no << " erased from the list of known robots.");
            if (robot == _robots.begin())
                new_leader = true;
            deactivateTurtlebot(std::distance(_robots.begin(), robot));
            robot = _robots.erase(robot);
            continue;
        }
                
        ++robot;
    }
    
    if (_robots.empty())
    {
        ROS_INFO("No active robot available anymore.");
        return;
    }
    
    if (new_leader)
        ROS_INFO_STREAM("Turtlebot_" << _robots.front()->commonData().turtlebot_no << " is the leader now.");
}


int TurtleCommanderMaster::findRobot(int turtlebot_no)
{
    for (int idx = 0; idx < _robots.size(); ++idx)
    {
        if (_robots[idx] && _robots[idx]->commonData().turtlebot_no == turtlebot_no)
            return idx;
    }
    return -1;
}

void TurtleCommanderMaster::processRobotGoals()
{
    // check if we have robots available already
    if (_robots.empty())
        return;
    
    std::string new_goal;
    std::string old_goal;
    old_goal.clear(); 
    
    for (int k=0; k < _robots.size(); ++k)  // the robot container is already sorted according to the first time of appearance
    {      
        // add reached goal to the next active turtlebot
        if(!old_goal.empty())
        {
            _robots[k]->_dest_list.push_back(old_goal);
            old_goal.clear();
        }
        
        // if reached at goal look for the next one
        if(_robots[k]->commonData().goal_reached || (_robots[k]->commonData().current_goal=="") )
        {
            old_goal = _robots[k]->commonData().current_goal;
            
            // the leading robot will get a random goal
            if(k == 0)
            {
                // if it drives out of the docking state clear the docking station
                if(!_robots[k]->commonData().current_docking.empty())
                {
                    _free_docking_st.push_back(_robots[k]->commonData().current_docking);
                    _robots[k]->commonData().current_docking = "";
                }
                
                new_goal = getRandomGoal();
                if (new_goal.empty())
                {
                    ROS_ERROR("processRobotGoals(): cannot find a valid goal randomly.");
                    return;
                }
                _robots[k]->process_event(EvSendGoal(new_goal));
                _robots[k]->commonData().time_flag = ros::Time::now();
                
                _dest_list.push_back(new_goal);
                _dest_list.remove(old_goal);
            }
            // the following robots have to look on their _dest_list
            // check if there is a new goal on the _dest_list
            else if(_robots[k]->setNextGoal())
            {
                _robots[k]->commonData().time_flag = ros::Time::now();
                // check if the goal is free
                if(isElementOfList(&_dest_list,_robots[k]->commonData().current_goal))
                {
                    // goal is not free -> wait here
                    old_goal.clear();
                    _robots[k]->process_event(EvGoalNotFree());
                }
                else
                {
                    // goal is free -> send it as new goal to the turtlebot
                    
                    // if it drives out of the docking state clear the docking station for the next
                    if(!_robots[k]->commonData().current_docking.empty())
                    {
                    _free_docking_st.push_back(_robots[k]->commonData().current_docking);
                    _robots[k]->commonData().current_docking = "";
                    }
                    _robots[k]->process_event(EvSendNextGoal()); 
                    new_goal = _robots[k]->commonData().current_goal;
                    
                    _dest_list.remove(old_goal);
                    _dest_list.push_back(new_goal);
                }
            }
            else 
            {
                // clear old_goal because turtlebot still stands at this goal
                old_goal.clear(); 
            }; 
        }
        // check if the robot got the goal message or send it again after 30sec
        else if(!_robots[k]->commonData().goal_received && ((_robots[k]->commonData().time_flag+ros::Duration(30))<=ros::Time::now()))
        {
            // send the goal message again until the robot answers
            ROS_INFO_STREAM("Turtlebot_" << _robots[k]->commonData().turtlebot_no << ": sending goal again, since I got no response.");
            _robots[k]->commonData().time_flag = ros::Time::now();
            _robots[k]->process_event(EvGoalNOTReceived());
        }
        // check if the goal got free or skip it after three minutes
        else if(_robots[k]->commonData().waiting_to_travel)
        {
            if(!isElementOfList(&_dest_list,_robots[k]->commonData().current_goal))
            {
                // the goal got free and we can travel there
                old_goal = _robots[k]->_reached_goals.back();
                new_goal = _robots[k]->commonData().current_goal;
                
                _dest_list.remove(old_goal);
                _dest_list.push_back(new_goal);
                _robots[k]->process_event(EvSendNextGoal()); 
            }
            else if( (_robots[k]->commonData().time_flag + _duration_skipping) <= ros::Time::now() )
            {
                // if the goal is after xx minutes still not free skip this one
                ROS_INFO_STREAM("Turtlebot_" << _robots[k]->commonData().turtlebot_no << ": I skip the goal, because it doesn't get free.");
                _robots[k]->commonData().current_goal = _robots[k]->_reached_goals.back();
                _robots[k]->commonData().goal_reached = true;
            }
        }
        // print a list of reached goals every ten minutes
        if ((ros::Time::now()-_global_time_flag)>ros::Duration(10*60) && !_robots[k]->_reached_goals.empty()) 
        {
            ROS_INFO_STREAM("Turtlebot_" << _robots[k]->commonData().turtlebot_no << ": I'm traveling now for " << _robots[k]->commonData().total_riding_time/60 << " minutes and reached following goals:");
            printGoalList(&_robots[k]->_reached_goals);
        }     
    }

    
    if((ros::Time::now()-_global_time_flag)>ros::Duration(10*60))
    {    
        // export data to the mat file
        _export_data.exportData();
        _global_time_flag = ros::Time::now();
    }
}


bool TurtleCommanderMaster::commandServiceCallback(turtle_commander_messages::CommandService::Request& req,
                                                     turtle_commander_messages::CommandService::Response& res)
{
  res.success = false;
  
  int robot_idx = findRobot(req.turtlebot_number);
    
  // the turtlebot wants to start traveling
  if(req.command == "REGISTER")
  {    
    // check if the robot isn't active or his last registration is as long ago as five minutes
//     if( !_robots[req.turtlebot_number].commonData().active || ((_robots[req.turtlebot_number].commonData().time_flag+ros::Duration(5*60)) <=ros::Time::now() ) )

    Robot* robot;
    if (robot_idx < 0) // not yet constructed
    {
        _robots.emplace_back( new Robot(req.turtlebot_number, false) );
        _robots.back()->initiate();
        robot = _robots.back().get();
        ROS_DEBUG_STREAM("Turtlebot_" << req.turtlebot_number << " object constructed.");
    }
    else
    {
        robot = _robots[robot_idx].get();
    }
      
    if( !robot->commonData().active )
    {
      res.success = true;
      
      activateTurtlebot(robot, req.argument);
    }
    else 
    {
        ROS_WARN("Registration of turtlebot %d failed. Is the turtlebot already registered?", req.turtlebot_number); 
        res.success = false;
    }
    
    return res.success;
  };
  
  // check if our robot is known, since we do not want to register:
  if (robot_idx<0)
  {
    ROS_WARN_STREAM("Got command '" << req.command << "' from Turtlebot_" << req.turtlebot_number << " which is not registered yet.");
    return false;
  }
  
  // the turtlebott reached at the goal location
  if(req.command == "REACHED")
  {
    _robots[robot_idx]->process_event(EvGoalReached());
    res.success = true;
  }
  
   // the turtlebot wants to go docking
  if(req.command == "DOCKING")
  {
    std::string  actual_docking = _robots[robot_idx]->commonData().current_docking;
    if(actual_docking  == "")
      getNextDockingStation(robot_idx);    
    else
      _robots[robot_idx]->process_event(EvDocking(actual_docking));
    res.success = true;
  }
  
  // the turtlebot got stuck and needs help
  if(req.command == "STUCK")
  {
    ROS_WARN_STREAM("Turtlebot_" << req.turtlebot_number << " got stucked. Deactivating...");
    deactivateTurtlebot(robot_idx);   
   _robots[robot_idx]->process_event(EvStuck());
   res.success = true;
  }
  
  // help function to clear destination list
  if(req.command == "CLEAR_DEST")
  {
    ROS_INFO("Destination list cleared!");
    _dest_list.clear();
    res.success = true;
  }
  
  // help function to delete one goal from the dest_list
  if(req.command == "DEL_GOAL")
  {
    ROS_INFO("Goal deleted from dest list!");
    _dest_list.remove(req.argument);
    res.success = true;
  }
  
  // help function to clear the docking station list
  if(req.command == "")
  {
    ROS_INFO("Free docking station list resetted!");
    _free_docking_st.clear();
    for (const auto& list_it : _docking_goals)
    {
      _free_docking_st.push_back(list_it.first);
    }
    res.success = true;
  }
  
  return res.success;
}

// callback for an incoming evaluationData message
void TurtleCommanderMaster::evaluationDataCallback(const turtle_commander_messages::EvaluationData::ConstPtr& msg)
{  
    ROS_INFO_STREAM("Turtlebot_" << msg->turtlebot_number << ": Got new evaluation Data!");
    _export_data.addData(msg);
    
    int robot_idx = findRobot(msg->turtlebot_number);
    if (robot_idx >= 0)
    {
        _robots[robot_idx]->commonData().total_riding_time += msg->driving_time;
    }
    else
    {
      ROS_WARN_STREAM("Got evaluation data from Turtlebot_" << msg->turtlebot_number << " which is not registered. Saving nevertheless.");
    }
}

// function to get a random goal out of the goal list (without the active goals)
std::string TurtleCommanderMaster::getRandomGoal()
{
    // select random element
    std::string new_goal;
    if (!_robots.empty())
        new_goal = _robots.front()->commonData().current_goal;
    
    for (int i=0; i<50; ++i) // try to find a new goal other than the previous one, but not more than 50 tries...
    {
            int random_index = rand() % _nav_goals.size();
            auto random_it = std::next(std::begin(_nav_goals), random_index);
            new_goal = random_it->first;  
            if (!isElementOfList(&_dest_list, new_goal))
                    break;
    }
    return new_goal;
}

void TurtleCommanderMaster::getNextDockingStation(int robot_no)
{  
  // The bot get his new goal and recognize then, that he isn't allowed to drive,
  // so we look in the area of the last goal (there he would be)
  // for the next docking station
  
  geometry_msgs::Pose current_pos;
  if(!_robots[robot_no]->_reached_goals.empty())
  {
    std::string  last_goal = _robots[robot_no]->_reached_goals.back();
    current_pos = _nav_goals.at(last_goal);
  }
  else
  {
    // the case if there are no reached goals
    current_pos.position.x = 0.0;
    current_pos.position.y = 0.0;
  }
    
    double dist = std::numeric_limits<double>::max();
    std::string nearest_station;
    
    for (const auto& pair_elem : _docking_goals)
    {
      // check if the docking station is free
      if(std::find(_free_docking_st.begin(), _free_docking_st.end(), pair_elem.first) != _free_docking_st.end())
      {
        double cur_dist = std::sqrt(std::pow(pair_elem.second.position.x-current_pos.position.x,2) + std::pow(pair_elem.second.position.y-current_pos.position.y,2));
        if (cur_dist < dist)
        {
            dist = cur_dist;
            nearest_station = pair_elem.first;
        }
      }
    }
    
    if (nearest_station.empty())
    {
        // found no free docking station
        deactivateTurtlebot(robot_no);
        _robots[robot_no]->process_event(EvStuck());
        ROS_ERROR("No docking station found.");
    }
    else
    {
        // send docking station to the turtlebot
        deactivateTurtlebot(robot_no);
        _robots[robot_no]->process_event(EvDocking(nearest_station));
        _free_docking_st.remove(nearest_station);
        return;
    }
}

// function to activate a Turtlebot, checking if he starts from a docking station
// and clearing his goals
void TurtleCommanderMaster::activateTurtlebot(Robot* robot, const std::string& start_location)
{
      // delete his current goal from the dest list
      if(!robot->commonData().current_goal.empty())
        _dest_list.remove(robot->commonData().current_goal);
      
      // if start location contains the word docking
      // it would be a start out of a docking station
      if(strstr(start_location.c_str(), "docking"))
      {
        _free_docking_st.remove(start_location);
        robot->commonData().current_docking = start_location;
        robot->commonData().current_goal = "";      
      }
      else
      {
        _dest_list.push_back(start_location);
        robot->commonData().current_docking = "";
        robot->commonData().current_goal = start_location; 
        // set goal reached true to check for the next one!
        robot->commonData().goal_reached = true;
      }
      robot->process_event(EvRegistration());
}      

// function to deactivate a Turtlebot, it includes get the list of next goals and sent them
// to the next active turtlebot, which isn't the first one
void TurtleCommanderMaster::deactivateTurtlebot(int robot_idx)
{
    // TODO: do we need to do someting here?
//     if (robot_idx >= (int)_robots.size()-1) // robot_idx is already the last one, we can safely deactivate without managing goals
//     {
//         
//     }
    
//     int next_bot = robot_idx+1;
//     if( next_bot != robot_idx)
//     {
      // find the next active turtlebot to set it as the first one
//       if(robot_idx == 0) // if the robot to be deactivated is the leader
//       {      
// 	_robots[next_bot]->_dest_list.clear();
//       }
      
      // if the next bot is now the first one find the next active one
//       if(next_bot == 0)
//       {
// 	next_bot = getNextTurtlebot(next_bot);		// find next bot which drives behind the first
//       }
      
      // if it now still the first robot or the robot itself, we don't have to transfer the goals
//       if(next_bot != 0 && next_bot != robot_idx)
//       {
// 	// take goal list and transfer it to the next active robot
// 	std::list<std::string> docking_goal_list;
// 	docking_goal_list = _robots[robot_idx]->_dest_list;
// 	docking_goal_list.push_back(_robots[robot_idx]->commonData().current_goal);
// 	
// 	if( next_bot != robot_idx && !docking_goal_list.empty() )
// 	{
// 	  ROS_INFO_STREAM("Turtlebot_" << _robots[next_bot]->commonData().turtlebot_no << ": I've got new goals. E.g.: " << docking_goal_list.front() );
// 	  _robots[next_bot]->_dest_list.merge(docking_goal_list);
// 	}
//       }
//     } 
//     else
//     {
//       _dest_list.clear();
// //       ROS_INFO("Nobody drives at the moment!");
//       _export_data.exportData();
//     }
    
    if (robot_idx < _robots.size())
    {
        _robots[robot_idx]->commonData().active = false;
        _dest_list.remove(_robots[robot_idx]->commonData().current_goal);
    }
}


bool TurtleCommanderMaster::isElementOfList(std::list< std::string >* list, const std::string& element)
{
  // if list doesn't include the element std::find will run to the end of the list
  return !(std::find(list->begin(), list->end(), element) == list->end());
}



// load goal/docking goals out of the rosparam server
bool TurtleCommanderMaster::parsePoseListFromYaml(ros::NodeHandle* nhandle, const std::string& param, std::unordered_map<std::string, geometry_msgs::Pose>& map_out)
{
    std::lock_guard<std::mutex> l(_goals_mutex);
  
    XmlRpc::XmlRpcValue param_yaml;
  
    bool ret_val = true;
    
    if (nhandle->getParam(param, param_yaml))
    {
        if(param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray) // list of goals
	{
	    for (int i = 0; i < param_yaml.size(); ++i)
	    {
		if(param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) 
		{
		    for (XmlRpc::XmlRpcValue::iterator it=param_yaml[i].begin(); it!=param_yaml[i].end(); ++it) // goal name + pose (list of doubles)
		    {
			if (it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && it->second.size() == 3)
			{
			  try
			  {
			    auto convDouble = [] (XmlRpc::XmlRpcValue& val) -> double
			    {
			      if (val.getType() == XmlRpc::XmlRpcValue::TypeInt) // XmlRpc cannot cast int to double
				return int(val);
			      return val; // if not double, an exception is thrown;
			    };
			    geometry_msgs::Pose pose;
			    pose.position.x = convDouble(it->second[0]);
			    pose.position.y = convDouble(it->second[1]);
			    pose.orientation = tf::createQuaternionMsgFromYaw( convDouble(it->second[2]) );
			    map_out[it->first] = pose;
			  }
			  catch (const XmlRpc::XmlRpcException& ex)
			  {
			    ROS_ERROR_STREAM("Cannot add current goal: " << ex.getMessage());
			    ret_val = false;
			  }
			}
			else
			{
			  ROS_ERROR_STREAM(param << " with index " << i << " does not define a correct 3d pose");
			  ret_val = false;
			}
		    }
		}
		else
		{
		  ROS_ERROR_STREAM(param << " with index " << i << " is not correct.");
		  ret_val = false;
		}
	    } 
	}
	else
	{
	  ROS_ERROR_STREAM(param << "struct is not correct.");
	  ret_val = false;
	}
    }
    else
    {
        ROS_ERROR_STREAM("Cannot read " << param << " from parameter server");
	ret_val = false;
    }
    return ret_val;
}

void TurtleCommanderMaster::printGoalList(std::list<std::string>* list)
{
  for (std::list<std::string>::iterator it=list->begin(); it != list->end(); ++it)
    std::cout << ' ' << *it;

  std::cout << '\n';
}
