/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann, Stephan Kurzawe
 *********************************************************************/

#include <turtle_commander/states_traveling.h>
#include <turtle_commander/substates_traveling.h>
#include <turtle_commander/states_docking.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>
#include <limits>



Traveling::Traveling(my_context ctx) : my_base(ctx)
{
//   ROS_INFO("Traveling started");
 
  std::string move_base_action = "move_base";
  _nhandle.param("move_base_action", move_base_action, move_base_action);
  
  _movebase_client = std::unique_ptr<MoveBaseClient>(new MoveBaseClient(move_base_action, true)); // threaded spinner
  
  bool server_online = false;
  for (int i=0; i<5; ++i)
  {
    server_online = _movebase_client->waitForServer(ros::Duration(1.0)); // wait some seconds and try again
    if (server_online)
      break;
    else 
      ROS_WARN_STREAM("MoveBase action server not found (try " << i+1 << "/5)");
  }
  
  if (!server_online)
  {
    ROS_ERROR_STREAM("MoveBase action server '" << move_base_action << "' not found. Leaving traveling state...");
    post_event(EvGoalMissedFinally());
    return;
  }
  
   std::string service_topic = "/clear_costmaps";
   service_topic = "/robot_" + std::to_string(outermost_context().commonData().turtlebot_number) + "/" + move_base_action + service_topic;				// needed for the namespace declaration!
   
  // clear costmap service
  _clear_costmap_service = _nhandle.serviceClient<std_srvs::Empty>(service_topic);
  
  // Parse goals
  _nhandle.param("goal_frame", _goal_frame, _goal_frame);
  if (!parsePoseListFromYaml("docking_goals", _docking_goals))
  {
    ROS_ERROR("Parsing docking_goals parameter failed.");
  }
  if (!parsePoseListFromYaml("nav_goals", _nav_goals))
  {
    ROS_ERROR("Parsing nav_goals parameter failed.");
  }
  
  // parse other paremeters
  _nhandle.param("traveling_timeout", _traveling_timeout, _traveling_timeout);
  _nhandle.param("nav_goal_idle_time", _nav_goal_idle_time, _nav_goal_idle_time);
  _nhandle.param("nav_error_idle_time", _nav_error_idle_time, _nav_error_idle_time);
  _nhandle.param("max_goal_misses_in_a_row", _max_goal_misses_in_a_row, _max_goal_misses_in_a_row);
  
  // Setup publisher
  _pub_docking_goals = _nhandle.advertise<geometry_msgs::PoseArray>("docking_goals", 5);
  _pub_nav_goals = _nhandle.advertise<geometry_msgs::PoseArray>("nav_goals", 5);

  double publish_frequency = 1;
  _nhandle.param("publish_frequency", publish_frequency, publish_frequency);
  _publish_timer = _nhandle.createTimer(ros::Duration(publish_frequency), &Traveling::publishCallback, this); 
}

Traveling::~Traveling()
{
//   ROS_INFO("Traveling stopped");
  _publish_timer.stop();
  ros::spinOnce(); // clear callback queue before destructing traveling (there could be a possible timer callback)
}

// reaction and transition for the event EvTravelToGoal
sc::result Traveling::react( const EvTravelToGoal& ev )
{    
    return transit<TravelToNavGoal>(&TurtleCommander::setNavGoal, ev);
}

// transition for the event EvTravelToDockingStation
sc::result Traveling::react( const EvTravelToDockingStation& ev )
{
    return transit<TravelToDockingStation>(&TurtleCommander::setDockingStation, ev);
}

// transition for the event EvDockingStationReached
sc::result Traveling::react( const EvDockingStationReached& ev )
{
  // reset counter of failed docking trys!
  outermost_context().commonData().autodocking_misses_in_a_row = 0;
  if(outermost_context().commonData().simulation)
  {
    ROS_INFO("We are in simulation mode and hence ignoring the actual docking manoeuvre. Switching to Idle state");
    outermost_context().commonData().start_location = outermost_context().commonData().current_docking_goal;
    return transit<Idle>();						// don't try to dock if we are in simulation
  }
  else
    return transit<AutoDocking>();
}

sc::result Traveling::react( const EvGoalMissedFinally& ev)
{
    // TODO: get rid of state EvGoalMissedFinally?
    if(outermost_context().commonData().random_mode)
    {
        return transit<SelectNextDockingStation>();
    }
    else if (outermost_context().commonData().remote_mode)
    {
      outermost_context().commonData().navigation_cancled = false;
      
      ROS_INFO("Requesting docking station from master...");
      boost::optional<std::string> docking_goal = outermost_context().masterSync().requestDocking(10); // TODO param -> timeout
      if (docking_goal)
      {
        ROS_INFO_STREAM("Received docking station name '" << *docking_goal << "'.");
        outermost_context().commonData().current_docking_goal = *docking_goal;
        return transit<TravelToDockingStation>();
      }
    }
    return transit<Failure>();
}

// transition for the event EvCannotReachDockingStation
sc::result Traveling::react( const EvDockingStationMissed& ev )
{
    return transit<AutoDockingFailed>();
}

// publish callback
void Traveling::publishCallback(const ros::TimerEvent&)
{
  if (!_publish_timer.isValid())
       return;
  
  geometry_msgs::PoseArray docking_goals;
  poseMapToArray(_docking_goals, docking_goals);
  _pub_docking_goals.publish(docking_goals);
  
  geometry_msgs::PoseArray nav_goals;
  poseMapToArray(_nav_goals, nav_goals);
  _pub_nav_goals.publish(nav_goals);
}

void Traveling::clearCostmap()
{
  std_srvs::Empty service;
  _clear_costmap_service.call(service);
}



// send goal to the move base and wait
bool Traveling::moveBaseToGoal(const geometry_msgs::Pose& goal)
{
        // send goal
        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = context<Traveling>()._goal_frame;
        goal_msg.target_pose.header.stamp = ros::Time::now();
        goal_msg.target_pose.pose = goal;
        
        Traveling& traveling = context<Traveling>();
        traveling._movebase_client->cancelAllGoals();
        ros::Duration(1).sleep();
        ros::Duration traveling_timeout(_traveling_timeout);
        
        if (_movebase_client->isServerConnected())
        {   
            _movebase_client->sendGoal(goal_msg);
            start_time = ros::Time::now();
            
            // we need a while loop to get the odom callbacks!
            while(_nhandle.ok())
            {
                if (!outermost_context().isAllowedToDrive())
                {
                    ROS_INFO("Canceling move base action, since we are not allowed to drive anymore");
                    break;
                }
                
                if(context<Traveling>()._movebase_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    outermost_context().dataAcquisition().data().travel_duration += ros::Time::now() - start_time;
                    return true;
                }
                if(context<Traveling>()._movebase_client->getState() == actionlib::SimpleClientGoalState::ABORTED)
                {
                    outermost_context().dataAcquisition().data().travel_duration += ros::Time::now() - start_time;
                    return false;
                }
                    
                if((ros::Time::now()- start_time) > traveling_timeout)
                {
                    outermost_context().dataAcquisition().data().travel_duration += ros::Time::now() - start_time;
                    ROS_INFO_STREAM("The base failed to reach the goal within " << _traveling_timeout << "s (abort due to timeout).");
                    break;
                }      
                ros::spinOnce();
            }
        }
        else
        {
            ROS_ERROR("Cannot travel to goal, since move_base action server is not connected.");
        }  
        
        traveling._movebase_client->cancelAllGoals();
        return false;
}


// creater an array to publish it
void Traveling::poseMapToArray(std::unordered_map<std::string, geometry_msgs::Pose>& poses, geometry_msgs::PoseArray& array)
{
  std::lock_guard<std::mutex> l(_goals_mutex);
  
  array.header.frame_id = _goal_frame;
  array.header.stamp = ros::Time::now();
  array.poses.clear();
  for (const std::pair<std::string, geometry_msgs::Pose>& pose : poses)
  {
    array.poses.push_back(pose.second);
  }
}


// load goal/docking goals out of the Yaml file
bool Traveling::parsePoseListFromYaml(const std::string& param, std::unordered_map<std::string, geometry_msgs::Pose>& map_out)
{
    std::lock_guard<std::mutex> l(_goals_mutex);
  
    XmlRpc::XmlRpcValue param_yaml;
  
    bool ret_val = true;
    
    if (_nhandle.getParam(param, param_yaml))
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
