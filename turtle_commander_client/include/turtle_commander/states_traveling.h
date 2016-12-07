/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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

#ifndef TRAVELING_H_
#define TRAVELING_H_

#include <turtle_commander/turtle_commander.h>
#include <turtle_commander/states_idle.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>

#include <unordered_map>
#include <mutex>
#include "std_msgs/String.h"

struct TravelingPause;
struct GoalReached;
struct GoalMissed;
struct SelectRandomGoal;
struct GetGoalFromMaster;
struct GotGoalFromMaster;
struct TravelToDockingStation;
struct SelectNextDockingStation;
struct Traveling : sc::state< Traveling, TurtleCommander, TravelingPause >
{
    friend struct TravelToNavGoal;
    friend struct GoalReached;
    friend struct GoalMissed;
    friend struct SelectRandomGoal;
    friend struct GetGoalFromMaster;
    friend struct GotGoalFromMaster;
    friend struct SelectNextDockingStation;
    friend struct TravelToDockingStation;
    friend struct DockingStationMissed;
	
    Traveling(my_context ctx);
    ~Traveling();
    
    using reactions = mpl::list< sc::custom_reaction<EvTravelToGoal>,
                                 sc::custom_reaction<EvTravelToDockingStation>,
                                 sc::custom_reaction<EvDockingStationReached>,
                                 sc::custom_reaction<EvDockingStationMissed>,
				 sc::custom_reaction<EvGoalMissedFinally>,
				 sc::transition<EvRandomTraveling, SelectRandomGoal>,
				 sc::transition<EvStopTraveling, Idle>,
				 sc::transition<EvGoalReached, GoalReached>,
				 sc::transition<EvGoalMissed, GoalMissed>,
                                 sc::transition<EvTravelToNextDockingStation, SelectNextDockingStation>, 
                                 sc::transition<EvDockingStationMissedFinally, Failure> >;
				 
    sc::result react(const EvTravelToGoal& ev);
    sc::result react(const EvTravelToDockingStation& ev);			 
    sc::result react(const EvDockingStationReached& ev);   
    sc::result react(const EvGoalMissedFinally& ev);
    sc::result react(const EvDockingStationMissed& ev); 
				 
    const geometry_msgs::Pose& getNavGoal(const std::string& name) const { return _nav_goals.at(name); }; 		// throws out-of-range if not available
    const geometry_msgs::Pose& getDockingGoal(const std::string& name) const { return _docking_goals.at(name); }; 	// throws out-of-range if not available
       
protected:
    /**
    * timer callback to publish nav_goals and docking_goals
    * 
    * @param 	ev	 	Timer Event
    */
    void publishCallback(const ros::TimerEvent&);
    /**
    * Get a list from the ros parameter server into a unordered map
    * 
    * @param 	param 		Name of the parameter on the ros parameter server
    * @param	map_out		Output map, where the list would be saved
    * 
    * @return	bool		Shows if an error occured
    */    
    bool parsePoseListFromYaml(const std::string& param, std::unordered_map<std::string, geometry_msgs::Pose>& map_out);
    /**
    * Get a pose array out of a unordered_map
    * 
    * @param 	poses 		unordered_map which should get into the array
    * @param 	array 		array to save it
    */    
    void poseMapToArray(std::unordered_map<std::string, geometry_msgs::Pose>& poses, geometry_msgs::PoseArray& array);

    /**
    * send pose to the move base, to travel to the goal
    * if it takes longer then the threshold -> cancle and return false
    * 
    * @param 	goal 		pose of the goal
    * 
    * @return 	bool 		true -> arrived at the goal
    */      
    bool moveBaseToGoal(const geometry_msgs::Pose& goal);
    /**
    * clear the global costmap of the robot
    */        
    void clearCostmap();
    /**
    * decide if the robot should select a docking station or should
    * send a docking request to the master
    * 
    * @return 	result 		transition for the state machine
    */     
    sc::result requestDocking();
    
private: 
    ros::NodeHandle _nhandle = ros::NodeHandle("~");			// handle to the turtle_commander_client node
    ros::Publisher _pub_docking_goals;					// publisher for the docking goals
    ros::Publisher _pub_nav_goals;					// publisher for the navigation goals
    ros::Timer _publish_timer;						// timer for the publisher of the docking and navigation goal
        
    // move base client to move the robot
    using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
    std::unique_ptr<MoveBaseClient> _movebase_client;
    
    ros::ServiceClient _clear_costmap_service;				// service to clear the costmap of a robot
    
    std::string _goal_frame = "/map";					// frame of the goals
    std::unordered_map<std::string, geometry_msgs::Pose> _docking_goals;// unordered_map with all possible docking goals
    std::unordered_map<std::string, geometry_msgs::Pose> _nav_goals;	// unordered_map with all possible navigation goals
    std::mutex _goals_mutex;						// mutex for the goal parser
    
    double _traveling_timeout = 1000;					// threshold for the traveling cancling
    double _nav_goal_idle_time = 10;					// stillstand time at a reached goal
    double _nav_error_idle_time = 10;					// stillstand time if the navigation get cancled
    int _max_goal_misses_in_a_row = 10;    				// threshold of maximal retrys to reach a goal
    ros::Time start_time;						// time when the robot startet to travel to the actual goal
};


#endif // TRAVELING_H_
