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

#include <turtle_commander/substates_traveling.h>
#include <turtle_commander/states_docking.h>

// wait during the turtlebot is traveling to the goal
TravelToNavGoal::TravelToNavGoal(my_context ctx) : my_base(ctx)
{    
    // get current goal
    std::string current_goal = outermost_context().commonData().current_nav_goal;
    
    try
    {
        // check if goal is known, otherwise it throws an exception
        const geometry_msgs::Pose& goal = context<Traveling>().getNavGoal(current_goal);
        
        // send goal
        ROS_INFO_STREAM("Sending nav goal: " << current_goal << " (current battery status: " << outermost_context().getBatteryPercentage() << "%)" );
	outermost_context().commonData().traveling = true;
        if( context<Traveling>().moveBaseToGoal(goal) )
        {
            ROS_INFO_STREAM("Hooray, we arrived at goal: " << current_goal);
	    outermost_context().commonData().traveling = false;
            post_event(EvGoalReached());
            return;
        }
        else
	    outermost_context().commonData().traveling = false;
            ROS_INFO_STREAM("The base failed to move to goal: " << current_goal);
  
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR_STREAM("Current nav goal " << current_goal << " not found. Canceling navigation ...");
    }
    
    post_event(EvGoalMissed()); // in case of an error
}



// select random goal out of the nav_goal list
SelectRandomGoal::SelectRandomGoal(my_context ctx) : my_base(ctx)
{
    ROS_INFO("Random Traveling activated");
    outermost_context().commonData().random_mode = true;
    
    Traveling& traveling = context<Traveling>();  
    
    if (traveling._nav_goals.empty())
    {
        ROS_ERROR("No navigation goals found on parameter server...");
        post_event(EvStopTraveling());
        return;
    }
    
    // select random element
	std::string new_goal = outermost_context().commonData().current_nav_goal;
	for (int i=0; i<50; ++i) // try to find a new goal other than the previous one, but not more than 50 tries...
	{
		int random_index = rand() % traveling._nav_goals.size();
		auto random_it = std::next(std::begin(traveling._nav_goals), random_index);
		new_goal = random_it->first;  
		if (new_goal.compare(outermost_context().commonData().current_nav_goal)!=0) // !=0 -> not equal
			break;
	}
     
    ROS_INFO_STREAM("New random goal: " << new_goal);
    post_event( EvTravelToGoal(new_goal) );
}


// reached goal successfully
GoalReached::GoalReached(my_context ctx) : my_base(ctx)
{
    int sleep_time = std::trunc(context<Traveling>()._nav_goal_idle_time);
    ROS_INFO_STREAM("I traveled "<< outermost_context().dataAcquisition().data().goal_distance << "m and needed "<< outermost_context().dataAcquisition().data().travel_duration.sec << "s. Waiting at goal for " << sleep_time << "s.");
        
    outermost_context().dataAcquisition().finalizeAndPublish();
    
    outermost_context().masterSync().setStateReached();
    
    // sleep for some time here
    ros::Duration(sleep_time).sleep();

    post_event( EvStopTraveling() );
}


// missed goal
GoalMissed::GoalMissed(my_context ctx) : my_base(ctx)
{
    // Increase number of goal misses
    ++outermost_context().commonData().goal_misses_in_a_row;
        
    // if misses in a row is greater than specified treshold, switch to error state
    if (outermost_context().commonData().goal_misses_in_a_row >= context<Traveling>()._max_goal_misses_in_a_row)
    {
            ROS_ERROR_STREAM("Could not reach goal in a row of " << outermost_context().commonData().goal_misses_in_a_row << " actions. Cancelling...");
            
            outermost_context().commonData().navigation_cancled = true;
            outermost_context().dataAcquisition().finalizeAndPublish();
            post_event(EvGoalMissedFinally());
            return;
    }
    
    // sleep for some time here
    int sleep_time = std::trunc(context<Traveling>()._nav_error_idle_time);
    ROS_INFO_STREAM("Goal missed - waiting for " << sleep_time << "s.");
    ros::Duration(sleep_time).sleep();
    
    // clear costmap if it is the second try
    if(outermost_context().commonData().goal_misses_in_a_row > 1)
    {
        context<Traveling>().clearCostmap();
    }
        
    // Otherwise try again
//     ROS_INFO_STREAM("Trying to reach previous goal again...");
//     post_event( EvTravelToGoal( outermost_context().commonData().current_nav_goal ) ); 
    
    post_event( EvStopTraveling() );
}




// select nearest docking station, if there are different stations declared
SelectNextDockingStation::SelectNextDockingStation(my_context ctx) : my_base(ctx)
{
    ROS_INFO("Searching nearest docking station...");
    
    Traveling& traveling = context<Traveling>();
    
    // get robot pose
    geometry_msgs::Pose robot_pose;
    outermost_context().getRobotPose(robot_pose);
    
    double dist = std::numeric_limits<double>::max();
    std::string nearest_station;
    for (const auto& pair_elem : traveling._docking_goals)
    {
        double cur_dist = std::sqrt(std::pow(pair_elem.second.position.x-robot_pose.position.x,2) + std::pow(pair_elem.second.position.y-robot_pose.position.y,2));
        if (cur_dist < dist)
	{
	    dist = cur_dist;
            nearest_station = pair_elem.first;
	}
    }
    
    if (nearest_station.empty())
    {
        ROS_ERROR("No docking station found.");
        post_event(EvDockingStationMissedFinally());
        return;
    }
    
    ROS_INFO_STREAM("Targeting nearest docking station: " << nearest_station);
    post_event(EvTravelToDockingStation(nearest_station));    
}


// send docking goal to the move base and wait during the turtlebot is traveling
TravelToDockingStation::TravelToDockingStation(my_context ctx) : my_base(ctx)
{
    // get current goal
    std::string current_docking = outermost_context().commonData().current_docking_goal;
    
    try
    {
        // check if goal is known, otherwise it throws an exception
        const geometry_msgs::Pose& docking_goal = context<Traveling>().getDockingGoal(current_docking);
        
        // send goal
        ROS_INFO_STREAM("Sending docking goal: " << current_docking << " (current battery status: " << outermost_context().getBatteryPercentage() << "%)");
        outermost_context().commonData().traveling = true;
	if( context<Traveling>().moveBaseToGoal(docking_goal) )
        {
	    outermost_context().commonData().traveling = false;
            ROS_INFO_STREAM("Hooray, we arrived at docking station: " << current_docking);
            post_event(EvDockingStationReached());
            return;
        }
        else
	    //outermost_context()._docking = false;
            ROS_INFO_STREAM("The base failed to move to docking station: " << current_docking);
  
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR_STREAM("Current docking station " << current_docking << " not found. Canceling navigation ...");
    }
    
    post_event(EvDockingStationMissed()); // in case of an error
}

 


  

