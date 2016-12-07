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

#include <turtle_commander/states_docking.h>
#include <turtle_commander/states_traveling.h>
#include <turtle_commander/substates_traveling.h>

AutoDocking::AutoDocking(my_context ctx) : my_base(ctx)
{
    
    ROS_INFO("Docking started");
 
    std::string autodocking_action = "/dock_drive_action";
    _nhandle.param("autodocking_action", autodocking_action, autodocking_action);
    
    _autodocking_client = std::unique_ptr<AutoDockingClient>(new AutoDockingClient(autodocking_action, true)); // threaded spinner
    
    bool server_online = false;
    for (int i=0; i<5; ++i)
    {
        server_online = _autodocking_client->waitForServer(ros::Duration(1.0)); // wait some seconds and try again
        if (server_online)
        break;
        else 
        ROS_WARN_STREAM("Autodocking action server not found (try " << i+1 << "/5)");
    }
    
    if (!server_online)
    {
        ROS_ERROR_STREAM("Autodocking action server '" << autodocking_action << "' not found. Leaving docking state...");
        post_event(EvAutoDockingFailedFinally());
        return;
    }
    
    // parse parameters
    _nhandle.param("autodocking_timeout", _autodocking_timeout, _autodocking_timeout);
    
    // start docking
    // send action goal
    ROS_INFO_STREAM("Starting auto docking action..." );
    kobuki_msgs::AutoDockingGoal goal_msg;
    
    _autodocking_client->cancelAllGoals();
    ros::Duration(1).sleep();
             
    _autodocking_client->sendGoal(goal_msg);
    
    // we know block the state machine until we achieve the result (another approach would be to use the goal callback)
    if (!_autodocking_client->waitForResult( ros::Duration(_autodocking_timeout) ) )
    {
        _autodocking_client->cancelAllGoals();
        ROS_INFO_STREAM("Autodocking failed within " << _autodocking_timeout << "s (abort due to timeout).");
        post_event(EvAutoDockingFailed());
        return;
    }
            
    if(_autodocking_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        post_event(EvAutoDockingSuccessful());
        return;
    }
    
    post_event(EvAutoDockingFailed());
}


AutoDocking::~AutoDocking()
{
    if (_autodocking_client)
    {
        _autodocking_client->cancelAllGoals();
    }
}

Charging::Charging(my_context ctx) : my_base(ctx)
{
    outermost_context().commonData().autodocking_misses_in_a_row = 0;

    outermost_context().commonData().goal_misses_in_a_row = 0;
    
    int current_percentage = outermost_context().getBatteryPercentage();
    int battery_check_count = 0;
    int battery_check_full_count = 0;
    const int battery_check_max_count = 15; // check after X loops if the battery value has changed
    const int battery_check_full_max_count = 2; // battery check must return full==true X times in a row 
    
    ROS_INFO_STREAM("Charging... waiting until battery is charged up to " << outermost_context().commonData().battery_full_percentage << "%.");
    // check battery state every X minutes
    while (ros::ok())
    {
        ROS_INFO_STREAM("Charging... Current battery level is " << outermost_context().getBatteryPercentage() << "%.");
        if (outermost_context().isBatteryFull())
        {
	  if (battery_check_full_count >= battery_check_full_max_count)
	  {
	    ROS_INFO("Battery full... continue with random traveling. TODO (use history for continuing previous work!)"); // TODO
	    battery_check_full_count = 0;
	    break;
	  }
	  ++battery_check_full_count; 
        }
              
        if (++battery_check_count >= battery_check_max_count)
        {
            int new_percentage = outermost_context().getBatteryPercentage();
            if (new_percentage-current_percentage < 2) // check if it has changed by at least 2 since battery value oscillates sometimes
            {
                ROS_WARN("The battery value has not been increased for quite some time now. Is it really charging?");
                outermost_context().sendMail("Am I charging?", "The battery value has not been increased for quite some time now. Is it really charging?");
            }
        }
        
        ros::Duration(3*60).sleep(); // every 3 minutes
        ros::spinOnce();
    }
    post_event(EvChargingCompleted());
}


AutoDockingFailed::AutoDockingFailed(my_context ctx) : my_base(ctx)
{
    // Increase number of goal misses
    ++outermost_context().commonData().autodocking_misses_in_a_row;
  
    // if misses in a row is greater than specified treshold, switch to error state
    if (outermost_context().commonData().autodocking_misses_in_a_row >= outermost_context().commonData().max_autodocking_misses_in_a_row)
    {
        ROS_ERROR_STREAM("Autodocking failed " << outermost_context().commonData().autodocking_misses_in_a_row << " times in a row. Cancelling...");
        post_event(EvAutoDockingFailedFinally());  // TODO select other docking station
        return;
    }
    
    // Otherwise try again
    ROS_INFO_STREAM("Travel to docking station again.");
    post_event( EvTravelToNextDockingStation() ); // but go into position first    
}

sc::result AutoDockingFailed::react(const EvTravelToNextDockingStation& ev)
{
    if(outermost_context().commonData().random_mode)
      return transit<SelectNextDockingStation>();
    if(outermost_context().commonData().remote_mode)
      return transit<TravelToDockingStation>();
}

// BackupFromDockingStation::BackupFromDockingStation(my_context ctx) : my_base(ctx)
// {
//     // TODO implemantation (driving backwards, continue work etc ...)
//     post_event(EvNothingToDo()); 
// }