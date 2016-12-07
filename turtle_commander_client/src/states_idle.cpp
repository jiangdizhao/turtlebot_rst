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

#include <turtle_commander/states_idle.h>
#include <turtle_commander/states_traveling.h>
#include <turtle_commander/substates_traveling.h>


Idle::Idle(my_context ctx) : my_base(ctx)
{
    // start an idle loop and check sometimes if the robot is allowed to move
    //outermost_context()._docking = false;
    outermost_context().commonData().traveling = false;
    
    bool allowed_drive_flag = true; // we introduce this variable to print the info below just once each time Idle is entered.

    ros::Duration duration( 5 ); // each 5 sec
    while (ros::ok())
    {
                   
        // first register
        if (outermost_context().isRemoteModeActive())
        {
            // registerClient() also returns true if already registered
            if (!outermost_context().masterSync().registerClient(outermost_context().commonData().start_location))
            {
                ros::Duration(5).sleep();        // needed to start up all subscribers
                ROS_INFO("Failed to register client, trying again...");
                continue; // try again
            }   
        }
                
        if (outermost_context().isBatteryLow())
        {
            ROS_WARN_STREAM("Battery is below " << outermost_context().commonData().battery_low_percentage << "%. Canceling navigation and travelling back to next docking station.");
            if (outermost_context().requestDockingEvent())
                return; // new event has been triggered
        }

        // check if we are allowed to drive
        if (outermost_context().isAllowedToDrive())
        {            
            allowed_drive_flag = true;
            
            // check if we have voice commands available
            if (outermost_context().isVoiceModeActive())
            {
                boost::optional<Voice_command::Goal> goal = outermost_context().voice_command().isGoalAvailable();
        
                if(goal)
                {
                        if(!goal->docking)
                            post_event(EvTravelToGoal(goal->name));
                        else
                            post_event(EvTravelToDockingStation(goal->name));
			return;// trigger event
                }
            }
            
            // check for further commands    
            if (outermost_context().isRandomModeActive())
            {
                ROS_INFO("Starting random traveling.");
                post_event(EvRandomTraveling()); 
                return; // trigger event
            }
            else if(outermost_context().isRemoteModeActive())
            {                             
                // check if master has requested goals (nav goals or docking goals)
                boost::optional<MasterSync::Goal> goal = outermost_context().masterSync().isGoalAvailable();  
                
                if (goal)
                {
                    if (goal->docking)
                        post_event( EvTravelToDockingStation(goal->name) );
                    else
                        post_event( EvTravelToGoal(goal->name) );
                    return; // trigger event
                }
            }
        }
        else
        {
            ROS_INFO_COND(allowed_drive_flag, "I'm not allowed to drive. Will check it again later!"); // show this msg once each state transition
            allowed_drive_flag = false;
        }
        duration.sleep();
    }
}

sc::result Idle::react( const EvRandomTraveling& ev )
{
    return transit<SelectRandomGoal>(); // TODO battery guard
}

// reaction and transition for the event EvTravelToGoal
sc::result Idle::react( const EvTravelToGoal& ev )
{
    return transit<TravelToNavGoal>(&TurtleCommander::setNavGoal, ev);
}


// transition for the event EvTravelToDockingStation
sc::result Idle::react( const EvTravelToDockingStation& ev )
{
    return transit<TravelToDockingStation>(&TurtleCommander::setDockingStation, ev);
}

// transition for the event EvTravelToNextDockingStation
sc::result Idle::react( const EvTravelToNextDockingStation& ev )
{
    return transit<SelectNextDockingStation>();
}


