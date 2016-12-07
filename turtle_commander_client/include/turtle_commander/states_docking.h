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

#ifndef DOCKING_H_
#define DOCKING_H_

#include <turtle_commander/turtle_commander.h>
#include <turtle_commander/states_idle.h>

#include <kobuki_msgs/AutoDockingAction.h>
#include <actionlib/client/simple_action_client.h>

#include <unordered_map>
#include <mutex>


struct Charging;
struct AutoDockingFailed;
/**
 * State to control auto docking service 
 * would get skipped in the remote mode
 */
struct AutoDocking : sc::state< AutoDocking, TurtleCommander >
{    
    using reactions = mpl::list< sc::transition<EvAutoDockingSuccessful, Charging>,
                                 sc::transition<EvAutoDockingFailed, AutoDockingFailed>,
                                 sc::transition<EvAutoDockingFailedFinally, Failure> >;
                                
    AutoDocking(my_context ctx);
    ~AutoDocking();
    
private:
    
    ros::NodeHandle _nhandle = ros::NodeHandle("~");
    
    double _autodocking_timeout = 100;
    
    using AutoDockingClient = actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction>;
    std::unique_ptr<AutoDockingClient> _autodocking_client;
};

/**
 * intermediate step for further modification at the turtle commander
 */
// struct BackupFromDockingStation : sc::state< BackupFromDockingStation, TurtleCommander >
// {
//     using reactions = sc::transition<EvNothingToDo, Idle>;
//     BackupFromDockingStation(my_context ctx);
// };

/**
 * state during the charging process, check the battery level periodicly
 */
struct Charging : sc::state< Charging, TurtleCommander >
{
    using reactions = sc::transition<EvChargingCompleted, Idle>;
    Charging(my_context ctx);
};

/**
 * state if the auto docking or the travel to the docking station wasn't 
 * successfull
 */
struct AutoDockingFailed: sc::state< AutoDockingFailed, TurtleCommander >
{
    using reactions = mpl::list< sc::custom_reaction<EvTravelToNextDockingStation>,
                                 sc::transition<EvAutoDockingFailedFinally, Failure>>;
    
    AutoDockingFailed(my_context ctx);
    
    sc::result react(const EvTravelToNextDockingStation& ev);
};




#endif // DOCKING_H_
