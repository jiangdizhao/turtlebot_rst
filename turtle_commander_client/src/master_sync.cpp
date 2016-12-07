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


#include <turtle_commander/master_sync.h>
#include <turtle_commander/types.h>
#include <turtle_commander/events.h>
#include <turtle_commander/turtle_commander.h>
#include <ros/subscribe_options.h>
#include <boost/bind.hpp>


void MasterSync::initialize(int turtlebot_no, TurtleCommander& turtle_commander)
{    
    _turtlebot_no = turtlebot_no;
    
    _turtle_commander = &turtle_commander;
    
    
    // initialize publisher to publish State Messages
    _cmd_client = _nhandle.serviceClient<tcm::CommandService>("/turtle_commander_master/cmd_to_master"); // TODO param
    _cmd_srv = _nhandle.advertiseService("cmd_from_master",  &MasterSync::commandServiceCallback, this);
        
    _initialized = true;
}

bool MasterSync::registerClient(const std::string& start_location)
{
    if (!_initialized)
        throw std::runtime_error("MasterSync class is not initialized, call initialize first.");
    
    if (_registered)
        return true;
    
    
    // initialize master bond
    // we initialize our bond here instead of in initialize() since we want to create a new bond 
    // whenever the master restarts. If we would not create a new bond, the old process UUID is cached
    // (unfortunately there is no reset method in bondcpp).
    _bond = std::unique_ptr<bond::Bond>(new bond::Bond("/turtle_commander_master/client_bond",
                                                       std::to_string(_turtlebot_no),
                                                       boost::bind(&MasterSync::onClientBoundBrokenCallback, this),
                                                       boost::bind(&MasterSync::onClientBoundFormedCallback, this) 
                                                       ) );
    _bond->setHeartbeatPeriod(4);
    _bond->setHeartbeatTimeout(10);
    
    // start bond to connect with other master process
    _bond->start();
    
    
    ROS_INFO("Registering turtlebot %d. Waiting for response from master...", _turtlebot_no);
    
    _registered = sendMasterCommand("REGISTER", start_location, false);
    
    ROS_INFO_COND(_registered, "Master registered me successfully! Waiting for further instructions...");
    
    return _registered;
}


bool MasterSync::setStateFailure()
{
    return sendMasterCommand("STUCK"); 
}

bool MasterSync::setStateReached()
{
    return sendMasterCommand("REACHED");  
}

bool MasterSync::setStateDocking()
{
    return sendMasterCommand("DOCKING");
}

bool MasterSync::sendMasterCommand(const std::string& command, const std::string& argument, bool req_registered)
{
    if (req_registered && !_registered)
        return false;
    
    tcm::CommandService srv;
    srv.request.turtlebot_number = _turtlebot_no;
    srv.request.command = command;
    srv.request.argument = argument;
    _cmd_client.call(srv);
    return srv.response.success;
};


bool MasterSync::commandServiceCallback(tcm::CommandService::Request& req,
                            tcm::CommandService::Response& res)
{
    if(req.turtlebot_number == _turtlebot_no)
    {
        if(req.command == "GOAL")
        {      
            ROS_INFO_STREAM("Goal '" << req.argument << "' received from master.");
            std::lock_guard<std::mutex> l(_goal_mutex);
            _goal.name = req.argument;
            _goal.docking = false;
            _new_goal_flag = true;
            res.success = true;
            return true;   
        }
        else if(req.command == "DOCKING_GOAL")
        {
            ROS_INFO_STREAM("Docking station '" << req.argument << "' received from master.");
            std::lock_guard<std::mutex> l(_goal_mutex);
            _goal.name = req.argument;
            _goal.docking = true;
            _new_goal_flag = true;
            res.success = true;
            return true;
        }
    }
    
    
    return true;    
}


boost::optional<MasterSync::Goal> MasterSync::isGoalAvailable()
{
    std::lock_guard<std::mutex> l(_goal_mutex);
    
    if (!_new_goal_flag)
        return boost::none;
    
    // reset new goal flag
    _new_goal_flag = false;
    
    return _goal;
}

boost::optional<std::string> MasterSync::requestDocking(double timeout_sec)
{
    setNewGoalFlag(false);
    bool req_flag = setStateDocking();
    if (!req_flag)
    {
            ROS_WARN("requestDockingGoal(): cannot send dockign request to master via service client");
            return boost::none;
    }
    ros::Duration d(1);
    ros::Time timer_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now()-timer_start).toSec() < timeout_sec)
    {
          ros::spinOnce(); // force triggering callbacks even if we are running a second async spinner
          // check if master has requested goals (nav goals or docking goals)
          boost::optional<MasterSync::Goal> goal = isGoalAvailable();
          if (goal && goal->docking)
              return goal->name;
          d.sleep();
    }
    ROS_WARN("requestDockingGoal(): Timeout -> no response from master.");
    return boost::none;
}


void MasterSync::onClientBoundFormedCallback()
{
    ROS_INFO("Bond to master established.");
}

void MasterSync::onClientBoundBrokenCallback()
{
   _registered = false;
   ROS_INFO("Bond to master broken."); 
}
