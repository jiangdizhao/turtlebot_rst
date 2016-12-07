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

#ifndef EVENTS_H_
#define EVENTS_H_

#include <turtle_commander/types.h>


// Events
struct EvTravelToGoal : sc::event< EvTravelToGoal > 
{
    EvTravelToGoal(const std::string& goal_name);
    const std::string& getGoal() const {return _goal_name;}
    
private:
  std::string _goal_name;
};

struct EvTravelToDockingStation : sc::event< EvTravelToDockingStation > 
{
    EvTravelToDockingStation(const std::string& goal_name);
    const std::string& getDockingStation() const {return _goal_name;}
    
private:
  std::string _goal_name;
};

struct EvGoalReached : sc::event< EvGoalReached > {};
struct EvGoalMissed : sc::event< EvGoalMissed > {};
struct EvGoalMissedFinally : sc::event< EvGoalMissedFinally > {};
struct EvStopTraveling : sc::event< EvStopTraveling > {};
struct EvRandomTraveling : sc::event< EvRandomTraveling > {};
struct EvWaitToGetDockingStation : sc::event< EvWaitToGetDockingStation > {};
struct EvTravelToNextDockingStation : sc::event< EvTravelToNextDockingStation > {};
struct EvDockingStationReached : sc::event< EvDockingStationReached > {};
struct EvDockingStationMissed : sc::event< EvDockingStationMissed > {};
struct EvDockingStationMissedFinally : sc::event< EvDockingStationMissedFinally > {};

struct EvAutoDockingSuccessful : sc::event< EvAutoDockingSuccessful > {};
struct EvAutoDockingFailed : sc::event< EvAutoDockingFailed > {};
struct EvAutoDockingFailedFinally : sc::event< EvAutoDockingFailedFinally > {};
struct EvChargingCompleted : sc::event< EvChargingCompleted > {};

//struct EvNothingToDo : sc::event< EvNothingToDo > {};

#endif // EVENTS_H_
