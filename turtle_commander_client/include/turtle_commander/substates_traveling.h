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

#ifndef SUBSTATES_TRAVELING_H_
#define SUBSTATES_TRAVELING_H_

#include <turtle_commander/states_traveling.h>


/**
 * class declaration for the sub-states of traveling
 */

/**
 * state to travel to a navigation goal
 */
struct TravelToNavGoal : sc::state<TravelToNavGoal, Traveling>
{
    TravelToNavGoal(my_context ctx);
};

/**
 * state to select a new random goal out of the list
 * of possible goals. It must be different to the acutal one
 */
struct SelectRandomGoal : sc::state<SelectRandomGoal, Traveling>
{
    SelectRandomGoal(my_context ctx);
};

/**
 * state if we reached successful at a navigation goal
 */
struct GoalReached : sc::state<GoalReached, Traveling> 
{    
    GoalReached(my_context ctx);
};

/**
 * state if the traveling to a navigation goal wasn't successful
 * try again and reset global costmap at the second miss
 */
struct GoalMissed : sc::state<GoalMissed, Traveling> 
{    
    GoalMissed(my_context ctx);
};

/**
 * state to select the next docking station in the near of the turtlebot
 */
struct SelectNextDockingStation : sc::state<SelectNextDockingStation, Traveling>
{
    SelectNextDockingStation(my_context ctx);
};


/**
 * state to travel to the docking station
 */
struct TravelToDockingStation : sc::state<TravelToDockingStation, Traveling>
{
    TravelToDockingStation(my_context ctx);
};



#endif // TRAVELING_H_
