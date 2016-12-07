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

#include <turtle_commander/data_acquisition.h>
#include <turtle_commander/turtle_commander.h>
#include <array>

void DataAcquisition::initialize(TurtleCommander& turtle_commander)
{    
    _turtle_commander = &turtle_commander;
    
     _pub_evaluation_data = _nhandle.advertise<tcm::EvaluationData>("/turtle_commander_master/evaluation_data", 5);

    _initialized = true;
}


void DataAcquisition::finalizeData()
{
    if (!_initialized)
        return;
    
    
  int number_v_bins = settings().number_v_bins;
  int number_omega_bins = settings().number_omega_bins;
  
  if (number_v_bins<=0 || number_omega_bins<=0)
  {
    ROS_ERROR("TurtleCommander Master Data Acquisition: 'number_v_bins' and 'number_omega_bins' must be strictly positive.");
    return;
  }
  
  float v_max = settings().v_max;
  float step_v = (v_max*2)/number_v_bins;
  
  std::vector<int> bins_v(number_v_bins, 0);   // initialize bins with zero
  
  std::vector<float> limits_v;	
  for(int i=0; i < (number_v_bins-1); ++i)
    limits_v.push_back( -v_max + step_v * (i+1) );      // set limits for the bins
  
  ROS_INFO_STREAM("Finalizing acquired data...");
  
  for (const auto& vector : data().temp_v)
  {
    // fill the bins of the linear velocity
    double temp_v;
    temp_v = vector.x;
    
    for(int i=0; i<(number_v_bins-1); i++)
    {      
      if(temp_v < limits_v[i])
      {
	bins_v[i]++;
	break;
      }
    }
    if(temp_v >= limits_v[number_v_bins-2]) 
        bins_v[number_v_bins-1]++;	// if it is bigger than the highest limit    
  }
  
  float omega_max = settings().omega_max;
  float step_omega = (omega_max*2)/number_omega_bins;
  
  std::vector<int> bins_omega(number_omega_bins, 0);  // initialize bins with zero

  std::vector<float> limits_omega;
  for(int i=0; i < (number_omega_bins-1); i++)
    limits_omega.push_back( -omega_max + step_omega * (i+1) );  // set limits for the bins
  
  for (const auto& vector : data().temp_omega)
  {      
    // fill bins of the angular velocity
    double temp_omega;
    temp_omega = vector.z;
    
    for(int i=0; i<(number_omega_bins-1); i++)
    {
      if(temp_omega < limits_omega[i])
      {
	bins_omega[i]++;
	break;
      }
    }
    if(temp_omega > limits_omega[number_omega_bins-2]) 
        bins_omega[number_omega_bins-1]++;	// if it is bigger than the highest limit  
  }  


    data().ev_data.v_bins = bins_v;
    data().ev_data.omega_bins = bins_omega;
}


void DataAcquisition::resetData()
{
   if (!_initialized)
       return;
    
   // reset evaluation data // TODO: CHECK IF CORRECT and even better -> get rid of turtlebot commonData() here. Extend the interal data object
    
  data().ev_data.v_bins.clear();
  data().ev_data.omega_bins.clear();
    
  _turtle_commander->commonData().goal_misses_in_a_row = 0;
  data().goal_distance = 0;
  data().temp_v.clear();
  data().temp_omega.clear();
  _turtle_commander->commonData().number_collisions = 0;  
  _turtle_commander->dataAcquisition().data().travel_duration = ros::Duration(0);
  _turtle_commander->commonData().start_location = _turtle_commander->commonData().current_nav_goal;
  ROS_INFO("Acquired data resetted!");   
}


void DataAcquisition::publishData()
{  
    
  ROS_INFO_STREAM("Publishing acquired data...");
  
  // TODO: a publish method should not write to member variables -> make const create new ev_data object
  data().ev_data.turtlebot_number = _turtle_commander->commonData().turtlebot_number;
  data().ev_data.navigation_cancled = _turtle_commander->commonData().navigation_cancled;
  data().ev_data.nav_planner = _turtle_commander->commonData().local_planner;
  data().ev_data.start = _turtle_commander->commonData().start_location;
  data().ev_data.goal = _turtle_commander->commonData().current_nav_goal;
  data().ev_data.number_of_collisions = _turtle_commander->commonData().number_collisions;
  data().ev_data.number_of_resets = _turtle_commander->commonData().goal_misses_in_a_row;
  data().ev_data.distance = data().goal_distance;
  data().ev_data.driving_time = data().travel_duration.toSec();
  data().ev_data.header.stamp = ros::Time::now();
  
  _pub_evaluation_data.publish(data().ev_data);
}


void DataAcquisition::finalizeAndPublish()
{
    finalizeData();
    publishData();
    resetData();
}
