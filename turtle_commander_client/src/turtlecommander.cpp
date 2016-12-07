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

#include <turtle_commander/turtle_commander.h>
#include <turtle_commander/states_idle.h>
#include <turtle_commander/mail.h>
#include <string>
#include <chrono>
#include <ctime>

TurtleCommander::TurtleCommander()
{   
   _robot_pose.position.x = _robot_pose.position.y = _robot_pose.position.z = 0;
   _robot_pose.orientation.w = _robot_pose.orientation.x = _robot_pose.orientation.y = _robot_pose.orientation.z = 0;
   
   // get turtlebot number to identificate
   _nhandle.param("turtlebot_number", _common_data.turtlebot_number, _common_data.turtlebot_number);
   
   // create subscriber for odom topic
   std::string odom_frame = "odom";
   _nhandle.param("odom_frame", odom_frame, odom_frame);
   odom_frame = "/robot_" + std::to_string(_common_data.turtlebot_number) + "/" + odom_frame;				// needed for the namespace declaration!
   _odom_sub = _nhandle.subscribe(odom_frame, 10, &TurtleCommander::odomCallback, this);
   
   // get more parameters
   _nhandle.param("max_autodocking_misses_in_a_row", _common_data.max_autodocking_misses_in_a_row, _common_data.max_autodocking_misses_in_a_row);
   
   // get navigation mode
   _nhandle.param("random_mode", _common_data.random_mode, _common_data.random_mode);
   _nhandle.param("remote_mode", _common_data.remote_mode, _common_data.remote_mode);   
   _nhandle.param("voice_mode", _common_data.voice_mode, _common_data.voice_mode);  
   _nhandle.param("simulation_mode", _common_data.simulation, _common_data.simulation);
   
   // get turtlebot number, local planner and initial_location to identificate
   _nhandle.param("turtlebot_number", _common_data.turtlebot_number, _common_data.turtlebot_number);
   _nhandle.param("local_planner", _common_data.local_planner, _common_data.local_planner);
   _nhandle.param("initial_location", _common_data.start_location, _common_data.start_location);
   
   // get battery data
   _nhandle.param("max_battery_value", _common_data.max_battery_value, _common_data.max_battery_value);
   _nhandle.param("min_battery_value", _common_data.min_battery_value, _common_data.min_battery_value);
   _nhandle.param("battery_low_percentage", _common_data.battery_low_percentage, _common_data.battery_low_percentage);
   _nhandle.param("battery_full_percentage", _common_data.battery_full_percentage, _common_data.battery_full_percentage);
   
   std::string kobuki_sensors_topic = "mobile_base/sensors/core";
   _nhandle.param("kobuki_sensors_topic", kobuki_sensors_topic, kobuki_sensors_topic);
   kobuki_sensors_topic = "/robot_" + std::to_string(_common_data.turtlebot_number) + "/" + kobuki_sensors_topic;			// needed for the namespace declaration!
   _battery_sub = _nhandle.subscribe(kobuki_sensors_topic, 2, &TurtleCommander::kobukiSensorsCallback, this);
   
   // get button data
   std::string kobuki_buttons_topic = "mobile_base/events/button";
   _nhandle.param("kobuki_buttons_topic", kobuki_buttons_topic, kobuki_buttons_topic);
   kobuki_buttons_topic = "/robot_" + std::to_string(_common_data.turtlebot_number) + "/" + kobuki_buttons_topic;			// needed for the namespace declaration!
   _button_sub = _nhandle.subscribe(kobuki_buttons_topic, 10, &TurtleCommander::kobukiButtonsCallback, this);
   
   //get DataAcquisition Settings   
   _nhandle.param("number_v_bins", _data_acquisition.settings().number_v_bins, _data_acquisition.settings().number_v_bins);
   _nhandle.param("number_omega_bins", _data_acquisition.settings().number_omega_bins, _data_acquisition.settings().number_omega_bins);
   _nhandle.param("v_max", _data_acquisition.settings().v_max, _data_acquisition.settings().v_max);
   _nhandle.param("omega_max", _data_acquisition.settings().omega_max, _data_acquisition.settings().omega_max);
   
   // initialize bumper detection time
   commonData().time_of_collision = ros::Time::now();
   commonData().traveling = false;
   _data_acquisition.data().goal_distance = 0;
   
   // initialize master sync object
   _master_sync.initialize(_common_data.turtlebot_number, *this);
   
  // initialize voice_command 
  _voice_command.Initialize(_common_data.turtlebot_number, *this);


   // initialize data acquistion object
   _data_acquisition.initialize(*this);
   
   if (_common_data.simulation)
       ROS_INFO("TurtleCommander started in simulation mode.");
   else
       ROS_INFO("TurtleCommander started.");
}


void TurtleCommander::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    ROS_INFO_ONCE("odom callback received. This message will be printed only once.");
    std::lock_guard<std::mutex> l(_odom_mutex);
    if(commonData().traveling)
    {
        double temp_x = _robot_pose.position.x - msg->pose.pose.position.x;
        double temp_y = _robot_pose.position.y - msg->pose.pose.position.y;
        _data_acquisition.data().goal_distance += sqrt(pow(temp_x,2)+pow(temp_y,2));

        _data_acquisition.data().temp_v.push_back(msg->twist.twist.linear);
        _data_acquisition.data().temp_omega.push_back(msg->twist.twist.angular);
    }
    _robot_pose = msg->pose.pose;
}

void TurtleCommander::kobukiSensorsCallback(const kobuki_msgs::SensorStateConstPtr& msg)
{
    ROS_INFO_ONCE("Kobuki sensors callback received. This message will be printed only once.");
    std::lock_guard<std::mutex> l(_kobuki_sensor_mutex);
    _current_battery = msg->battery;
    if(msg->bumper != 0 && (msg->header.stamp.toSec() - commonData().time_of_collision.toSec()) > commonData().debounce_time_bumper)
    {
      commonData().number_collisions++;
      commonData().time_of_collision = msg->header.stamp;
    }
}

void TurtleCommander::kobukiButtonsCallback(const kobuki_msgs::ButtonEventConstPtr& msg)
{
    ROS_INFO_ONCE("Kobuki button pressed...");
    std::lock_guard<std::mutex> l(_kobuki_button_mutex);
    if (msg->button == kobuki_msgs::ButtonEvent::Button0)
        std::get<0>(_buttons_state) = true;
    else if (msg->button == kobuki_msgs::ButtonEvent::Button1)
        std::get<1>(_buttons_state) = true;
    else if (msg->button == kobuki_msgs::ButtonEvent::Button2)
        std::get<2>(_buttons_state) = true;
}

bool TurtleCommander::isButtonPressed(uint8_t button_number)
{
    std::lock_guard<std::mutex> l(_kobuki_button_mutex);
    if (button_number == kobuki_msgs::ButtonEvent::Button0)
        return std::get<0>(_buttons_state);
    else if (button_number == kobuki_msgs::ButtonEvent::Button1)
        return std::get<1>(_buttons_state);
    else if (button_number == kobuki_msgs::ButtonEvent::Button2)
        return std::get<2>(_buttons_state);
    else
        ROS_ERROR("Unknown button number. Only 0,1,2 are possible");
}
void TurtleCommander::resetButtonState(uint8_t button_number)
{
    std::lock_guard<std::mutex> l(_kobuki_button_mutex);
    if (button_number == kobuki_msgs::ButtonEvent::Button0)
        std::get<0>(_buttons_state) = false;
    else if (button_number == kobuki_msgs::ButtonEvent::Button1)
        std::get<1>(_buttons_state) = false;
    else if (button_number == kobuki_msgs::ButtonEvent::Button2)
        std::get<2>(_buttons_state) = false;
    else
        ROS_ERROR("Unknown button number. Only 0,1,2 are possible");
}

void TurtleCommander::getRobotPose(geometry_msgs::Pose& pose_out)
{
    std::lock_guard<std::mutex> l(_odom_mutex);
    pose_out = _robot_pose;
}


int TurtleCommander::getBatteryPercentage()
{
    std::lock_guard<std::mutex> l(_kobuki_sensor_mutex);
    return int((_current_battery-_common_data.min_battery_value)/(_common_data.max_battery_value-_common_data.min_battery_value) * 100.0);
}

bool TurtleCommander::isBatteryLow()
{
     return getBatteryPercentage() <= _common_data.battery_low_percentage;
}

bool TurtleCommander::isBatteryFull()
{
    return getBatteryPercentage() >= _common_data.battery_full_percentage;
}

bool TurtleCommander::isRandomModeActive()
{
  return _common_data.random_mode;
}

bool TurtleCommander::isRemoteModeActive()
{
  return _common_data.remote_mode;
}

bool TurtleCommander::isVoiceModeActive()
{
  return _common_data.voice_mode;
}



bool TurtleCommander::requestDockingEvent()
{
  // decide if the turtlebot should wait for a docking goal from the master
  // or looking itself for a goal
  if(outermost_context().commonData().random_mode)
  {
      post_event( EvTravelToNextDockingStation() );
      return true;
  } 
  
  if(outermost_context().commonData().remote_mode)
  {
      outermost_context().commonData().navigation_cancled = false;
      ROS_INFO("Requesting docking station from master...");
      boost::optional<std::string> docking_goal = outermost_context().masterSync().requestDocking(10); // TODO param -> timeout
      if (docking_goal)
      {
        ROS_INFO_STREAM("Received docking station name '" << *docking_goal << "'.");
        post_event( EvTravelToDockingStation(*docking_goal) ); 
        return true;
      }
  }
  return false;
}

void TurtleCommander::requestTravelingStop(bool stop)
{
        commonData().traveling_stop_requested = stop;
}

bool TurtleCommander::isAllowedToDrive()
{
    // check if we are in the allowed timeframe
    if (!commonData().simulation && !isWithinDrivingTimeframe()) // do not check in simulation mode
        return false;
    
    // check if stopping is requested explicitly
    if (commonData().traveling_stop_requested)
    {
        ROS_INFO("We are currently not allowed to drive, since a stopping request is active currently");
        return false;
    }
    
    return true;
}


bool TurtleCommander::isWithinDrivingTimeframe()
{
    bool ret_val = false;
    
    using clock = std::chrono::system_clock;
    
    int start_hour = 0;
    int start_min = 0;
    int end_hour = 23;
    int end_min = 59;
    
    _nhandle.param( "allow_driving_start_hour", start_hour, start_hour );
    _nhandle.param( "allow_driving_start_min", start_min, start_min );
    _nhandle.param( "allow_driving_end_hour", end_hour, end_hour );
    _nhandle.param( "allow_driving_end_min", end_min, end_min );
    
    
    // check start and stop time of day (to allow autonomous driving)
    std::time_t now = clock::to_time_t( clock::now() );
    std::tm local_tm = *std::localtime(&now);
    
    // check hours
    if (local_tm.tm_hour >= start_hour && local_tm.tm_hour <= end_hour)
    {
        ret_val = true;
        
        // check minutes in case of same hours
        if (local_tm.tm_hour == start_hour)
        {
            if (local_tm.tm_min >= start_min)
                ret_val = true;
            else 
                ret_val = false;
        }
        
        if (local_tm.tm_hour == end_hour)
        {
            if (ret_val == true && local_tm.tm_min <= end_min)
                ret_val = true;
            else
                ret_val = false; 
        }
    }
    else 
        ret_val = false;
       
    
    return ret_val;
}

bool TurtleCommander::sendMail(const std::string& subject, const std::string& body)
{
    bool enabled = false;
    if (!_nhandle.getParam("turtle_mail_enabled", enabled) || !enabled)
    {
            ROS_INFO("Sending notification via mail disabled.");
            return false;
    }
    
    std::string sender_address;
    if (!_nhandle.getParam("turtle_mail_address", sender_address))
    {
        ROS_ERROR("sendMail(): mail address of the sender not specified, set ros param 'turtle_mail_address'");
        return false;
    }
    
    std::string receiver_address;
    if (!_nhandle.getParam("turtle_mail_receiver", receiver_address))
    {
        ROS_ERROR("sendMail(): mail address of the receiver not specified, set ros param 'turtle_mail_receiver'");
        return false;
    }
    
    std::string smpt_server;
    if (!_nhandle.getParam("turtle_mail_smtp", smpt_server))
    {
        ROS_ERROR("sendMail(): mail address of the receiver not specified, set ros param 'turtle_mail_smtp'");
        return false;
    }
    
    std::string login_name;
    if (!_nhandle.getParam("turtle_mail_login", login_name))
    {
        ROS_ERROR_STREAM("sendMail(): login name not specified for mail account " << sender_address);
        return false;
    }
    
    std::string login_password;
    if (!_nhandle.getParam("turtle_mail_pw", login_password))
    {
        ROS_ERROR_STREAM("sendMail(): login password not specified for mail account " << sender_address);
        return false;
    }
    std::string  name = "Turtlebot_" + std::to_string(_common_data.turtlebot_number);
    if (mail::sendMail(name, subject, body, sender_address, smpt_server, login_name, login_password, receiver_address))
    {
        ROS_INFO_STREAM("Send mail to " << receiver_address);
        return true;
    }
    return false;
}

bool TurtleCommander::sendTelegram(const std::string& text)
{
	bool enabled = false;
	if (!_nhandle.getParam("telegram_enabled", enabled) || !enabled)
	{
		ROS_INFO("Sending notification via telegram disabled.");
		return false;
	}
	
    std::string telegram_token;
    if (!_nhandle.getParam("telegram_token", telegram_token))
    {
        ROS_ERROR("sendTelegram(): cannot find bot token, set ros param 'telegram_token'");
        return false;
    }
    
    std::string telegram_chat_id;
    if (!_nhandle.getParam("telegram_chat_id", telegram_chat_id))
    {
        ROS_ERROR("sendTelegram(): telegram chat id not specified, set ros param 'telegram_chat_id'");
        return false;
    }
    
    mail::sendTelegram(text, telegram_chat_id, telegram_token);
}


Failure::Failure(my_context ctx) : my_base(ctx)
{
    outermost_context().masterSync().setStateFailure();
    ROS_ERROR("Cannot move... PLEASE HELP ME");
    ROS_ERROR("Cannot move... PLEASE HELP");
    ROS_ERROR("Cannot move... PLEASE HELP");
    // TODO notification, recovery, ...., or back to docking station
    
    std::string text = "Turtlebot_" + std::to_string(outermost_context().commonData().turtlebot_number) +": Sorry, but I don't know how to proceed finding the docking station...\r\nMy current battery status is " + std::to_string(outermost_context().getBatteryPercentage()) + "%. Thanks for help! Your Turtlebot 1";
	
    outermost_context().sendTelegram(text);
    outermost_context().sendMail("Cannot do anything", text);

    
    // TODO: what should we do here? go back to IDLE but have some failure flag?
}
