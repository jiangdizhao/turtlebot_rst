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

#ifndef TURTLE_COMMANDER_
#define TURTLE_COMMANDER_


#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/ButtonEvent.h>

#include <turtle_commander/types.h>
#include <turtle_commander/events.h>
#include <turtle_commander/data_acquisition.h>
#include <turtle_commander/master_sync.h>
#include <turtle_commander/voice_command.h>
#include <turtle_commander_messages/StateMessage.h>
// State machine
struct Idle; // default state (forward declared)
struct TurtleCommander : sc::state_machine< TurtleCommander, Idle >
{  
public:
    TurtleCommander();
                
    struct CommonData
    {
        int turtlebot_number = 1;                       //!< number of the turtlebot (namespace: robot_X)
        std::string local_planner = "";                 //!< name of the local planner
        
        bool random_mode = false;                       //!< selection bit of random mode
        bool remote_mode = false;                       //!< selection bit of remote mode (get goals from a master)
        bool voice_mode = true;                        //!< selection bit of voice mode
        bool simulation = false;                        //!< seelction bit of simulation mode (skip autodocking, ignore driving times)
        
        double min_battery_value = 140;                 //!< minimum battery voltage [0.1V] to calculate battery level
        double max_battery_value = 150;                 //!< maximum battery voltage [0.1V] to calculate battery level
        int battery_full_percentage = 95;               //!< upper threshold of battery load -> start traveling again
        int battery_low_percentage = 50;                //!< lower thresholf of battery load -> stop traveling, move to docking
        
        std::string current_nav_goal;                   //!< current goal of navigation
        std::string current_docking_goal;               //!< current docking goal
        std::string start_location = "";                //!< start location of this route
        
        int goal_misses_in_a_row = 0;                   //!< number of misses to reach a goal
        int autodocking_misses_in_a_row = 0;            //!< number of misses to finisch docking
        int max_autodocking_misses_in_a_row = 10;       //!< threshold for maximal misses to finisch docking
        
        int maxRegisterTrys = 5;                        //!< threshold for maximal register trys at the master
        int registerTrys = 0;                           //!< number of register trys at the master
        double master_travelsync_timeout = 30;          //!< specify number of seconds how long the client tries to synchronize with the master before leaving traveling mode.
        
        ros::Time time_of_collision;                    //!< time of the last collision, to debounce the bumper
        int debounce_time_bumper = 2;                   //!< debounce time for the bumper sensor
        int number_collisions = 0;                      //!< number of collisions at this route
        bool navigation_cancled = false;                //!< bit if the maximal number of misses a goal in row is reached
        
        bool traveling = false;                         //!< keep track if the robot is currently traveling or not
        bool traveling_stop_requested = false;          //!< true if a stop is requested externally
    };
        
    
    /** @name Public Data Accessors */
    ///@{ 

    /**
    * get access to the _common_data of this turtlebot
    * 
    * @return	CommonData		actual _common_data of this turtlebot
    */
    CommonData& commonData() {return _common_data;}
    
    /**
    * get access to the _data_acquisition object of this turtlebot
    * 
    * @return	DataAcquisition	        actual _data_acquisition object of this turtlebot
    */
    DataAcquisition& dataAcquisition() {return _data_acquisition;}    
    
    /**
     * get access to the master synchronization object
     * 
     * @return MasterSync       actual _master_sync object of this turtlebot
     */
    MasterSync& masterSync() {return _master_sync;}

    Voice_command& voice_command() {return _voice_command;}
   
    
    
    /** @name Common robot Options and Status */
    ///@{ 
    
    /**
    * set the _current_nav_goal out of an event of the type EvTravelToGoal
    * 
    * @param 	ev	 	Event Travel to Goal
    */
    void setNavGoal(const EvTravelToGoal& ev) {_common_data.current_nav_goal = ev.getGoal();}
    
    /**
    * set the _current_docking_goal out of an event of the type EvTravelToDockingStation
    * 
    * @param 	ev	 	Event Travel to Docking Station
    */
    void setDockingStation(const EvTravelToDockingStation& ev) {_common_data.current_docking_goal = ev.getDockingStation();}
    
    /**
    * get the actual pose of the robot (out of the last odom-Callback
    * 
    * @param 	pose_out 	tag to save the actual value
    */
    void getRobotPose(geometry_msgs::Pose& pose_out);
    
    /**
    * get the actual battery percentage. out of the values:
    * _common_data.min_battery_value,
    * _common_data.max_battery_value and
    * the value out of the kobukiSensorsCallback (_current_battery)
    * 
    * @return 	int 		battery load in percent
    */
    int getBatteryPercentage();
    
    /**
    * check if the battery load is below the threshold out 
    * of _common_data.battery_low_percentage
    * 
    * @return 	bool 		true -> battery load is below threshold
    */
    bool isBatteryLow();
    
    /**
    * check if the battery load is abouve the threshold out 
    * of _common_data.battery_full_percentage
    * 
    * @return 	bool 		true -> battery load is above threshold
    */    
    bool isBatteryFull();
    
    /**
    * check if the random mode is activated
    * 
    * @return 	bool 		true -> random mode is activated
    */
    
    bool isRandomModeActive();
    /**
    * check if the remote mode is activated
    * 
    * @return 	bool 		true -> remote mode is activated
    */    
    
    bool isRemoteModeActive();
    
    /**
    * check if the remote mode is activated
    * 
    * @return 	bool 		true -> remote mode is activated
    */    
    
    bool isVoiceModeActive();
    
    /**
    * check if the robot is allowed to drive:
    * check the allowed driving times
    * 
    * if simulation mode is active it returns always true!
    * 
    * @return 	bool 		true -> robot is allowed to drive
    */    
    bool isAllowedToDrive();
        
    /**
    * check if the button at the kobuki base was pressed
    * 
    * @param	button		number of button which should checked
    * 
    * @return 	bool 		true -> this button was pressed
    */ 
    
    bool isButtonPressed(uint8_t button);
    /**
    * reset the state of this button
    * 
    * @param	button		number of button which should resetted
    */     
    void resetButtonState(uint8_t button);
    
    /**
     * Trigger a docking event
     * 
     * This method distinguishes between random and remote mode whehter to 
     * select the closest docking station or to retrieve a docking station from the master.
     * 
     * @return bool     true in case the event has been triggered.
     */
    bool requestDockingEvent();
    
    /**
     * Request a stop if in traveling mode!
     */
    void requestTravelingStop(bool stop);
    
    
    sc::result transitToDocking();
    
    ///@}
    
    
    /** @name Utilitiy Methods */
    ///@{ 

    /**
    * send a mail with the given subject and body
    * 
    * @param	subject		subject text of the sending mail
    * @param	body		body text of the sending mail
    * 
    * @return	bool		true -> mail sent successful
    */     
    bool sendMail(const std::string& subject, const std::string& body);
    
    /**
    * send a telegram messages with the given text
    * 
    * @param	text		text of the sending message
    * 
    * @return	bool		true -> message sent successful
    */         
    bool sendTelegram(const std::string& text);
        

    
    ///@}
    
    
  
protected:
    
    /** @name Subscriber Callbacks */ 
    ///@{ 
    // ### do not forget to use mutexes since the spinner is called in a separate thread ###
    
    /**
    * subsriber callback for the topic
    * "robot_X/odom"
    * to get a new odom message
    * 
    * @param	msg 		odom message
    */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    
    /**
    * subsriber callback for the topic
    * "robot_X/mobile_base/sensors/core" (on default)
    * to get a new sensor callback message from the kobuki base
    * 
    * @param	msg		kobuki sensor message
    */    
    void kobukiSensorsCallback(const kobuki_msgs::SensorStateConstPtr& msg);
    
    /**
    * subsriber callback for the topic
    * "robot_X/mobile_base/events/button" (on default)
    * to get a new button callback message from the kobuki base
    * 
    * @param	msg		kobuki button message
    */        
    void kobukiButtonsCallback(const kobuki_msgs::ButtonEventConstPtr& msg);
    
    ///@}
    
    /** @name Utility methods **/
    ///@{
    
    
    /**
    * check driving timeframe
    * check the allowed driving times
    * 
    * if simulation mode is active it returns always true!
    * 
    * @return bool     true -> robot is allowed to drive w.r.t. current timeframe
    */    
    bool isWithinDrivingTimeframe();
    
    
    ///@}
    
    
    
private:
    ros::NodeHandle _nhandle = ros::NodeHandle("~");            //!< handle to the turtle_commander_client node
    CommonData _common_data;                                    //!< common data of this turtlebot
    DataAcquisition _data_acquisition;                          //!< data acquisition object of this turtlebot
    MasterSync _master_sync;                                    //!< object that regulates the communication with the master
    Voice_command _voice_command;                               //!< voice command object
    ros::Subscriber _odom_sub;                                  //!< subscriber for the odom topic    
    std::mutex _odom_mutex;                                     //!< mutex for the odom callback
    geometry_msgs::Pose _robot_pose;                            //!< store current robot pose (out of the odom topic)
    
    ros::Subscriber _battery_sub;                               //!< Subscriber for the kobuki sensor topic
    std::mutex _kobuki_sensor_mutex;                            //!< mutex for the kobuki sensor callback 
    double _current_battery = 1000;                             //!< extremely high value to start always with a full battery until receiving the first msg...
    
    ros::Subscriber _button_sub;                                //!< subscriber for the kobuki button topic
    std::mutex _kobuki_button_mutex;                            //!< mutex for the kobuki button callback
    std::tuple<bool, bool, bool> _buttons_state;                //!< tuple to store the state of the three button
};


struct Failure : sc::state<Failure, TurtleCommander>
{
    Failure(my_context ctx);
};



#endif // TURTLE_COMMANDER_
