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

#ifndef DATA_ACQUISITION_
#define DATA_ACQUISITION_

#include <ros/ros.h>
#include <turtle_commander_messages/EvaluationData.h>
#include <geometry_msgs/Vector3.h>

namespace tcm = turtle_commander_messages;

struct TurtleCommander;

// TODO: gather important data not in the CommonData class but in this class!
// and hence get rid of the TurtleCommander pointer
class DataAcquisition
{  
public:
    DataAcquisition() {};
      
    struct Settings
    {
      int number_v_bins = 10;
      float v_max = 0.3;
      int number_omega_bins = 10;
      float omega_max = 0.5;      
    };
    
    struct Data
    {
      double goal_distance;
      ros::Duration travel_duration;
      
      tcm::EvaluationData ev_data;
      std::list<geometry_msgs::Vector3> temp_v;
      std::list<geometry_msgs::Vector3> temp_omega;    
    };
    
    
    /** @name Common Methods **/
    ///@{
    
    void initialize(TurtleCommander& turtle_commander);
    
   /**
    * publish a acquisition Data message for the master
    * at the topic "turtle_commander_master/evaluationData"
    */         
    void finalizeAndPublish();
        
    ///@}
    
    
    /** @name Public Data Accessors */
    ///@{ 


    /**
    * get access to the _acquisition_settings of this turtlebot
    * 
    * @return	DataAcquisition::AcquisitionSettings	actual _acquisition_settings of this turtlebot
    */
    Settings& settings() {return _settings;}    
    
    /**
    * get access to the _acquisition_data of this turtlebot
    * 
    * @return	DataAcquisition::AcquisitionData 	actual _acquisition_settings of this turtlebot
    */
    Data& data() {return _data;}  
    
    ///@}
       
   
protected:
       
    void finalizeData();
    void resetData();
    void publishData();
    
private:
    ros::NodeHandle _nhandle = ros::NodeHandle("~");		// handle to the turtle_commander_client node
    Settings _settings;						// settings of this turtlebot
    Data _data;							// data of this turtlebot
    
    bool _initialized = false;
    
        
    ros::Publisher _pub_evaluation_data;                        //!< publisher to send evaluation data to the master
    
    TurtleCommander* _turtle_commander = nullptr;               // TODO get rid of this pointer
    
};




#endif // DATA_ACQUISITION_
