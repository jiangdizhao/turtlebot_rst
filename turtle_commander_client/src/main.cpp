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
#include <turtle_commander/events.h>
#include <turtle_commander/states_idle.h>

#include <turtle_commander/mail.h>

#include <signal.h> // custom sig-handler

void slowRosSpinner();


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_commander");

    ros::NodeHandle n("~");
    
    // create our own asynchronous ros spinner with a slow rate
    std::thread ros_spinner(slowRosSpinner);
    
    TurtleCommander commander;

    commander.initiate(); // start with initial state
    
    //commander.process_event(EvRandomTraveling()); // EvRandomTraveling blocks the complete statemachine until the user aborts or some error state is reached!
    //commander.process_event(EvTravelToNextDockingStation());

    ros_spinner.join();
    
    return 0;
}





void slowRosSpinner()
{
    while(ros::ok())
    {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}



