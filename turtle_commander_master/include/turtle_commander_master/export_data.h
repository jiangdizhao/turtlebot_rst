#ifndef EXPORT_DATA_H
#define EXPORT_DATA_H

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <vector>
#include <mutex>
#include <turtle_commander_master/types.h>
#include <geometry_msgs/PoseArray.h>

#ifdef MATIO
	#include <matio.h>
#else
	#include <turtle_commander_master/matio_dummy.h>
#endif


class DataExport
{
public:
  DataExport();
   /**
   * initialize an export object
   * get the setting and the goal list
   * @param 	evaluation_settings 	Setting for the evaluation file (work path, number of bins)
   * @param 	goal_list 		list of the goals ( with their positions, for beeline)
   */
  void init(const EvaluationSettings* evaluation_settings, std::unordered_map<std::string, geometry_msgs::Pose>* goal_list);
     /**
   * save actual data to the "evaluation_data.mat" file
   * it overwrites each time the existing one!
   */
  void exportData();
     /**
   * add new data to export them next time
   * @param	evaluationData		data which should be exported
   */  
  void addData(const turtle_commander_messages::EvaluationData::ConstPtr&  evaluation_data);
     /**
   * declare the shortcuts for each route (takes the goal list) and export them to the
   * "config_routes.txt" file
   */    
  void declareShortcuts();
     /**
   * find the shortcut for a route. A to B and B to A, are the same route!
   * @param	start		start position of the route
   * @param	goal		goal position of the route
   */    
  int findRoute(std::string start,std::string goal);
     /**
   * find the shortcut for a planner. if it is the first time of this planner
   * add it and save the new configuration to the "config_planner.txt" file
   * @param	planner		planner of this route
   */      
  int findPlanner(std::string planner);
  
private:
    // pointer to the "evaluation_data.mat"-file in the work-path!
    mat_t*	matfp;
    // cell array which will be write into the mat file
    matvar_t*	cell_array, *cell_element, *bin_config;
    // dimension vector
    size_t 	dims[2] = {10,1};
    
    std::recursive_mutex _data_mutex;
    
    bool _initialized = false;
    
    // settings of the evaluation (number of bins, work path...)
    const EvaluationSettings* _evaluation_settings;
    // number of datasets
    int number_of_datasets = 0;
    // number of different local planner
    int _number_of_planner = 0;
    // ofstream to the "config_planner.txt" file to save new planner to it
    std::ofstream* _config_file_planner;
    
    // map of the actual goals
    std::unordered_map<std::string, geometry_msgs::Pose>* _goal_list;  
    // map of the routes out of the goals, with their shortcut
    std::unordered_map<std::string, int> _route_map;
    // map of the planner with their shortcuts
    std::unordered_map<std::string, int> _planner_map;
    
    //vecotrs for the dataset
    std::vector<int> turtlebot_number;
    std::vector<uint8_t> navigation_cancled;
    std::vector<int> nav_planner;
    std::vector<int> route;
    std::vector<double> distance;
    std::vector<double> driving_time;
    std::vector<int> number_of_resets;    
    std::vector<int> number_collisions;
    // vector in a vector to enable flexible number of bins
    std::vector<std::vector<int>> v_bins;
    // vector in a vector to enable flexible number of bins    
    std::vector<std::vector<int>> omega_bins;
};


#endif
