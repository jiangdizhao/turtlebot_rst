
#include <turtle_commander_master/export_data.h>

#ifdef MATIO
  DataExport::DataExport(){ }
  
  void DataExport::init(const EvaluationSettings* evaluation_settings, std::unordered_map<std::string, geometry_msgs::Pose>* goal_list)
  {
      _evaluation_settings = evaluation_settings;    
      _goal_list = goal_list;
      
      // Create Mat File
      matfp = Mat_CreateVer(_evaluation_settings->evaluation_file.c_str(),NULL,MAT_FT_DEFAULT);
      if ( NULL == matfp ) {
	ROS_ERROR_STREAM("Error creating MAT file" << _evaluation_settings->evaluation_file);
      }
      Mat_Close(matfp);
      
      _config_file_planner = new std::ofstream(_evaluation_settings->configuration_file_planner);
      std::string text_to_write = "number \t planner \n";
      const char* line_to_write = text_to_write.c_str();
      _config_file_planner->write(line_to_write,text_to_write.length());
      _config_file_planner->close();
      declareShortcuts();
      
      _initialized = true;
  }

    
  void DataExport::addData(const turtle_commander_messages::EvaluationData::ConstPtr&  evaluation_data)
  {
      if (!_initialized)
          return;
      
      std::lock_guard<std::recursive_mutex> l(_data_mutex);
      
      number_of_datasets++;
      turtlebot_number.push_back(evaluation_data->turtlebot_number);
      navigation_cancled.push_back(evaluation_data->navigation_cancled);
      nav_planner.push_back(findPlanner(evaluation_data->nav_planner));
      route.push_back(findRoute(evaluation_data->start, evaluation_data->goal));
      distance.push_back(evaluation_data->distance);
      driving_time.push_back(evaluation_data->driving_time);
      number_of_resets.push_back(evaluation_data->number_of_resets);    
      number_collisions.push_back(evaluation_data->number_of_collisions);
      // get velocity bins
      if(v_bins.empty()){
	// to get a std::vector inside a std::vector you have to push for each bin
	// the first elemnt step by step!
	for(int i=0; i<_evaluation_settings->number_v_bins; i++){
	  std::vector<int> temp;
	  temp.push_back(evaluation_data->v_bins[i]);
	  v_bins.push_back(temp);
	}
      }
      else{
	for(int i=0; i<_evaluation_settings->number_v_bins; i++)
	  v_bins[i].push_back(evaluation_data->v_bins[i]);
      }
      // get omega bins  
      if(omega_bins.empty()){
	for(int i=0; i<_evaluation_settings->number_omega_bins; i++){
	  std::vector<int> temp;
	  temp.push_back(evaluation_data->omega_bins[i]);
	  omega_bins.push_back(temp);
	}
      }
      else{
	for(int i=0; i<_evaluation_settings->number_omega_bins; i++)
	  omega_bins[i].push_back(evaluation_data->omega_bins[i]);
      }
  }


  void DataExport::exportData()
    {  
        if (!_initialized)
          return;
          
        std::lock_guard<std::recursive_mutex> l(_data_mutex);
        
        if(number_of_datasets > 0)
        {
            ROS_INFO("Export Data!");
            matfp = Mat_CreateVer(_evaluation_settings->evaluation_file.c_str(),NULL,MAT_FT_DEFAULT);
            if ( NULL == matfp ) {
            ROS_ERROR_STREAM("Error creating MAT file "<< _evaluation_settings->evaluation_file.c_str());
            }
                
            // create cell array named "data"
            dims[0] = 8+_evaluation_settings->number_v_bins+_evaluation_settings->number_omega_bins;
            dims[1] = 1;
            cell_array = Mat_VarCreate("data",MAT_C_CELL,MAT_T_CELL,2,dims,NULL,0);
            if ( NULL == cell_array ) {
            ROS_ERROR("Error creating variable for ’data’");
            Mat_Close(matfp);
            }
            
            // create cell-element with the data of turtlebot number
            dims[0] = number_of_datasets;
            dims[1] = 1;
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&turtlebot_number[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,0,cell_element);
            
            // create cell-element with the data of navigation_cancled
            cell_element = Mat_VarCreate(NULL,MAT_C_UINT8,MAT_T_UINT8,2,dims,&navigation_cancled[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,1,cell_element);
            
            // create cell-element with the data of nav_planner (see decoding!)
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&nav_planner[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,2,cell_element);
            
            // create cell-element with the data of route (see decoding!)
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&route[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,3,cell_element);
            
            // create cell-element with the data of distance
            cell_element = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&distance[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,4,cell_element);
            
            // create cell-element with the data of driving_time
            cell_element = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&driving_time[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,5,cell_element);
            
            // create cell-element with the data of number of resets
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&number_of_resets[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,6,cell_element);
            
            // create cell-element with the data of number of collisions
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&number_collisions[0],0);
            if ( NULL == cell_element ) {
            ROS_ERROR("Error creating cell element variable");
            Mat_VarFree(cell_array);
            Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,7,cell_element);
            
            for(int i=0; i< _evaluation_settings->number_v_bins; i++)
            {
            // create cell-element with the data of the bins of linear velocity
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&v_bins[i][0],0);
            if ( NULL == cell_element ) {
                ROS_ERROR("Error creating cell element variable");
                Mat_VarFree(cell_array);
                Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,8+i,cell_element);
            }
            for(int i=0; i< _evaluation_settings->number_omega_bins; i++)
            {
            // create cell-element with the data of the bins of angular velocity
            cell_element = Mat_VarCreate(NULL,MAT_C_INT32,MAT_T_INT32,2,dims,&omega_bins[i][0],0);
            if ( NULL == cell_element ) {
                ROS_ERROR("Error creating cell element variable");
                Mat_VarFree(cell_array);
                Mat_Close(matfp);
            }    
            Mat_VarSetCell(cell_array,8+_evaluation_settings->number_v_bins+i,cell_element);
            }
            
            // write data into the matfile
            Mat_VarWrite(matfp,cell_array,MAT_COMPRESSION_NONE);
            Mat_VarFree(cell_array);
            
            // create int array named "number_bins" to save configuration of the velocity bins
            dims[0] = 2;
            dims[1] = 1;
            int temp_v [2];
            temp_v[0] = _evaluation_settings->number_v_bins;
            temp_v[1] = _evaluation_settings->number_omega_bins;

            bin_config = Mat_VarCreate("number_bins",MAT_C_INT32,MAT_T_INT32,2,dims,(void*)temp_v,0);
            if ( NULL == bin_config ) {
            ROS_ERROR("Error creating variable for number_bins");
            Mat_Close(matfp);
            }
            else{
            Mat_VarWrite(matfp,bin_config,MAT_COMPRESSION_NONE);
            Mat_VarFree(bin_config);
            }
            
            // create int array named "max_values" to save max values
            dims[0] = 2;
            dims[1] = 1;
            double temp_omega [2];
            temp_omega[0] = _evaluation_settings->v_max;
            temp_omega[1] = _evaluation_settings->omega_max;

            bin_config = Mat_VarCreate("max_values",MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,(void*)temp_omega,0);
            if ( NULL == bin_config ) {
            ROS_ERROR("Error creating variable for max_values");
            Mat_Close(matfp);
            }
            else{
            Mat_VarWrite(matfp,bin_config,MAT_COMPRESSION_NONE);
            Mat_VarFree(bin_config);
            }
            

            Mat_Close(matfp);
        } 
  }
  
  void DataExport::declareShortcuts()
  {
      std::ofstream config_file_routes(_evaluation_settings->configuration_file_routes);
      config_file_routes << "number \t start \t goal \t distance \n";
	  
      std::list<std::string> temp_list;
      // get a list of the available nav goals in a list
      for (const auto& list_it : *_goal_list)
      {
	temp_list.push_back(list_it.first);
      }
      
      // for the goals
      int number_of_goals = 0;
      for(auto it1=_goal_list->begin(); it1 != _goal_list->end() ; it1++)
      {
	if(std::next(it1,1) != _goal_list->end())
	{
	  for (auto it2=std::next(it1,1); it2 != _goal_list->end(); ++it2)
	  {
	    ++number_of_goals;
	    // give every possible route a number
	    std::pair<std::string, int> element_route ((it1->first + it2->first), number_of_goals );
	    // calculate the beeline between the points
	    double distance = sqrt(pow(it2->second.position.x - it1->second.position.x,2)+pow(it2->second.position.y - it1->second.position.y,2));
	    std::pair<int, double> element_distance (number_of_goals, distance );
	    
	    // save ShortCuts to the configfile  
	    config_file_routes << number_of_goals << "\t" << it1->first << "\t" << it2->first << "\t" << distance << "\n";
	    _route_map.insert(element_route);	  
	  }
	}
      }
      
	config_file_routes.close();
  }

  int DataExport::findRoute(std::string start, std::string goal)
  {
    std::string route = start+goal;
    std::unordered_map<std::string, int>::const_iterator thisRoute = _route_map.find(route);
    
    if ( thisRoute == _route_map.end() )
    {    
      route = goal+start;    
      thisRoute = _route_map.find(route);
      
      if ( thisRoute == _route_map.end() )
      {
	ROS_INFO("Route not found!");
	return 0;
      }
    }
    return thisRoute->second;
  }

  int DataExport::findPlanner(std::string planner)
  {
    std::unordered_map<std::string, int>::const_iterator thisRoute = _planner_map.find(planner);
    // add planner if it isn't listed in the planner map
    if ( thisRoute == _planner_map.end() )
    {  
	  ++_number_of_planner;
	  std::pair<std::string, int> element (planner, _number_of_planner);
	  _planner_map.insert(element);
	  
	  // save ShortCuts to the configfile 
	  _config_file_planner->open(_evaluation_settings->configuration_file_planner,std::ofstream::app);
	  std::string text_to_write = std::to_string(_number_of_planner) + "\t" + planner + "\n";
	  const char* line_to_write = text_to_write.c_str();
	  _config_file_planner->write(line_to_write,text_to_write.length());
	  _config_file_planner->close();
	  ROS_INFO_STREAM("Planner (" << planner << ") added to this configuration!");
	return _number_of_planner;
    }
    return thisRoute->second;
  }

/**
 * if the MatIO Libary is not found take this dummy
 */
#else
	int     	Mat_Close(mat_t *mat){};
	mat_t   	*Mat_CreateVer(const char *matname, const char *hdr_str,
			      enum mat_ft mat_file_ver){};
	mat_t   	*Mat_Open(const char *matname, int mode){};
	void   		Mat_VarFree(matvar_t *matvar){};
	matvar_t        *Mat_VarCreate(const char *name, enum matio_classes class_type, 
                              enum matio_types data_type, int rank, size_t *dims,
                              void *data, int opt){};
	matvar_t        *Mat_VarReadInfo(mat_t *mat, const char *name){};
	matvar_t        *Mat_VarSetCell(matvar_t *matvar, int index, matvar_t *cell){};
	int             Mat_VarWrite(mat_t *mat, matvar_t *matvar,
                              enum matio_compression compress){};
			      
	DataExport::DataExport(){
	    ROS_WARN("MatIO Libary is not found!");
	}
	
	void DataExport::init(const EvaluationSettings* evaluationSettings, std::unordered_map<std::string, geometry_msgs::Pose>* goal_list){};
	void DataExport::exportData(){};
	void DataExport::addData(const turtle_commander_messages::EvaluationData::ConstPtr&  evaluationData){};
	void DataExport::declareShortcuts(){};
	int DataExport::findRoute(std::string start,std::string goal){};
	int DataExport::findPlanner(std::string planner){};
#endif



