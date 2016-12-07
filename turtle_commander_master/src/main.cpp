
#include <turtle_commander_master/turtle_commander_master.h>




int main(int argc, char **argv)
{  
  ros::init(argc, argv, "turtle_commander_master");
  ros::NodeHandle _nhandle;

  TurtleCommanderMaster turtle_commander_master;
  
  // start the master (blocking call)
  turtle_commander_master.start();
    
  return 0;
}

