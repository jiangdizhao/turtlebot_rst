# turtlebot_rst
This meta package contains relevant ROS packages for our turtlebot

# **Quick-Start Guide**

## turtlebot_rst_bringup:
This package provides the launch of the following applications:
 * map server
 * stage simulation
 * real turtlebot: all neccessary drivers and navigation components
 * simulated roboter in stage: all neccessary navigation components

### Start-up for **real** turtlebots:

 1. Start the roscore-service (this service is condition of the ROS-system)
 
     ```
     roscore
     ```
 2. Start the map server
     (if the map server is needed only for a single turtlebot, you could start it with the second description in step 3; configuration: *turtlebot_rst_bringup/maps/rst_lab.yaml*)
     
     ```	
     roslaunch turtlebot_rst_bringup start_map_server.launch
     ```
 3. Initialize the turtlebot
     This step should be invoked on the mobile computer of the turtlebot.
     Start a terminal and connect to the turtlebot's pc via SSH (replace X by an appropriate turtlebot number):
     ```
     ssh turtlebot@turtlebotX
     ```
     Now you have to fill in the user password and then a connection will we established.
     To communicate with the roscore on the lab-PC, you have to change the ROS_MASTER_URI:
     ```
     export ROS_MASTER_URI=http://<Computername>:11311
     ```

     * turtlebot **X** starts from the defined Dockingstation **X**:
          (configuration: *turtlebot_rst_bringup/cfg/turtlebots/turtlebot_**X**.launch*)
	  
          ```
          roslaunch turtlebot_rst_bringup start_client.launch simulation:=false turtlebot_no:=X         
          ```
     * turtlebot **X** starts from any position (x,y,a) (-> AMCL initial location):
     
          ```
          roslaunch turtlebot_rst_bringup start_robot.launch
                    robot_name:=robot_X
                    local_planner:=teb	(teb/dwa/eband)
                    start_map_server:=false	(s. step 1)
                    initial_pose_x:=x
                    initial_pose_y:=y
                    initial_pose_a:=a	(rad)
                    simulation:= false
          ```

### Start-up for **Stage simulation**:
 1. Start simulation and map server
    (configuration: *turlebot_rst_bringup/stage/rst_lab.world*)
    
     ```
     roslaunch turtlebot_rst_bringup start_stage.launch
     ```
 2. Initialize the simulated roboter
     * turtlebot **X** starts from the defined Dockingstartion **X**
          (configuration: *turtlebot_rst_bringup/cfg/turtlebots/turtlebot_**X**.launch*)
	  
          ```
          roslaunch turtlebot_rst_bringup start_client.launch simulation:=true turtlebot_no:=X        
          ```
     * turtlebot **X** starts from any position (x,y,a) (-> AMCL initial location):
     
          ```
          roslaunch turtlebot_rst_bringup start_robot.launch
                    robot_name:=robot_X
                    local_planner:=teb	(teb/dwa/eband)
                    start_map_server:=false	(s. step 1)
                    initial_pose_x:=x
                    initial_pose_y:=y
                    initial_pose_a:=a	(rad)
                    simulation:= true
          ```

Step 2 is in both cases for any number of robots repeatable.


_______________________

## turtle_commander_client:

The *turtle_commander_client* node implements a state machine for a single turtlebot.
Currently, two different modes are available:
* *Random mode*: Randomly select a goal from the parameter server and travel around until the battery is empty or if the robot is not allowed to drive anymore. In that case, it heads for the next docking station to recharge automatically.
* *Remote mode*: Obtain goals and commands from the *turtle_commander_master* (see below). In this mode, multiple turtlebots can be maintained and controlled by the master, e.g. in order to gather specifiy data. You might start the *turtle_commander_master* first (see below).

 1. Configure the *goals.yaml* file:
		*turtle_commander_client/cfg/goals.yaml*
     * define the time of operation (*allow_driving_*...)
     * Select the Modus:
          - random_mode (independent random goal selection)  *#overrides remote_mode if true*
          - remote_mode (assignment of goals by *turtle_commander_master*)
     * Specify simulation mode if desired (skip charging and ignore driving time window)
          - simulation_mode *true* or *false*
 2. Start the Turtle-Commander-Client for robot **X**
 
     ```
     roslaunch turtle_commander_client turtle_commander_client.launch
          turtlebot_no:=X
          local_planner:=teb		(opt., zur Matlab-Auswertung)
          initial_location:=docking1	(opt., zur Anmeldung beim Master)
     ```

_______________________


If you want to maintain and coordinate serveral turtlebots with a *turtle_commander_master*, run the *turtle_commander_client* in *remote* mode. Make sure to setup your network properly for ROS with multiple PCs.


## turtle_commander_master:
 1. Configure the *goals.yaml* file:
		*turtle_commander_client/cfg/goals.yaml*
     * configure goals (add, delete, comment)
       If one goal should not be choosen, you can uncomment it (#) in the YAML-file. You can add new goals with a **clear** name and its position (x,y,a).
 2. Start the Turtle-Commander-Master
 
     ```
     roslaunch turtle_commander_master turtle_commander_master.launch
          work_path:="/home/robotik_3/Documents/Versuch_1/"	(path for Matlab Export)     
     ```

_______________________


## Visualization

Visualization with RVIZ for 
 1. a single robot (default no.: 1)
 
     ```
     roslaunch turtlebot_rst_bringup view_navigation.launch turtlebot_no:=1
     ```
 2. multimater
 
     ```
     roslaunch turtlebot_rst_bringup view_navigation.launch 
          turtlebot_no:=1
          multi:=true
          simulation:=true	(if you have one roscore and could see all robot data)
     ```



