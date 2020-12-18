This is the read me for the Mobile Robotics assignment. This will make it much easier to remember all the things that
we have done and hold all the important information that we need to share with one another.

The document will be broken up into 5 sections that will be listed immediately below:
1) Set Up
2) Task 1: Mapping of the Maze
3) Task 2: Navigation
4) Task 3: Rescue operation
5) Task 4: Creativity task



SETUP
In this portion we will go over all the necessary files needed for the operation of the robot.
Make sure to install the following things

    sudo apt-get install ros-melodic-dwa-local-planner


Everytime that you bring up the terminal to run something make sure to type the following command

    export TURTLEBOT3_MODEL=burger


TASK 1
This task is to implement an algorithm so the robot explores the whole Maze environment. The robot is required to start outside of the Maze enviornment. The robot should come back to the start point after exploration is over.
The first step in this portion of the project is to create an opening in the maze. To do this you need to navigate to the worlds directory inside of the Fira_maze package.
To do this from your intial terminal type

    cd catkin_ws/src/fira_maze/worlds

then type the below statement

    code maze_s.world

to create the opening you need to remove a section of the one of the walls. To do this we create our opening in the yellow wall. To do this we deleted <model name='Box_Yellow_clone_3'> and all pertaining code. The lines pertaining to this were lines 2632 to 2733.
After deleteing this make sure to save. You must now go into the maze_1_world.launch file to modify the spawn position of the robot.
To do this, change your directiory by typing 

    cd ../launch

this will bring you to the launch file older inside of the fira_maze directory. You then need to type the code that is written below

    code maze_1_world.launch

This will let you open the maze_1_world.launch file, here you must modify lines 3, 4, and 5 which control the spawn position of the robot. The values we chose are written below. This signifies the position just outside the maze. Copy and paste the code shown below.

  <arg name="x_pos" default="-2"/>
  <arg name="y_pos" default="0.145804"/>
  <arg name="z_pos" default="0.15"/>

Once the position is set, then you can go about generating the map. There are some things that need to be added to the maze_1_world.launch file.


when you are about to run the maze, you need to dow






TASK 2
The goal of this task is to have to robot navigate through the maze to a specific point, stay there for 5 seconds and then return to the place that it originally spawned in. The challenge of this task is that the robot must receive the location as a tuple, sent to the '/target' topic.
This task can be broken up into 5 steps. 
Step 1) Spawn the robot, and load the map of the enviornment.
Step 2) Specify the location that you wish the robot to travel to.
Step 3) Publish the location as a tuple on the '/target'
Step 4) Have the robot, subscribe to the '/target' and then proceed to the location.

The way that you will complete step one is to start by modifying the maze_1_world.launch file. First you need to comment out the following lines of code to make sure that Slam's gmapping capability doesn't activate allowing the preprocessed map to come up. To do this, open up the maze_1_world.launch file by moving to that directory from the home directory. See line below.

    cd catkin_ws/src/fira_maze/launch

Once in the launch directory of the fira_maze directory, you need to open the maze_1_world.launch file. To do so, type the command below.

    code maze_1_world.launch

In the file you will need to make a few edits, start by commenting out lines 21 through 24, this will look like this
    
    <--<include file="$(find turtlebot3_slam)/launch/turtlebot3_$(arg slam_methods).launch"> 
    <arg name="model" value="turtlebot3_burger.urdf.xacro"/> 
    <arg name="configuration_basename" value="$(arg configuration_basename)"/> 
  </include>-->

Then you will need to add the following lines to the top of the file, these will be line 13 and 14

    <arg name="map_file" default="/home/river/map/correct_maze_map.yaml"/><!-- edit this with the location of the map-->
  <arg name="move_forward_only" default="false"/>

The following lines will be what uploads tehe map for the robot, which is why it arg map_file is very important. It must point to the right location of the maps .yaml file otherwise, it will not work.
Once that is done go to the bottom of the file a insert the following code, these will be lines 47, through 57.

    <!-- Map server-->
  <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/> 

  <!-- AMCL -->
  <include file="$(find turtlebot3_navigation)/launch/amcl.launch"/> 

  <!-- move_base -->
  <include file="$(find turtlebot3_navigation)/launch/move_base.launch">
    <arg name="model" value="burger"/>
    <arg name="move_forward_only" value="$(arg move_forward_only)"/>
  </include>

These codes will allow you to have the robot move. After this is input, you can save the file. Then you will need to change directories out of the launch directory, and move to the scripts directory. to do this, in the terminal, type the following code.

    cd ../scripts

Once in the scripts directory of the fira_maze directory, you will want to type the following code.

    code movebase_client.py

This will create the python script needed to command the robot to move to a specific location of the maze. As well as allow it to subscribe to the '/target' topic. Once the file is created you will need to copy and paste the following code.

    #!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

tuple_x_value = 0.0
tuple_y_value = 0.0

def callback(data):
    float_converted_tuple = tuple(float(num) for num in data.data.replace('(', '').replace(')', '').split(', ')) 
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(float_converted_tuple)
    tuple_x_value,tuple_y_value = float_converted_tuple
    rospy.loginfo("The penguin x value of the tuple is %4.2f and the y component is %6.3f", tuple_x_value, tuple_y_value)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("target", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def movebase_client(tuple_x_value,tuple_y_value):

    #rospy.Subscriber("target", String, callback)

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = tuple_x_value
    goal.target_pose.pose.position.y = tuple_y_value
    goal.target_pose.pose.position.y = -0.00143
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        listener()
        rospy.loginfo("Puppy x value of the tuple is %4.2f and the y component is %6.3f", tuple_x_value, tuple_y_value)
        result = movebase_client(tuple_x_value, tuple_y_value)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

The listener() fuction is what subscribes to the '/target' topic, with the callback function being what takes the tuple values, converts it from a string to a float, and then breaks it up to be able to be a position value. Once this has been input, you will need to save the file. After saving the file you will need to make the file executable, return to terminal and type the following code.

    chmod +x movebase_client.py

This will make the file executable. Then you will need to go an include the file in the CMakeList. To do this you will need to change directory out of the script directory to the Fira_maze directory.

    cd ../

Here type the following code

    code CMakeLists.txt 

Here you will need to navigate to line 162, so that you will be able to add the the following line to line 163

    scripts/movebase_client

Save the file, and then return to terminal so that you can change directory back to the root workspace, and then run catkin_make

    cd
    cd catkin_ws
    catkin_make

Once that is there are two more systems that need to be modified. The first will be the .yaml file that holds the information of the map. For our project, that .yaml file is correct_maze_map.yaml. Open what ever respective .yaml file holds your map information, here you will need to modify your maps origin, so that it spawn correctly relative to the location of the robot, that we modified in Task 1 to no longer spawn at the origin. Change the origin line in the file, to read what is listed below.

    origin: [-8.000000, -10.180000, 0.000000]

Save that file, then, you will need to navigate to the turtlebot3 directory, specifically to the turtlebot3_navigation directory. Then within that directory navigate to the params directory. From the home directory you can type the following commands and it will take you there.

    cd catkin_ws/src/turtlebot3/turtlebot3_navigation/param/

Once here, you need to open the costmap_common_params_burger.yaml, to do this type the below code.

    code costmap_common_params_burger.yaml

This will open the costmap, which you need to modify so that it modify the way the robot views itself, so that it travel through the map. Modify the .yaml, by copying and pasting the entire code below.

    obstacle_range: 0.0
    raytrace_range: 0.0

    footprint: [[-0.015, -0.015], [-0.015, 0.015], [0.041, 0.015], [0.041, -0.015]]
    #robot_radius: 0.105

    inflation_radius: 0.04
    cost_scaling_factor: 0.001

    map_type: costmap
    observation_sources: scan
    scan: {sensor_frame: base_scan, data_type: LaserScan, topic: scan, marking: true, clearing: true}

After that is done, save the file. After this you are ready to start Task 2. Open up three terminals, and remeber to source them. In the first terminal start by launching the maze_1_world.launch file. This can be done with the following code below.

    roslaunch fira_maze maze_1_world.launch

In another terminal, naviagte to the following directory with the code below

    cd catkin_ws/src/fira_maze/script/

Here type the following code to launch the mover.

    ./movebase_client.py

In the third terminal, run the following command to specify the location on the map you wish the robot to move to.

    rostopic pub /target std_msgs/String '(1.78, -0.274)

And watch as the robot, goes to that location waits for five seconds, and then returns home.



TASK 3
This task is to have the original robot save a secondary robot from the somewhere in the maze. For the purpose of this task, the original robot will be refered to as Robot_Master, and the secondary robot will be refered to as Robot_Slave. 
The first step in completing this task is the spawn Robot_Slave somewhere in the maze. 
The second step is to have Robot_Master search for Robot_Slave, using its map of the maze. 
The third step activates after Robot_Slave detects the first robot. It should publih the string msg "hello" onto the '/comm' topic
The fourth step is for Robot_Master to respond on '/comm' topic
The fifth step after Robot_Master has responded, it should return on its path to exit the maze. Publishing its position.
The sixth step is for Robot_Slave to subscribe to Robot_Master's position and follow it out of the maze, along the same path.

To complete step 1, there are a couple things that need to be done. 
First, naviagte to the src folder in your workspace. Next naviagte to the Fira_Maze directory. Then navigate to the launch file directory. In this directory type exactly what is listed below

    code one_robot.launch

This will create a launch file that is needed. Copy and paste and save exactly what is written below.

<launch>
    <arg name="robot_name"/>
    <arg name="init_pose"/>

    <node name="spawn_minibot_model" pkg="gazebo_ros" type="spawn_model"
     args="$(arg init_pose) -urdf -param /robot_description -model $(arg robot_name)"
     respawn="false" output="screen" />

    <node pkg="robot_state_publisher" type="state_publisher" 
          name="robot_state_publisher" output="screen"/>

    <!-- The odometry estimator, throttling, fake laser etc. go here -->
    <!-- All the stuff as from usual robot launch file -->
</launch>



In the same catkin_ws/src/Fira_maze/launch directory type exactly what is listed below

    code robots.launch

This will create a launch file that is needed. Copy and paste and save exactly what is written below.

<launch>
  <!-- No namespace here as we will share this description. 
       Access with slash at the beginning -->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <param name="robot_description" command="$(find xacro)/xacro.py $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
    

  <!-- BEGIN ROBOT 1-->
  <group ns="robot_master">
    <param name="tf_prefix" value="robot_master_tf" />
    <include file="$(find fira_maze)/launch/one_robot.launch" >
      <arg name="init_pose" value="-x -2 -y 0.145804 -z 0.15" />
      <arg name="robot_name"  value="RobotMaster" />
    </include>
  </group>

  <!-- BEGIN ROBOT 2-->
  <group ns="robot_slave">
    <param name="tf_prefix" value="robot_slave_tf" />
    <include file="$(find fira_maze)/launch/one_robot.launch" >
      <arg name="init_pose" value="-x 0.0 -y 0.0 -z 0.0" />
      <arg name="robot_name"  value="RobotSlave" />
    </include>
  </group>
</launch>


Then in the same catkin_ws/src/Fira_maze/launch directory type exactly what is listed below

    code maze_1_world.launch

This will allow you to edit the needed launch file, type CTRL + A and delete the text of the file and replace it with what is written below. 

<launch>

  <node pkg="tf" type="static_transform_publisher" name="camera_tf" args="-1.95 -0.55 2.0 -1.58 0 -1.58 /odom /camera_link 100"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find fira_maze)/worlds/maze_2.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- include our robots -->
  <include file="$(find fira_maze)/launch/robots.launch"/>
</launch>

To launch the world and the turtlebot, run the following command

    roslaunch fira_maze maze_1_world.launch

To verify that everything worked properly, in a seperate terminal, run the following command

    rostopic list

From the list that appears you should be able to see that there are two sets of topics. One dedicated to Robot_Master and then one dedicated to Robot_Slave

For the next steps it is important to download the following package. In another terminal type, what is written below

    sudo apt-get install ros-noetic-teleop-twist-keyboard

wait for the instilation to finish. Then run the follow command to control Robot_Master

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot_master/cmd_vel

to control Robot_Slave use the following command

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot_slave/cmd_vel



TASK 4