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




TASK 2





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