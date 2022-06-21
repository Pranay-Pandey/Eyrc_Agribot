# Eyrc Agribot

![image](https://user-images.githubusercontent.com/79053599/173903917-1ac7cefe-e3f1-4669-8d54-193f1c38fba1.png)

One of the themes of E-yantra 2021 was Agri-bot: 
An agricultural bot designed to be automated in ripe tomato collection from the yard autonomously and bring all the fruits by traversing through out the yard, picking and placing it in the basket.

<b>Agribot (AB)</b>


This theme inspires engineers in the area of agricultural automation. Agribot uses an autonomous Ground Vehicle (AGV) to traverse in a simulated Greenhouse environment, find the targeted yield and correctly execute pick-&-place. The AGV is retrofitted with a customized gripper to plug yields easily, and so is the created environment ensured to compile with the gripper. The theme is divided into tasks to build the AgriBot system step-by-step under the guidance of e-Yantra mentors.

<b>Concepts used</b>: 
Robot Operating System (ROS), Autonomous Navigation, Perception, Pick & Place, etc.

<b>Implementation</b>: Simulator based.

<b>System Requirements:</b>
Operating System: Ubuntu 20
<br>
Processor: more than 4 cores, x86_64 (64-bit x86 Instruction Set)
<br>
HDD or SSD storage space: 30GB or more
<br>
RAM: 8GB or more

The robot and the simulation world is provided and and completetion requires writing ROS nodes (scripts) to operate the robot to complete the task as an agribot.


<h2>The Tasks</h2>
The complete work was divided into 3 major tasks <br><br>

<b> Task 0 </b> - Installation and setup of all the required programs and softwares. Generating a demo script that launches turtlebot and makes infinity symbol using it.


<b> Task 1 </b> - Writing a script to move the robot around the 2 rows of plants so as to traverse all the plants in the rows using the wheels of the robot.


<b> Task 2 </b> - Writing script to pluck the fruits of a single plant placed in front of the robot using the robotic gripper attached to the agri bot using move-it planner.


<b> Task 3 </b> - <br>
<i>Task 3.1</i> - Writing script for detection of ripe red fruits through the rgbd camera using computer vision to detect red color and then broadcast the tf coodinates and then plucking the fruits at those coordinates using the robotic gripper using move-it.

<i>Task 3.2</i> - Writing script for detection of ripe red fruits through the rgbd camera using computer vision to detect red color while the robot traverses the rows of plants using the wheels.

<hr>

<h1> Setting up the world and the configuration files</h1>

Download the files the official github repository of the eRYC agribot from 
<a href = 'https://github.com/erts-RnD/eYRC-2021_Agribot'>here</a> or a small version and limited files from 
<a href = 'https://drive.google.com/file/d/1FbfnzgGJbvbTyBv4jPj8YnUuE-ieYeC7/view?usp=sharing'>here</a>.

Make a catkin package and place these files making a src folder in the workspace

To see these scipts running

First launch the world from package ebot_gazebo for the respective task -

<br>TASK 0-
<pre> <code> roslaunch ebot_gazebo task0.launch </code></pre>
TASK 1-
<pre> <code> roslaunch ebot_gazebo task1_3.launch </code></pre>
TASK 2-
<pre> <code> roslaunch ebot_gazebo task2.launch </code></pre>
TASK 3_1-
<pre> <code> roslaunch ebot_gazebo task3_1.launch </code></pre>
TASK 3_2-
<pre> <code> roslaunch ebot_gazebo task3_2.launch </code></pre>

<br><br>
Additionaly to run the script in the respective task -
Either build the respective package
<pre> <code> rosrun pkg_task0 node_turtle_revolve.py</code></pre>

or directly run the python file from the file location using 
<pre> <code> python3 node_turtle_revolve.py</code></pre>

<b>Additionally</b>, in the src folder in 
<a href = 'https://drive.google.com/file/d/1FbfnzgGJbvbTyBv4jPj8YnUuE-ieYeC7/view?usp=sharing'>here</a>
there are some script files in folder such as <br>eYRC-2021_Agribit/agri_bot_moveit/scripts

<br>
<b>Moreover</b>, you will find some bag files in packages such as task0
<br>
To play a bag file simply run the bag file after launching the world (.launch) file of the respective task
<pre><code>rosbag play bagfile_name.bag</code></pre>
