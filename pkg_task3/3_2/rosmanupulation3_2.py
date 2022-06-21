# /usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import actionlib
import sys
import numpy

pose = geometry_msgs.msg.TransformStamped()

rospy.init_node("tomato_picker", anonymous=False)

planning_group1 = "arm"
planning_group2 = "gripper"
commander = moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group1 = moveit_commander.MoveGroupCommander(
    planning_group1)
group2 = moveit_commander.MoveGroupCommander(
    planning_group2)
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=1)

exectute_trajectory_client = actionlib.SimpleActionClient(
    'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

planning_frame_1 = group1.get_planning_frame()
planning_frame_2 = group2.get_planning_frame()
eef_link = group1.get_end_effector_link()
group_names = robot.get_group_names()
curr_state = robot.get_current_state()  


# Joint angle for closing the end effector
end_effector_closed = [0.8040]

# Joint angle for opening the end effector
end_effector_open = [0.0000]

# Joint angles for bringing the gripper closer to the basket for dropping
# tomatoes
drop_pose = [0.000, 0.900, 0.000, 0.000, 0.000, 0.000]

# Joint values to set the arm of the gripper in a starting position
# from where it can see both the tomatoes through its camera
t1 = numpy.radians([83.55, -22.94, -68.253, 91.20, -7.22, -0.4248])


def go_to_pose(arg_pose):

    # Function used for Inverse kinematics 
       
    flag= False
    while not flag:
        pose_values = group1.get_current_pose().pose
        group1.set_pose_target(arg_pose)
        flag = group1.go(wait=True)
        if (flag == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

def set_joint_angles_ee(arg_list_joint_angles):  

    # function to open and close end effector

    count = False
    while not count:
        group2.set_joint_value_target(arg_list_joint_angles)
        group2.plan()
        count = group2.go(wait=True)

def set_joint_angles_arm(arg_list_joint_angles):

    # function to implement forward kinematics of arm

    count = False
    while not count:

        group1.set_joint_value_target(arg_list_joint_angles)
        group1.plan()
        count = group1.go(wait=True)


set_joint_angles_arm(t1)   # Set the arm in starting position

if __name__ == '__main__' :

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)
    

    # Listen to the tf published for tomatoes. Get its coordinates with respect to ebot_base
    # It was found through experimenting that all tf of tomatoes are at a small offset in y direction\
    # with respect to ebot_base reference frame.
    # Read the tf for obj0 (tomato) and perform pick and place operation
    # After picking the arm comes to starting position
    # Continue this process, in each step the furthest tomato will pe picked first


    # The following procedure is followed to pick and place tomatoes :-
    # 1) Go near a tomato
    # 2) Move closer to tomato such that the tomato is between end effector
    # 3) Close the end effector
    # 4) Bring the tomato closer to the basket
    # 5) Open the end effector
    # 6) Come back to start position
    # 7) Repeat the same until all tomatoes are picked
    
    while not rospy.is_shutdown():
        try :
            trans = tf_buffer.lookup_transform('ebot_base', 'obj0', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
            rate.sleep()
            continue
        rospy.loginfo(trans)
        
        set_joint_angles_ee(end_effector_open)
        
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = trans.transform.translation.x
        ur5_pose_1.position.y = trans.transform.translation.y - 0.5
        ur5_pose_1.position.z = trans.transform.translation.z            
        ur5_pose_1.orientation.x = 0
        ur5_pose_1.orientation.y = 0
        ur5_pose_1.orientation.z = 0
        ur5_pose_1.orientation.w = 1  
        go_to_pose(ur5_pose_1)
        
        pose_values = group1.get_current_pose().pose

        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = pose_values.position.x
        ur5_pose_1.position.y = pose_values.position.y + 0.24
        ur5_pose_1.position.z = pose_values.position.z            
        ur5_pose_1.orientation.x = pose_values.orientation.x
        ur5_pose_1.orientation.y = pose_values.orientation.y
        ur5_pose_1.orientation.z = pose_values.orientation.z
        ur5_pose_1.orientation.w = pose_values.orientation.w
        go_to_pose(ur5_pose_1)
        
        set_joint_angles_ee(end_effector_closed)

        set_joint_angles_arm(drop_pose)

        set_joint_angles_ee(end_effector_open)
        
        set_joint_angles_arm(t1)

        rate.sleep()