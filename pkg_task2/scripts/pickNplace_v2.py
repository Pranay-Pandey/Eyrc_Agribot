#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import numpy


class Ur5Moveit:

    # Constructor
    def __init__(self):
     
        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group1 = "arm"
        self._planning_group2 = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self.group1 = moveit_commander.MoveGroupCommander(self._planning_group1)
        self.group2 = moveit_commander.MoveGroupCommander(self._planning_group2)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self.planning_frame_1 = self.group1.get_planning_frame()
        self._planning_frame_2 = self.group2.get_planning_frame()
        self._eef_link = self.group1.get_end_effector_link()
        self.group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self.planning_frame_1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self.group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        
    
    def set_joint_angles_ee(self, arg_list_joint_angles):

        flag_plan = False
        while not flag_plan :
            list_joint_values = self.group2.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            self.group2.set_joint_value_target(arg_list_joint_angles)
            self.group2.plan()
            flag_plan = self.group2.go(wait=True)

            list_joint_values = self.group2.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            pose_values = self.group2.get_current_pose().pose
            rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            rospy.loginfo(pose_values)

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
    
        
    def set_joint_angles_arm(self, arg_list_joint_angles):

        flag_plan = False
        while not flag_plan :
            list_joint_values = self.group1.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            self.group1.set_joint_value_target(arg_list_joint_angles)
            self.group1.plan()
            flag_plan = self.group1.go(wait=True)

            list_joint_values = self.group1.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            pose_values = self.group1.get_current_pose().pose
            rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            rospy.loginfo(pose_values)

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
            
      

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    end_effector_closed = [0.8040]
    end_effector_open = [0.0000]

    start_pose = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
    drop_pose = [0.000, 0.900, 0.000, 0.000, 0.000, 0.000]
    
    t1  = numpy.radians([-1.721, -2.92, -86.28, 89.527, -87.186, -171.211])
    t1_ = numpy.radians([-11.410, -2.566, -95.6326, 98.1672, -101.3764, -179.9976])

    t2  = numpy.radians([75.9399, 33.3322, -2.49, -210.740, -165.884, -0.035])
    t2_ = numpy.radians([79.162, 45.96, -14.326, -211.632, -169.174, 0.0044])

    t3  = numpy.radians([119.45, 39.886, 6.258, -46.082, 29.479, -179.979])
    t3_ = numpy.radians([110.488, 49.867, -3.68, -46.193, 20.4763, -180.015])

    while not rospy.is_shutdown():
        
        ur5.set_joint_angles_ee(end_effector_open)
        ur5.set_joint_angles_arm(t1)
        ur5.set_joint_angles_arm(t1_)
        ur5.set_joint_angles_ee(end_effector_closed)
        ur5.set_joint_angles_arm(drop_pose)
        ur5.set_joint_angles_ee(end_effector_open)
        ur5.set_joint_angles_arm(start_pose) 
        
        ur5.set_joint_angles_arm(t2)
        ur5.set_joint_angles_arm(t2_)
        ur5.set_joint_angles_ee(end_effector_closed)
        ur5.set_joint_angles_arm(drop_pose)
        ur5.set_joint_angles_ee(end_effector_open)
        ur5.set_joint_angles_arm(start_pose)  

        ur5.set_joint_angles_arm(t3)
        ur5.set_joint_angles_arm(t3_)
        ur5.set_joint_angles_ee(end_effector_closed)
        ur5.set_joint_angles_arm(drop_pose)
        ur5.set_joint_angles_ee(end_effector_open)
        ur5.set_joint_angles_arm(start_pose)

        break
        

    del ur5


if __name__ == '__main__':
    main()
