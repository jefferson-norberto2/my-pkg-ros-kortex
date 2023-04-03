#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg

from math import radians

class MoveJoint:
  def __init__(self):
    # Initialize the node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_joint')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", True)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_joint_angles(self, tolerance):
    self.arm_group 
    success = True

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      sucesse &= self.move_to_joints_angles(90,  0, 45, 45, 0, 90, 0)
    elif self.degrees_of_freedom == 6:
        success &= self.move_to_joints_angles(52, -10, 81, 0, 53, 2)    # position left

    return success
  
  def move_to_joints_angles(self, j1: int, j2: int, j3: int, j4: int, j5: int, j6: int, j7: int=None):
    joint_positions = self.arm_group.get_current_joint_values()
    
    joint_positions[0] = radians(j1)
    joint_positions[1] = radians(j2)
    joint_positions[2] = radians(j3)
    joint_positions[3] = radians(j4)
    joint_positions[4] = radians(j5)
    joint_positions[5] = radians(j6)
    if j7:
      joint_positions[6] = radians(j7)
    
    self.arm_group.set_joint_value_target(joint_positions)
    return self.arm_group.go(wait=True)


def main():
  example = MoveJoint()
  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  
  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles(tolerance=0.01) #rad
    print (success)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()