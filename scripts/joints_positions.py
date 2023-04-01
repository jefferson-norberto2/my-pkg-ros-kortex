#!/usr/bin/env python3

import rospy
import time
import json

from kortex_driver.srv import *
from kortex_driver.msg import *

class MoveJoints:
    def __init__(self):
        try:
            rospy.init_node("move_joints")

            self.all_notifs_succeeded = True

            self.robot_name = "my_gen3_lite"
            rospy.loginfo("Using robot_name " + self.robot_name)

            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.action_topic_callback)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy   .wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            joint1 = JointAngle()
            joint2 = JointAngle()
            joint3 = JointAngle()
            joint4 = JointAngle()
            joint5 = JointAngle()
            joint6 = JointAngle()

            joint1.joint_identifier = 1
            joint2.joint_identifier = 2
            joint3.joint_identifier = 3
            joint4.joint_identifier = 4
            joint5.joint_identifier = 5
            joint6.joint_identifier = 6

            joint1.value = 200
            joint2.value = 0
            joint3.value = 0
            joint4.value = 0
            joint5.value = 0
            joint6.value = 0

            my_joints = JointAngles()

            my_joints.joint_angles.append(joint1)
            my_joints.joint_angles.append(joint2)
            my_joints.joint_angles.append(joint3)
            my_joints.joint_angles.append(joint4)
            my_joints.joint_angles.append(joint5)
            my_joints.joint_angles.append(joint6)

            self.my_constrained_joints = ConstrainedJointAngles()

            self.my_constrained_joints.joint_angles = my_joints


            self.handle_identifier = 1001

            self.is_initialized = True
        except:
            self.is_initialized = False

    def action_topic_callback(self, notif):
        self.last_action_notif_type = notif.action_event

    def my_clear_faults(self):
        try:
            self.clear_faults()
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        
    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)
        return False
    
    def my_home_robot(self, identifier=2):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = identifier # Value default home is 2 and vertical is 3
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            self.execute_action(req)
            return self.wait_for_action_end_or_abort()
        except rospy.ServiceException as error:
            rospy.logerr("Failed to call ReadAction or ExecuteAction " + str(error))
            return False
    
    def my_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
            rospy.loginfo("Successfully activated the Action Notifications!")
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
                    
        rospy.sleep(1.0)
        return True
    
    def execute_joints_positions(self, pose_name: str):
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_joint_angles.append(self.my_constrained_joints)
        req.input.name = pose_name
        req.input.handle.action_type = ActionType.REACH_JOINT_ANGLES
        req.input.handle.identifier = self.handle_identifier
        self.handle_identifier += 1

        rospy.loginfo("Sending " + pose_name + "...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
            rospy.loginfo("Waiting for "+ pose_name +" to finish...")
        except rospy.ServiceException:
            rospy.logerr("Failed to send " + pose_name)

    
    def call_move_joints(self, pose_name="pose_zero"):
        self.execute_joints_positions(pose_name)
        return self.wait_for_action_end_or_abort()
            
    def main(self):
        success = self.is_initialized

        if success:
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.my_clear_faults()
            
            #*******************************************************************************
            # Start the example from the Home position
            success &= self.my_home_robot()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.my_subscribe_to_a_robot_notification()
            
            success &= self.call_move_joints("joints")
                        
            if not success:
                rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = MoveJoints()
    ex.main()

