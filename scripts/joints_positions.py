#!/usr/bin/env python3

import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *

import json

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


            self.joint1 = JointAngle(joint_identifier=1)
            self.joint2 = JointAngle(joint_identifier=2)
            self.joint3 = JointAngle(joint_identifier=3)
            self.joint4 = JointAngle(joint_identifier=4)
            self.joint5 = JointAngle(joint_identifier=5)
            self.joint6 = JointAngle(joint_identifier=6)

            my_joints = JointAngles()
            my_joints.joint_angles.append(self.joint1)
            my_joints.joint_angles.append(self.joint2)
            my_joints.joint_angles.append(self.joint3)
            my_joints.joint_angles.append(self.joint4)
            my_joints.joint_angles.append(self.joint5)
            my_joints.joint_angles.append(self.joint6)

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
    
    def my_home_robot(self):
        req = ReadActionRequest()
        req.input.identifier = 2 # Value default home is 2 
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
    
    def execute_joints_positions(self, name:str, speed: float):
        req = ExecuteActionRequest()
        if speed:
            self.my_constrained_joints.constraint.type = JointTrajectoryConstraintType.UNSPECIFIED_JOINT_CONSTRAINT
            self.my_constrained_joints.constraint.value = speed # Velocity
        req.input.oneof_action_parameters.reach_joint_angles.append(self.my_constrained_joints)
        req.input.name = "Move Joints"
        req.input.handle.action_type = ActionType.REACH_JOINT_ANGLES
        req.input.handle.identifier = self.handle_identifier
        self.handle_identifier += 1

        rospy.loginfo("Sending to joints angles "+name)
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
            rospy.loginfo("Waiting to finish...")
        except rospy.ServiceException:
            rospy.logerr("Failed to execute joint angles")

    def call_move_joints(self,name: str, joints: list, speed=None) -> bool:
        self.joint1.value = joints[0]
        self.joint2.value = joints[1]
        self.joint3.value = joints[2]
        self.joint4.value = joints[3]
        self.joint5.value = joints[4]
        self.joint6.value = joints[5]
        self.execute_joints_positions(name, speed)
        return self.wait_for_action_end_or_abort()
            
    def main(self):
        success = self.is_initialized

        if success:
            success &= self.my_clear_faults()
            success &= self.my_subscribe_to_a_robot_notification()
            #success &= self.my_home_robot()
            
            i = 0
            positions = {
                            "lower"  :  [357.72, 84.68, 118.5, 95.13, 94.07, 271.35], # ok
                            "center" :  [353.78, 52.65, 85.31, 95.5, 106.2, 269.12], # ok
                            "upper"  :  [343.64, 33.03, 48.11, 101.47, 125.05, 267.14], # ok
                            "left"   :  [296.23, 59.73, 47.19, 145.53, 142.62, 276.72], # ok
                            "rigth"  :  [71.06, 62.51, 60.53, 38.22, 138.9, 280.37], # ok
                            "zero"   :  [0, 0, 0, 0, 0, 0]
                        }

            for _ in range(2):
                success &= self.call_move_joints('Superior', positions['center'])
                if i == 0:
                    input("Press enter to continue")
                    i += 1
                success &= self.call_move_joints('Baixo', positions['lower'])
                success &= self.call_move_joints('Superior', positions['center'])
                success &= self.call_move_joints('Centro', positions['upper'])
                success &= self.call_move_joints('Esquerda',  positions['left'])
                success &= self.call_move_joints('Direita',    positions['rigth'])
                success &= self.call_move_joints('Esquerda',  positions['left'])
                success &= self.call_move_joints('Centro', positions['upper'])

            
            success &= self.call_move_joints('Zero', positions['zero'])
                        
            if not success:
                rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = MoveJoints()
    ex.main()

