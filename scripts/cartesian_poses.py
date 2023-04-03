#!/usr/bin/env python3

import rospy
import time
import json

from kortex_driver.srv import *
from kortex_driver.msg import *

class CartesianPoses:
    def __init__(self):
        try:
            rospy.init_node("cartesian_poses")

            self.all_notifs_succeeded = True

            self.robot_name = "my_gen3_lite"
            rospy.loginfo("Using robot_name " + self.robot_name)

            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.action_topic_callback)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            # Init atributes to cartesian configs
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.08 # m/s
            my_cartesian_speed.orientation = 5  # deg/s
            
            self.my_constrained_pose = ConstrainedPose()
            self.my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
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
    
    def my_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)
    
    def get_cartesian_pose(self):
        req = ReadActionRequest()
        rospy.logwarn(str(req.input))
    
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
    
    def execute_cartesian_pose(self, pose_name: str):
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(self.my_constrained_pose)
        req.input.name = pose_name
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = self.handle_identifier
        self.handle_identifier += 1

        rospy.loginfo("Sending " + pose_name + "...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
            rospy.loginfo("Waiting for "+ pose_name +" to finish...")
        except rospy.ServiceException:
            rospy.logerr("Failed to send " + pose_name)

    
    def call_pose(self, pose_name="pose_zero", poses=[0, 0, 0.9, 0, 0, 0]):
        for index in range(3):
                poses[index] = poses[index]/100
        
        cartesian = Pose(poses[0], poses[1], poses[2], poses[3], poses[4], poses[5])
        self.my_constrained_pose.target_pose = cartesian
        self.execute_cartesian_pose(pose_name)
        return self.wait_for_action_end_or_abort()

            
    def main(self):
        success = bool()
        if self.is_initialized:
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.my_clear_faults()

            #*******************************************************************************
            # Start the example from the Home position
            #success &= self.my_home_robot()
            #*******************************************************************************

            self.get_cartesian_pose()
            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.my_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.my_subscribe_to_a_robot_notification()
            # juntas: 328.69, 66.75, 95.95, 291.50, 256.66, 74.99

            poses_yp = {
                        "Pose 01": [4.5, 0.1, 40.7, 129.7, -0.3, 88.1],
                        "Pose 02": [5.8, 31.8, 41, 128.8, -0.3, 73.9]
                    }
            
            poses_yn = {
                        "Pose 01": [4.5, -0.3, 40.7, 129.9, 1.2, 87.4],
                        "Pose 02":[4.5, -29.9, 41, 128.3, -2.4, 108.1]
                    }
            input("Press enter to continue")
            self.call_pose("Pose 02 +", poses_yp["Pose 02"])
            self.call_pose("Pose 01 +", poses_yp["Pose 01"])
            self.call_pose("Pose 02 -", poses_yn["Pose 02"])
            self.call_pose("Pose 01 -", poses_yn["Pose 01"])
            

            # self.call_pose("Pose 03", poses["Pose 03"])
            # input("Press enter to continue...")
            
            # for _ in range(5):
            #     for key in poses.keys():
            #         if key != "Pose 01" and key != "Pose 02":
            #             success &= self.call_pose(key, poses[key])
                            
            if not success:
                rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = CartesianPoses()
    ex.main()

