#!/usr/bin/env python3

import rospy
import time

from kortex_driver.srv import Base_ClearFaults, ReadAction, ReadActionRequest, ExecuteAction, ExecuteActionRequest, SetCartesianReferenceFrame
from kortex_driver.srv import SetCartesianReferenceFrameRequest, OnNotificationActionTopic, OnNotificationActionTopicRequest

from kortex_driver.msg import ActionNotification, ActionEvent, CartesianReferenceFrame, CartesianSpeed, ConstrainedPose, ActionType, Pose

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
            my_cartesian_speed.translation = 0.1 # m/s
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
            self.set_cartesian_reference_frame.call(req.input)
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
    
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

    
    def call_pose(self, pose_name="pose_zero", x=0.374, y=0.081, z=0.450, theta_x=-57.6, theta_y=91.1, theta_z=2.3):
        cartesian = Pose(x, y, z, theta_x, theta_y, theta_z)
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
            success &= self.my_home_robot()
            #*******************************************************************************

            self.get_cartesian_pose()
            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.my_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.my_subscribe_to_a_robot_notification()

            #success = self.call_pose("Ponto A1", x=0.28, y=-0.28, z=0.45, theta_x=130.0, theta_y=0.0, theta_z=0.0)
            """ for _ in range(2):
                success = self.call_pose("Ponto A2", x=0.20, y=-0.25, z=0.45, theta_x=130.0, theta_y=0.0, theta_z=-25.0)
                success = self.call_pose("Ponto C", x=00.0, y=-0.35, z=0.35, theta_x=130.0, theta_y=0.0, theta_z=0.0)
                success = self.call_pose("Ponto B", x=-0.28, y=-0.28, z=0.45, theta_x=130.0, theta_y=0.0, theta_z=25.0)
                success = self.call_pose("Ponto C", x=00.0, y=-0.35, z=0.40, theta_x=130.0, theta_y=0.0, theta_z=0.0) """
            
                        
            #success = self.call_pose("pose 2", x=0.27, y=-0.27, z=0.37, theta_x=0.0, theta_y=175.0, theta_z=0.0)
            success &= self.my_home_robot(3)
            
            
            if not success:
                rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = CartesianPoses()
    ex.main()

