#!/usr/bin/env python3

from rospy import init_node, loginfo, Subscriber
from kortex_driver.msg import BaseCyclic_Feedback
import png
import pyrealsense2 as rs
import json
import logging
logging.basicConfig(level=logging.INFO)
import numpy as np
import cv2
import time
import os

class Camera:
    def __init__(self, folder_destiny: str, record_length: int):
        try:
            # ROBOT CONFIG
            init_node("camera")
            self.robot_name = "my_gen3_lite"
            loginfo("Using robot_name " + self.robot_name)

            self.base_feedback_topic_sub = Subscriber("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback, self.base_feedback)

            self.x = None
            self.y = None
            self.z = None
            self.roll = None
            self.pitch = None
            self.yaw = None

            self.is_initialized = True

            # CAMERA CONFIG
            self.folder = folder_destiny
            self.record_length = record_length
            self.FileName=0
            self.make_directories(self.folder)
            
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
            
            # Start pipeline
            profile = self.pipeline.start(self.config)
            self.frames = self.pipeline.wait_for_frames()
            self.color_frame = self.frames.get_color_frame()

            # Color Intrinsics 
            intr = self.color_frame.profile.as_video_stream_profile().intrinsics
            camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                                'ppx': intr.ppx, 'ppy': intr.ppy,
                                'height': intr.height, 'width': intr.width,
                                'distortion': intr.coeffs,
                                'depth_scale':profile.get_device().first_depth_sensor().get_depth_scale()
            }
            with open(self.folder+'intrinsics.json', 'w') as fp:
                json.dump(camera_parameters, fp)
            
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)

        except Exception as e:
            loginfo("Error " + str(e))
            self.is_initialized = False
    
    def base_feedback(self, base: BaseCyclic_Feedback):
        self.x = base.base.tool_pose_x
        self.y = base.base.tool_pose_y
        self.z = base.base.tool_pose_z
        self.roll = base.base.tool_pose_theta_x
        self.pitch = base.base.tool_pose_theta_y
        self.yaw = base.base.tool_pose_theta_z
    
    def save_current_position(self, id) -> bool:
        if self.x and self.y and self.z and self.roll and self.pitch and self.yaw:
            with open(self.folder+"robot/"+str(id)+'.txt', 'w') as file:
                file.write(f'{self.x}, {self.y}, {self.z}, {self.roll}, {self.pitch}, {self.yaw}')
            return True
        return False
    
    @staticmethod
    def make_directories(folder):
        if not os.path.exists(folder+"JPEGImages/"):
            os.makedirs(folder+"JPEGImages/")
        if not os.path.exists(folder+"depth/"):
            os.makedirs(folder+"depth/")
        if not os.path.exists(folder+"robot/"):
            os.makedirs(folder+"robot/")
            
    def main(self):
        success = self.is_initialized

        if success:
            T_start = time.time()
            while time.time() - T_start > self.record_length + 5:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)

                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue
                
                d = np.asanyarray(aligned_depth_frame.get_data())
                c = np.asanyarray(color_frame.get_data())
        
                # Visualize count down
                if time.time() - T_start > 5:
                    if self.save_current_position(self.FileName):
                        filecad= self.folder+"JPEGImages/%s.jpg" % self.FileName
                        filedepth= self.folder+"depth/%s.png" % self.FileName
                        cv2.imwrite(filecad, c)
                        
                        with open(filedepth, 'wb') as f:
                            writer = png.Writer(width=d.shape[1], height=d.shape[0],
                                                bitdepth=16, greyscale=True)
                            zgray2list = d.tolist()
                            writer.write(f, zgray2list)
                        self.FileName+=1

                    if time.time() -T_start < 5:
                        cv2.putText(c,str(5-int(time.time() -T_start)),(240,320), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 4,(0,0,255),2,cv2.LINE_AA)
                    if time.time() -T_start > self.record_length:
                        cv2.putText(c,str(self.record_length+5-int(time.time()-T_start)),(240,320), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 4,(0,0,255),2,cv2.LINE_AA)
                    cv2.imshow('COLOR IMAGE',c)

                    # press q to quit the program
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            self.pipeline.stop()
            # Release everything if job is finished
            cv2.destroyAllWindows()

if __name__ == "__main__":
    ex = Camera(r"LINEMOD/urna/", 35)
    ex.main()

