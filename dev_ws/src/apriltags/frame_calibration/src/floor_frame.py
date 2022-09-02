#! /usr/bin/env python3

from multiprocessing import dummy
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray,AprilTagDetect

MAX=100
tags=[0,1,2]

class GetFrame():
    def __init__(self) -> None:
        
        ## tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.tags={'origin':[],'x':[],'y':[]}
        self.sub_tag = rospy.Subscriber("tag_detections",AprilTagDetectionArray,self.tag_cb,queue_size=1)
    


    def tag_cb(self,tag_msg):
        
        for tag in tag_msg.detection:
            if tag.id[0] not in tags:
                continue
            dum_pose= PoseStamped
            dum_pose.header=tag.header
            dum_pose.pose=tag.pose.pose.pose
            cam2base=self.tfBuffer.lookup_transform('camera_base',dum_pose.header.frame_id,dum_pose.header.stamp,rospy.Duration(0.1))
            pose_cambase=tf2_geometry_msgs.do_transform_pose(dum_pose,cam2base)

            if tag.id[0] == tags[0]:
                self.tags['origin'].append(pose_cambase.pose.position)
            elif tag.id[0] == tags[1]:
                self.tags['x'].append(pose_cambase.pose.position)
            elif tag.id[0] == tags[2]:
                self.tags['y'].append(pose_cambase.pose.position)
            
            
