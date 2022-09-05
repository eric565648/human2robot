#! /usr/bin/env python3

from multiprocessing import dummy
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray,AprilTagDetection
import general_robotics_toolbox as rox

MAX=50
tags=[0,1,2]

class GetFrame():
    def __init__(self) -> None:
        
        ## tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        ## tf broadcaster
        self.tfbroadcaster = tf2_ros.StaticTransformBroadcaster()

        ## subscribe tag detections
        self.tags={'origin':[],'x':[],'y':[]}
        self.pub_tf_flag=False
        self.sub_tag = rospy.Subscriber("tag_detections",AprilTagDetectionArray,self.tag_cb,queue_size=1)

        ## timer for tf pub
        self.timer = rospy.Timer(rospy.Duration(0.1),self.tf_timer_cb)

    def tag_cb(self,tag_msg):
        
        if self.pub_tf_flag:
            return

        for tag in tag_msg.detections:
            if tag.id[0] not in tags:
                continue
            dum_pose= tf2_geometry_msgs.PoseStamped()
            dum_pose.header=tag.pose.header
            dum_pose.pose=tag.pose.pose.pose
            cam2base=self.tfBuffer.lookup_transform('camera_base',dum_pose.header.frame_id,dum_pose.header.stamp,rospy.Duration(0.1))
            pose_cambase=tf2_geometry_msgs.do_transform_pose(dum_pose,cam2base)

            if tag.id[0] == tags[0]:
                self.tags['origin'].append(np.array([pose_cambase.pose.position.x,\
                    pose_cambase.pose.position.y,pose_cambase.pose.position.z]))
            elif tag.id[0] == tags[1]:
                self.tags['x'].append(np.array([pose_cambase.pose.position.x,\
                    pose_cambase.pose.position.y,pose_cambase.pose.position.z]))
            elif tag.id[0] == tags[2]:
                self.tags['y'].append(np.array([pose_cambase.pose.position.x,\
                    pose_cambase.pose.position.y,pose_cambase.pose.position.z]))
        
        total_num = np.mean([len(self.tags['origin']),len(self.tags['x']),len(self.tags['y'])])
        progress = round(total_num/MAX*100,2)
        rospy.loginfo('Get floor frame: '+str(progress)+' %')
        
        if len(self.tags['origin'])>=MAX and len(self.tags['x'])>=MAX and len(self.tags['y'])>=MAX:
            self.pub_tf_flag=True
            self.sub_tag.unregister()
    
    def tf_timer_cb(self,event):
        
        if not self.pub_tf_flag:
            return
        
        origin_p = np.mean(self.tags['origin'],0)
        x_p = np.mean(self.tags['x'],0)
        y_p = np.mean(self.tags['y'],0)
        Rx = (x_p-origin_p)/np.linalg.norm(x_p-origin_p)
        Ry = (y_p-origin_p)/np.linalg.norm(y_p-origin_p)
        Ry = Ry-np.dot(Ry,Rx)*Rx
        Ry = Ry/np.linalg.norm(Ry)
        Rz = np.cross(Rx,Ry)/np.linalg.norm(np.cross(Rx,Ry))

        T_cam2floor = rox.Transform(np.vstack((Rx,Ry,Rz)),origin_p)
        T_floor2cam = T_cam2floor.inv()

        static_tfstamped=TransformStamped()
        ## cam to floor
        static_tfstamped.header.stamp = rospy.Time.now()
        static_tfstamped.header.frame_id = 'camera_base'
        static_tfstamped.child_frame_id = 'floor'
        static_tfstamped.transform.translation.x = T_cam2floor.p[0]
        static_tfstamped.transform.translation.y = T_cam2floor.p[1]
        static_tfstamped.transform.translation.z = T_cam2floor.p[2]
        quat = rox.R2q(T_cam2floor.R)
        static_tfstamped.transform.rotation.w = quat[0]
        static_tfstamped.transform.rotation.x = quat[1]
        static_tfstamped.transform.rotation.y = quat[2]
        static_tfstamped.transform.rotation.z = quat[3]
        self.tfbroadcaster.sendTransform(static_tfstamped)
        ## floor to cam
        static_tfstamped.header.frame_id = 'floor'
        static_tfstamped.child_frame_id = 'camera_base'
        static_tfstamped.transform.translation.x = T_floor2cam.p[0]
        static_tfstamped.transform.translation.y = T_floor2cam.p[1]
        static_tfstamped.transform.translation.z = T_floor2cam.p[2]
        quat = rox.R2q(T_floor2cam.R)
        static_tfstamped.transform.rotation.w = quat[0]
        static_tfstamped.transform.rotation.x = quat[1]
        static_tfstamped.transform.rotation.y = quat[2]
        static_tfstamped.transform.rotation.z = quat[3]
        self.tfbroadcaster.sendTransform(static_tfstamped)
    
    def on_shutdown(self):
        pass

if __name__=='__main__':
    rospy.init_node('floor_frame')
    node=GetFrame()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
            
            
