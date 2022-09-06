#! /usr/bin/env python3

from copy import deepcopy
from yaml import Mark
import rospy
import numpy as np
import tf2_ros
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import PoseStamped

class JointFilter():
    def __init__(self) -> None:

        ##### variables
        self.follower_num=0
        self.leader_num=1
        self.base_frame_id='floor'
        self.fol_handr_p = None
        self.fol_handl_p = None
        self.lea_handr_p = None
        self.lea_handl_p = None

        # marker setup
        self.marker_fr = Marker()
        self.marker_fr.header.frame_id = self.base_frame_id
        self.marker_fr.type = self.marker_fr.SPHERE
        self.marker_fr.id=10
        self.marker_fr.action = 0
        self.marker_fr.scale.x = 0.05
        self.marker_fr.scale.y = 0.05
        self.marker_fr.scale.z = 0.05
        self.marker_fr.color.r = 1
        self.marker_fr.color.g = 0
        self.marker_fr.color.a = 1
        self.marker_fl = deepcopy(self.marker_fr)
        self.marker_fl.id=11
        self.marker_lr = deepcopy(self.marker_fr)
        self.marker_lr.id=20
        self.marker_lr.color.r = 0
        self.marker_lr.color.g = 1
        self.marker_ll = deepcopy(self.marker_lr)
        self.marker_ll.id=21
        

        ## tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        ## tf broadcaster
        self.tfbroadcaster = tf2_ros.StaticTransformBroadcaster()

        # publisher
        self.joint_pub = rospy.Publisher('/body_tracking_smooth_data',MarkerArray,queue_size=1)

        # subscriber
        self.joint_sub = rospy.Subscriber('/body_tracking_data',MarkerArray,self.body_marker_cb,queue_size=1)
    
    def body_marker_cb(self,msg):

        msg_header = msg.markers[0].header
        this_stamp = msg_header.stamp

        ## hand frame
        fol_hand_r = 'Hand_right'+str(self.follower_num)
        fol_hand_l = 'Hand_left'+str(self.follower_num)
        lea_hand_r = 'Hand_right'+str(self.leader_num)
        lea_hand_l = 'Hand_left'+str(self.leader_num)
        ## get tf
        try:
            fol_hand_r_tf=self.tfBuffer.lookup_transform(self.base_frame_id,fol_hand_r,this_stamp,rospy.Duration(0.1))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            pass
        try:
            fol_hand_l_tf=self.tfBuffer.lookup_transform(self.base_frame_id,fol_hand_l,this_stamp,rospy.Duration(0.1))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            pass
        try:
            lea_hand_r_tf=self.tfBuffer.lookup_transform(self.base_frame_id,lea_hand_r,this_stamp,rospy.Duration(0.1))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            pass
        try:
            lea_hand_l_tf=self.tfBuffer.lookup_transform(self.base_frame_id,lea_hand_l,this_stamp,rospy.Duration(0.1))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            pass
        
        ## filter the joint
        self.joint_filter(fol_hand_r_tf.transform.translation,fol_hand_l_tf.transform.translation,\
            lea_hand_r_tf.transform.translation,lea_hand_l_tf.transform.translation)
        
        ## broadcast
        fol_handr_smooth_tf = deepcopy(fol_hand_r_tf)
        fol_handr_smooth_tf.child_frame_id='Hand_right_smooth'+str(self.follower_num)
        fol_handr_smooth_tf.transform.translation.x=self.fol_handr_p[0]
        fol_handr_smooth_tf.transform.translation.y=self.fol_handr_p[1]
        fol_handr_smooth_tf.transform.translation.z=self.fol_handr_p[2]
        fol_handl_smooth_tf = deepcopy(fol_hand_l_tf)
        fol_handl_smooth_tf.child_frame_id='Hand_left_smooth'+str(self.follower_num)
        fol_handl_smooth_tf.transform.translation.x=self.fol_handl_p[0]
        fol_handl_smooth_tf.transform.translation.y=self.fol_handl_p[1]
        fol_handl_smooth_tf.transform.translation.z=self.fol_handl_p[2]
        lea_handr_smooth_tf = deepcopy(lea_hand_r_tf)
        lea_handr_smooth_tf.child_frame_id='Hand_right_smooth'+str(self.leader_num)
        lea_handr_smooth_tf.transform.translation.x=self.lea_handr_p[0]
        lea_handr_smooth_tf.transform.translation.y=self.lea_handr_p[1]
        lea_handr_smooth_tf.transform.translation.z=self.lea_handr_p[2]
        lea_handl_smooth_tf = deepcopy(lea_hand_l_tf)
        lea_handl_smooth_tf.child_frame_id='Hand_left_smooth'+str(self.leader_num)
        lea_handl_smooth_tf.transform.translation.x=self.lea_handl_p[0]
        lea_handl_smooth_tf.transform.translation.y=self.lea_handl_p[1]
        lea_handl_smooth_tf.transform.translation.z=self.lea_handl_p[2]
        self.tfbroadcaster.sendTransform(fol_handr_smooth_tf)
        self.tfbroadcaster.sendTransform(fol_handl_smooth_tf)
        self.tfbroadcaster.sendTransform(lea_handr_smooth_tf)
        self.tfbroadcaster.sendTransform(lea_handl_smooth_tf)
        ## marker publish
        vmarray = MarkerArray()
        self.marker_fr.pose.position = fol_handr_smooth_tf.transform.translation
        self.marker_fl.pose.position = fol_handl_smooth_tf.transform.translation
        self.marker_lr.pose.position = lea_handr_smooth_tf.transform.translation
        self.marker_ll.pose.position = lea_handl_smooth_tf.transform.translation
        self.marker_fr.header.stamp = fol_handr_smooth_tf.header.stamp
        self.marker_fl.header.stamp = fol_handl_smooth_tf.header.stamp
        self.marker_lr.header.stamp = lea_handr_smooth_tf.header.stamp
        self.marker_ll.header.stamp = lea_handl_smooth_tf.header.stamp
        vmarray.markers.append(self.marker_fr)
        vmarray.markers.append(self.marker_fl)
        vmarray.markers.append(self.marker_lr)
        vmarray.markers.append(self.marker_ll)
        self.joint_pub.publish(vmarray)

    def joint_filter(self,folhr,folhl,leahr,leahl):

        self.fol_handr_p = [folhr.translation.x,folhr.translation.y,folhr.translation.z]
        self.fol_handl_p = [folhl.translation.x,folhl.translation.y,folhl.translation.z]
        self.lea_handr_p = [leahr.translation.x,leahr.translation.y,leahr.translation.z]
        self.lea_handl_p = [leahl.translation.x,leahl.translation.y,leahl.translation.z]

    def on_shutdown(self):
        pass

if __name__=='__main__':
    rospy.init_node('joint_filter')
    node=JointFilter()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()