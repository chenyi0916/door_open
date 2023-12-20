#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class Handle_CoM: 

    def __init__(self):
        rospy.init_node('handle_com', anonymous=True)
        self.link1 = rospy.get_param("~link1", "link1")
        self.link2 = rospy.get_param("~link2", "link2")
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        self.init_link_inertial_info()
        self.init_marker()

    def init_link_inertial_info(self):
        robot = URDF.from_parameter_server()
        link1 = robot.link_map["robot1/link1"]
        link2= robot.link_map["robot1/link2"]
        self.link_info_origin1 = link1.inertial.origin
        self.link_info_origin2 = link2.inertial.origin

        print("link1_info", link1)
        print("====================================================")
        print("link2_info", link2)
        print("====================================================")
    

    def init_marker(self):
        marker = Marker()
        marker.header.frame_id = self.link2
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        self.marker = marker
        self.marker_pub = rospy.Publisher('com', Marker, queue_size=1)

    def transform_CoM(self):
        global xn 
        global yn 
        global zn 
        global md 

        xn = 0
        yn = 0
        zn = 0
        md = 0

        robot = URDF.from_parameter_server()
        link1 = robot.link_map["robot1/link1"]
        link2= robot.link_map["robot1/link2"]

        trans1 = self.tfBuffer.lookup_transform("robot1/link1", "robot1/link2", rospy.Time())
        point_origin1 = PointStamped()
        point_origin1.header.frame_id = link2
        point_origin1.header.stamp = rospy.Time()
        point_origin1.point.x = link2.inertial.origin.xyz[0]
        point_origin1.point.y = link2.inertial.origin.xyz[1]
        point_origin1.point.z = link2.inertial.origin.xyz[2]

        point_trans1 = tf_geo.do_transform_point(point_origin1, trans1)
        delta = point_trans1.point.z - link1.inertial.origin.xyz[2]

        print("delta = ", delta)

    # def calculate_CoM(self):

    #     self.com_x = xn / md
    #     self.com_y = yn / md
    #     self.com_z = zn / md
    #     # print('x=', self.com_x, 'y=', self.com_y, 'z=', self.com_z)
    
    def visualize_CoM(self):
        self.marker_pub.publish(self.marker)
             
if __name__ == '__main__':
    handle_com = Handle_CoM()
    rate = rospy.Rate(100)
    rospy.sleep(2.0)
    
    while not rospy.is_shutdown():
        handle_com.transform_CoM()
        # handle_com.calculate_CoM()
        # handle_com.visualize_CoM()
        rospy.get_param_names()

        rate.sleep()