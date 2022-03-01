#!/usr/bin/env python3

from cProfile import label
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from tf.transformations import euler_from_quaternion

class PlotNavigation():
    def __init__(self):
        rospy.init_node('plot_navigation')
        self.node_rate = rospy.get_param("~rate")
        self.path_topic = rospy.get_param("~path_topic")
        self.robot_front_frame = rospy.get_param("~robot_front_frame")
        self.robot_rear_frame = rospy.get_param("~robot_rear_frame")
        self.ref_frame = rospy.get_param("~reference_frame")
        self.w_plt = 0.5
        self.h_plt = 0.25
        self.global_path_x = []
        self.global_path_y = []
        self.robot_front_x = []
        self.robot_front_y = []
        self.robot_front_yaw = []
        self.robot_rear_x = []
        self.robot_rear_y = []
        self.robot_rear_yaw = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(self.path_topic, Path, callback=self.global_path_cb)
        rospy.Timer(rospy.Duration(0.05), self.robot_pose_cb)

        self.filepath = __file__.split(sep='.')[0]+".png"
        self.fig, self.ax = plt.subplots(nrows=1, ncols=1)
        self.ax.set_title('Global and actual path between start and goal')
        self.ax.set_xlabel('position x (m)')
        self.ax.set_ylabel('position y (m)')
        self.ax.set_aspect('equal')

        rospy.on_shutdown(self.save_plot)

    def global_path_cb(self, global_path):
        for pose_stamped in global_path.poses:
            self.global_path_x.append(pose_stamped.pose.position.x)
            self.global_path_y.append(pose_stamped.pose.position.y)
        
    def get_yaw(self, trans):
        quaternion = trans.transform.rotation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(quaternion_list)
        return yaw

    def robot_pose_cb(self, timer):
        try:
            trans_front = self.tf_buffer.lookup_transform(self.ref_frame, self.robot_front_frame, rospy.Time.now())
            trans_rear = self.tf_buffer.lookup_transform(self.ref_frame, self.robot_rear_frame, rospy.Time.now())
            
            self.robot_front_x.append(trans_front.transform.translation.x)
            self.robot_front_y.append(trans_front.transform.translation.y)
            self.robot_front_yaw.append(self.get_yaw(trans_front))

            self.robot_rear_x.append(trans_rear.transform.translation.x)
            self.robot_rear_y.append(trans_rear.transform.translation.y)
            self.robot_rear_yaw.append(self.get_yaw(trans_rear))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def start_plotting(self):
        rate = rospy.Rate(self.node_rate)
        while not rospy.is_shutdown():
            self.ax.plot(self.global_path_x, self.global_path_y, '-b', label="initial global path", linewidth=1)
            self.ax.plot(self.robot_front_x, self.robot_front_y, '-c', label='actual front position', linewidth=1)
            self.ax.plot(self.robot_rear_x, self.robot_rear_y, ':c', label='actual rear position', linewidth=1)
            plt.draw()
            plt.pause(0.05)
            rate.sleep() 
            
    def save_plot(self):
        self.ax.clear()
        self.ax.plot(self.global_path_x, self.global_path_y, '-b', label="initial global path", linewidth=1)
        self.ax.plot(self.robot_front_x, self.robot_front_y, '-c', label='front carriage', linewidth=1)  
        self.ax.plot(self.robot_rear_x, self.robot_rear_y, ':c', label='rear carriage', linewidth=1)
        self.ax.plot(self.global_path_x[0], self.global_path_y[0], 'om', markersize=3, label="front start", linewidth=1)
        self.ax.plot(self.global_path_x[-1], self.global_path_y[-1], 'og', markersize=3, label="front goal", linewidth=1)
        pose_start_f = [self.robot_front_x[0], self.robot_front_y[0], math.cos(self.robot_front_yaw[0]), math.sin(self.robot_front_yaw[0])] 
        pose_start_r = [self.robot_rear_x[0], self.robot_rear_y[0], math.cos(self.robot_rear_yaw[0]), math.sin(self.robot_rear_yaw[0])] 
        pose_end_f = [self.robot_front_x[-1], self.robot_front_y[-1], math.cos(self.robot_front_yaw[-1]), math.sin(self.robot_front_yaw[-1])] 
        pose_end_r = [self.robot_rear_x[-1], self.robot_rear_y[-1], math.cos(self.robot_rear_yaw[-1]), math.sin(self.robot_rear_yaw[-1])] 
        q_start = self.ax.quiver(pose_start_f[0], pose_start_f[1], pose_start_f[2], pose_start_f[3], width=0.003, headwidth=6, color='magenta')
        self.ax.quiver(pose_start_r[0], pose_start_r[1], pose_start_r[2], pose_start_r[3], width=0.003, headwidth=6, color='magenta')
        q_end = self.ax.quiver(pose_end_f[0], pose_end_f[1], pose_end_f[2], pose_end_f[3], width=0.003, headwidth=6, color='green')
        self.ax.quiver(pose_end_r[0], pose_end_r[1], pose_end_r[2], pose_end_r[3], width=0.003, headwidth=6, color='green')
        # self.ax.quiverkey(q_start, 0.8, 0.2, 1, 'start', labelpos='E')
        # self.ax.quiverkey(q_end, 0.8, 0.1, 1, 'end', labelpos='E')      
        self.ax.legend(loc='best')
        plt.savefig(self.filepath,dpi=(300), bbox_inches='tight')
        rospy.loginfo("saved figure as "+self.filepath)



if __name__ == '__main__':
    plot = PlotNavigation()
    plot.start_plotting()
