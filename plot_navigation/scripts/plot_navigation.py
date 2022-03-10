#!/usr/bin/env python3

import math
import rospy
import tf2_ros
from nav_msgs.msg import Path, OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import pylab
from matplotlib.patches import Circle
from tf.transformations import euler_from_quaternion

class PlotNavigation():
    def __init__(self):
        rospy.init_node('plot_navigation')
        self.node_rate = rospy.get_param("~rate")
        self.path_topic = rospy.get_param("~path_topic")
        self.robot_front_frame = rospy.get_param("~robot_front_frame")
        self.robot_rear_frame = rospy.get_param("~robot_rear_frame")
        self.ref_frame = rospy.get_param("~reference_frame")
        self.plot_title = rospy.get_param("~plot_title")
        self.global_planner = rospy.get_param("/move_base/base_global_planner")
        self.local_planner = rospy.get_param("/move_base/base_local_planner")
        # trim e.g. global_planner/GlobalPlanner to global_planner
        self.global_planner = self.global_planner.split(sep='/')[0] 
        self.local_planner = self.local_planner.split(sep='/')[0] 
        self.xy_goal_tol = rospy.get_param("~xy_goal_tolererance")
        self.global_path_x = []
        self.global_path_y = []
        self.global_path_yaw = []
        self.robot_front_x = []
        self.robot_front_y = []
        self.robot_front_yaw = []
        self.robot_rear_x = []
        self.robot_rear_y = []
        self.robot_rear_yaw = []
        self.map_available = False
        self.global_path_available = False
        self.robot_pose_available = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(self.path_topic, Path, callback=self.global_path_cb)
        rospy.Subscriber("/map", OccupancyGrid, callback=self.occ_grid_cb)
        rospy.Timer(rospy.Duration(0.05), self.robot_pose_cb)

        self.filepath = __file__.split(sep='.')[0]
        self.fig_plot, self.ax_plot = plt.subplots(nrows=1, ncols=1)
        self.ax_plot.set_xlim([0.0, 5.0])
        self.ax_plot.set_ylim([-0.5, 0.5])
        rospy.on_shutdown(self.save_plot)

    def get_quaternion_yaw(self, quaternion):
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(quaternion_list)
        return yaw

    def global_path_cb(self, global_path_msg):
        # Sometimes, several global paths are determined consecutively, 
        # while the robot is approaching the goal.
        # But we want to plot only the first global path.
        if not self.global_path_available:
            for pose_stamped in global_path_msg.poses:
                self.global_path_x.append(pose_stamped.pose.position.x)
                self.global_path_y.append(pose_stamped.pose.position.y)
                self.global_path_yaw.append(self.get_quaternion_yaw(pose_stamped.pose.orientation))
            self.global_path_available = True
        
    def occ_grid_cb(self, occ_grid_msg):
        self.map_width_px = occ_grid_msg.info.width
        self.map_height_px = occ_grid_msg.info.height
        self.map_res = occ_grid_msg.info.resolution
        self.map_origin = np.array([occ_grid_msg.info.origin.position.x, occ_grid_msg.info.origin.position.y])
        self.map_data = np.array(occ_grid_msg.data).reshape((occ_grid_msg.info.height, occ_grid_msg.info.width))
        self.map_available = True
    
    def robot_pose_cb(self, timer):
        try:
            trans_front = self.tf_buffer.lookup_transform(self.ref_frame, self.robot_front_frame, rospy.Time.now())
            trans_rear = self.tf_buffer.lookup_transform(self.ref_frame, self.robot_rear_frame, rospy.Time.now())
            
            self.robot_front_x.append(trans_front.transform.translation.x)
            self.robot_front_y.append(trans_front.transform.translation.y)
            self.robot_front_yaw.append(self.get_quaternion_yaw(trans_front.transform.rotation))

            self.robot_rear_x.append(trans_rear.transform.translation.x)
            self.robot_rear_y.append(trans_rear.transform.translation.y)
            self.robot_rear_yaw.append(self.get_quaternion_yaw(trans_rear.transform.rotation))
            self.robot_pose_available = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def plot_global_path(self):
        if self.global_path_available:
            self.ax_plot.plot(self.global_path_x, self.global_path_y, '-b', label="initial global path", linewidth=1)
            self.ax_plot.plot(self.global_path_x[0], self.global_path_y[0], 'or', markersize=6, label="start global path (front carriage)", linewidth=1)
            self.ax_plot.plot(self.global_path_x[-1], self.global_path_y[-1], 'xg', markersize=6, label="goal global path (front carriage)", linewidth=1)
            pose_global_goal_f = [self.global_path_x[-1], self.global_path_y[-1], math.cos(self.global_path_yaw[-1]), math.sin(self.global_path_yaw[-1])] 
            # pose_global_start_f = [self.global_path_x[0], self.global_path_y[0], math.cos(self.global_path_yaw[0]), math.sin(self.global_path_yaw[0])] 
            self.ax_plot.quiver(pose_global_goal_f[0], pose_global_goal_f[1], pose_global_goal_f[2], pose_global_goal_f[3], width=0.003, headwidth=6, color='b')
            # self.ax.quiver(pose_global_start_f[0], pose_global_start_f[1], pose_global_start_f[2], pose_global_start_f[3], width=0.003, headwidth=6, color='b')
            self.ax_plot.add_patch(Circle([self.global_path_x[-1], self.global_path_y[-1]], self.xy_goal_tol, fc='none', ec='g', ls='--', label="goal tolerance"))

    def plot_robot_pose(self):
        if self.robot_pose_available:
            self.ax_plot.plot(self.robot_front_x, self.robot_front_y, '--c', label='front carriage', linewidth=1)  
            self.ax_plot.plot(self.robot_rear_x, self.robot_rear_y, ':c', label='rear carriage', linewidth=1)
            pose_start_f = [self.robot_front_x[0], self.robot_front_y[0], math.cos(self.robot_front_yaw[0]), math.sin(self.robot_front_yaw[0])] 
            pose_start_r = [self.robot_rear_x[0], self.robot_rear_y[0], math.cos(self.robot_rear_yaw[0]), math.sin(self.robot_rear_yaw[0])] 
            pose_end_f = [self.robot_front_x[-1], self.robot_front_y[-1], math.cos(self.robot_front_yaw[-1]), math.sin(self.robot_front_yaw[-1])] 
            pose_end_r = [self.robot_rear_x[-1], self.robot_rear_y[-1], math.cos(self.robot_rear_yaw[-1]), math.sin(self.robot_rear_yaw[-1])] 
            self.ax_plot.quiver(pose_start_f[0], pose_start_f[1], pose_start_f[2], pose_start_f[3], width=0.003, headwidth=6, color='r')
            self.ax_plot.quiver(pose_start_r[0], pose_start_r[1], pose_start_r[2], pose_start_r[3], width=0.003, headwidth=6, color='r')
            self.ax_plot.quiver(pose_end_f[0], pose_end_f[1], pose_end_f[2], pose_end_f[3], width=0.003, headwidth=6, color='g')
            self.ax_plot.quiver(pose_end_r[0], pose_end_r[1], pose_end_r[2], pose_end_r[3], width=0.003, headwidth=6, color='g')
            # add dummy point to create legend for robot orientation 
            self.ax_plot.scatter([], [], c='red', marker=u"$\u2212\!\u2192$", s=200, label='robot start orientation')
            self.ax_plot.scatter([], [], c='green', marker=u"$\u2212\!\u2192$", s=200, label='robot stop orientation')
            self.ax_plot.scatter([], [], c='blue', marker=u"$\u2212\!\u2192$", s=200, label='given goal orientation')

    def plot_obstacles(self):
        if self.map_available:
            # create mesh grid containing x and y coordinates in meters of each cell in the occupancy grid map
            map_width_m = self.map_width_px * self.map_res
            map_height_m = self.map_height_px * self.map_res
            x_m = np.linspace(self.map_origin[0], map_width_m + self.map_origin[0], self.map_width_px)
            y_m = np.linspace(self.map_origin[1], map_height_m + self.map_origin[1], self.map_height_px)
            xx_m, yy_m = np.meshgrid(x_m, y_m)

            # obtain current boundaries of plot area in meters
            xlim_plot_m = self.ax_plot.get_xlim()
            ylim_plot_m = self.ax_plot.get_ylim()
            
            # create True/False mask for meshgrid (True: xy position within plot area, False: otherwise)
            xx_mask = np.logical_and(xx_m > xlim_plot_m[0], xx_m < xlim_plot_m[1])
            yy_mask = np.logical_and(yy_m > ylim_plot_m[0], yy_m < ylim_plot_m[1])
            xy_mask = np.logical_and(xx_mask, yy_mask)

            # apply mask to mesh grid and occupancy grid --> crop
            xx_crop_m = xx_m[xy_mask]
            yy_crop_m = yy_m[xy_mask]
            map_data_crop = self.map_data[xy_mask]

            # identify all obstacles in map and their corresponding xy-position in map-frame
            mask_obstacle = map_data_crop >= 100
            x_obstacle = xx_crop_m[mask_obstacle]
            y_obstacle = yy_crop_m[mask_obstacle]

            # plot all obstacles as grey circles in background
            self.ax_plot.scatter(x_obstacle, y_obstacle, marker='o', color='lightgrey', label='obstacle', zorder=0)

    def start_plotting(self):
        rate = rospy.Rate(self.node_rate)
        while not rospy.is_shutdown():
            self.ax_plot.clear()            
            self.plot_global_path()
            self.plot_robot_pose()
            self.plot_obstacles()           
            plt.draw()
            plt.pause(0.25)
            rate.sleep() 

    def export_legend(self, legend, filename="legend.png", expand=[-5,-5,5,5]):
        fig  = legend.figure
        fig.canvas.draw()
        bbox  = legend.get_window_extent()
        bbox = bbox.from_extents(*(bbox.extents + np.array(expand)))
        bbox = bbox.transformed(fig.dpi_scale_trans.inverted())
        fig.savefig(filename, dpi=300, bbox_inches=bbox)
            
    def save_plot(self):
        self.ax_plot.clear()            
        self.plot_global_path()
        self.plot_robot_pose()
        self.plot_obstacles()           

        self.ax_plot.set_title('Scenario:' + self.plot_title + ', GP:' + self.global_planner + ', LP: ' + self.local_planner, loc='left')
        self.ax_plot.set_xlabel('position x (m)')
        self.ax_plot.set_ylabel('position y (m)')
        # save figure containing the actual plot
        self.fig_plot.savefig(self.filepath, dpi=(300), bbox_inches='tight')
        rospy.loginfo("saved figure as "+self.filepath+".png")

        # # Since we want the legend plotted on a separate figure, do not call the following line:
        # # self.ax.legend(loc='best')    
        # # Instead create a second figure for the legend (here with pylab)
        # fig_legend = pylab.figure(figsize = (1.5,1.3))
        # # produce a legend for the objects in the original figure
        # pylab.fig_legend(*self.ax_plot.get_legend_handles_labels(), loc = 'upper left')
        # fig_legend.savefig(self.filepath+"_legend.png")
        # rospy.loginfo("saved legend as "+self.filepath+"_legend.png")
        fig_legend, ax_legend = plt.subplots(nrows=1, ncols=1, figsize=(1,1))

        # fig_leg = plt.figure(figsize=(3, 3))
        # ax_leg = fig_leg.add_subplot(111)
        # add the legend from the previous axes
        ax_legend.legend(*self.ax_plot.get_legend_handles_labels(), loc='center')
        # hide the axes frame and the x/y labels
        ax_legend.axis('off')
        fig_legend.savefig(self.filepath+"_legend.png", dpi=(300), bbox_inches='tight')
        # rospy.loginfo("saved legend as "+self.filepath+"_legend.png")


        pass


if __name__ == '__main__':
    plot = PlotNavigation()
    plot.start_plotting()