#ifndef SWITCH_NAV_BASE_H
#define SWITCH_NAV_BASE_H

#include <ros/time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <base/Angle.h>

#include <ros/console.h>

class SwitchNavBase
{
    public:
        SwitchNavBase();
        SwitchNavBase(ros::NodeHandle* nh);
        ~SwitchNavBase();

    private:
        void create_sub_pub_();
        void cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr& msg);
        void body_angle_callback_(const base::Angle::ConstPtr &msg);
        void global_goal_callback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void goal_status_callback_(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
        void tf_callback(const ros::TimerEvent& e);
        uint32_t seq_tf_;
        uint8_t goal_status_;
        uint8_t goal_success_ctr_;
        
        bool backward_motion_;
        bool rear_sub_goal_active_;
        bool front_sub_goal_active_;
        bool main_goal_reached_;
        bool switch_sub_goal_permitted;
        bool goal_success_now_;
        double wheelbase_theta_zero_;
        std::string frame_id_front_;
        std::string frame_id_rear_;
        std::string frame_id_output_;
        std::string frame_id_map_;
        std::string active_goal_id_;
        uint8_t active_goal_status_;
        ros::NodeHandle* nh_;
        ros::Timer tf_timer_;
        ros::Time time_at_switch_sub_goal_;
        ros::Duration time_delay_switch_;
        ros::Subscriber cmd_vel_subscriber_;
        ros::Subscriber body_angle_subscriber_;
        ros::Subscriber global_goal_subscriber_;
        ros::Subscriber goal_status_subscriber_;
        ros::Publisher global_goal_publisher_;
        geometry_msgs::PoseStamped global_goal_pose_front_;
        geometry_msgs::PoseStamped global_goal_pose_rear_;
        geometry_msgs::TransformStamped tf_nav_base_default_;
        geometry_msgs::TransformStamped tf_nav_base_;
        geometry_msgs::TransformStamped tf_rear2front_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener* ptr_tf_listener_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
};
#endif