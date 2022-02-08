#include <switch_nav_base/switch_nav_base.h>
// TODO:
// - goals are still jumping further away
// - goal success detection not perfect yet
// - maybe use jointFront instead of nav_base for global costmap

SwitchNavBase::SwitchNavBase()
{
    // constructor
    ptr_tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}

SwitchNavBase::SwitchNavBase(ros::NodeHandle *nh)
{
    // overloaded constructor
    ptr_tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    nh_ = nh;
    seq_tf_ = 0;
    goal_status_ = 0;
    goal_success_ctr_ = 0;
    backward_motion_ = false;
    rear_goal_active_ = false;
    front_goal_active_ = false;
    ready_for_new_goal_ = true;
    you_can_switch_now_ = true;
    time_at_switch_ = ros::Time::now();
    nh->getParam("frame_id_front", frame_id_front_);
    nh->getParam("frame_id_rear", frame_id_rear_);
    nh->getParam("frame_id_output", frame_id_output_);
    nh->getParam("frame_id_map", frame_id_map_);
    nh->getParam("wheelbase_theta_zero", wheelbase_theta_zero_);
    double delay_switch = 0;
    nh->getParam("delay_switch", delay_switch);
    time_delay_switch_ = ros::Duration(delay_switch);
    // default pose of nav_base (or named differently via frame_id_output) is identical to frame of front carriage
    tf_nav_base_default_.header.stamp = ros::Time::now();
    tf_nav_base_default_.header.frame_id = frame_id_front_;
    tf_nav_base_default_.child_frame_id = frame_id_output_;
    tf_nav_base_default_.header.seq = seq_tf_;
    tf_nav_base_default_.transform.translation.x = 0.0;
    tf_nav_base_default_.transform.translation.y = 0.0;
    tf_nav_base_default_.transform.translation.z = 0.0;
    tf_nav_base_default_.transform.rotation.x = 0.0;
    tf_nav_base_default_.transform.rotation.y = 0.0;
    tf_nav_base_default_.transform.rotation.z = 0.0;
    tf_nav_base_default_.transform.rotation.w = 1.0;
    // reset nav_base
    tf_nav_base_ = tf_nav_base_default_;
    create_sub_pub_();
}

SwitchNavBase::~SwitchNavBase()
{
    // destructor
    delete ptr_tf_listener_;
}

void SwitchNavBase::create_sub_pub_()
{   
    cmd_vel_subscriber_ = nh_->subscribe("/cmd_vel", 1, &SwitchNavBase::cmd_vel_callback_, this);
    body_angle_subscriber_ = nh_->subscribe("/sensors/bodyAngle", 1, &SwitchNavBase::body_angle_callback_, this);
    goal_status_subscriber_ = nh_->subscribe("/move_base/status", 1, &SwitchNavBase::goal_status_callback_, this);
    global_goal_subscriber_ = nh_->subscribe("/move_base_simple/goal", 1, &SwitchNavBase::global_goal_callback_, this);
    global_goal_publisher_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);
}

void SwitchNavBase::goal_status_callback_(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    if (!msg->status_list.empty())
    {
        // check if front carriage successfully has reached the front goal
        // ignore rear carriage reaching rear goal
        goal_status_ = msg->status_list[0].status;
        goal_success_now_ = goal_status_ == actionlib_msgs::GoalStatus::SUCCEEDED;
        if (goal_success_now_ && front_goal_active_)
        {
            goal_success_ctr_++;
            ROS_INFO("Counted goal in row: %d", goal_success_ctr_);
        }
        else
        {
            goal_success_ctr_ = 0;
        }

        // only if the goal status SUCCEEDED persists for n times in a row, accept goal success
        bool goal_success_accepted_ = goal_success_ctr_ >= 10;   
        if (!ready_for_new_goal_ && goal_success_accepted_)
        {
            goal_success_ctr_ = 0;
            ready_for_new_goal_ = true;
        }
    }
}

void SwitchNavBase::cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr &msg)
{
    // determine if robot moves for- or backwards
    backward_motion_ = msg->linear.x < 0.0;
    // may the robot switch between front and rear goal or between front and rear frame?
    you_can_switch_now_ = ros::Time::now() - time_at_switch_ >= time_delay_switch_;
    // if robot is driving backwards
    if (backward_motion_)
    {
        if (rear_goal_active_)
        {
            // determine pose of rear frame
            bool exception_caught = true;
            try
            {
                tf_rear2front_ = tf_buffer_.lookupTransform(frame_id_front_, frame_id_rear_, ros::Time(0));
                exception_caught = false;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.1).sleep();
            }
            
            // set nav_base to rear frame
            if (!exception_caught)
            {
                tf_nav_base_ = tf_rear2front_;
            }
        }
        else
        {
            if (you_can_switch_now_)
            {
                global_goal_pose_rear_.header.frame_id = frame_id_map_;
                global_goal_pose_rear_.header.stamp = ros::Time::now();
                global_goal_publisher_.publish(global_goal_pose_rear_);
                rear_goal_active_ = true;
                front_goal_active_ = false;            
                time_at_switch_ = ros::Time::now();
            }
        }
    }
    // if robot is driving forwards
    else
    {
        if (front_goal_active_)
        {                   
            // set nav_base to front frame
            tf_nav_base_ = tf_nav_base_default_;
        }
        else
        {    
            if (you_can_switch_now_)
            {
                global_goal_pose_front_.header.frame_id = frame_id_map_;
                global_goal_pose_front_.header.stamp = ros::Time::now();
                global_goal_publisher_.publish(global_goal_pose_front_);
                front_goal_active_ = true;
                rear_goal_active_ = false;
                time_at_switch_ = ros::Time::now();
            } 
        }
    }
}


void SwitchNavBase::body_angle_callback_(const base::Angle::ConstPtr &msg)
{
    // The topic /sensors/bodyAngle is only subscribed in order to broadcast
    // the transform with the same rate used within the simulation/ real robot.
    // This reduces error messages like: 
    // "TF_REPEATED_DATA ignoring data with redundant timestamp for frame ..."

    // update header information of nav_base frame w.r.t. frame of front carriage
    tf_nav_base_.header.seq = seq_tf_++;
    tf_nav_base_.header.stamp = ros::Time::now();
    tf_nav_base_.header.frame_id = frame_id_front_;
    tf_nav_base_.child_frame_id = frame_id_output_;
    // publish nav_base frame so it can be used by the local planner
    tf_broadcaster_.sendTransform(tf_nav_base_);
}

void SwitchNavBase::global_goal_callback_(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // only reinizialize rear and front goal if previous goal has been reached successfully
    // otherwise the global_goal_publisher_ and global_goal_subscriber_ would trigger each other.
    if (ready_for_new_goal_)
    {
        // goal for front frame
        global_goal_pose_front_ = *msg;

        // goal for rear frame
        global_goal_pose_rear_ = global_goal_pose_front_;

        // retrieve yaw angle of goal pose
        tf2::Quaternion q_rear(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );        
        tf2::Matrix3x3 m_rear(q_rear);
        double roll, pitch, yaw;
        m_rear.getRPY(roll, pitch, yaw);

        // determine goal for rear frame, lying behind the front goal by the length of the wheelbase which the articulated vehicle exhibits, 
        // when the articulation angle is zero and the x-axis of the front and rear carriage is parallel.
        global_goal_pose_rear_.pose.position.x -= cos(yaw)*wheelbase_theta_zero_;
        global_goal_pose_rear_.pose.position.y -= sin(yaw)*wheelbase_theta_zero_;
        ROS_INFO("NEW GOAL SET BY USER");
        ready_for_new_goal_ = false;
    }
}
