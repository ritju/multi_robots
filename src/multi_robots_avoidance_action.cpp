#include "nav2_multi_robots_avoidance/multi_robots_avoidance_action.hpp"

using std::placeholders::_1, std::placeholders::_2;

namespace multi_robots_avoidance_action
{
    MultiRobotsAvoidanceAction::MultiRobotsAvoidanceAction(const rclcpp::NodeOptions &options)
        : rclcpp::Node("multi_robots_avoidance", options)
    {
        RCLCPP_INFO(get_logger(), "multi_robot_avoidance constructor.");
        // 初始化部分成员变量
        this->twist_zero_.linear.x = 0.0;
        this->twist_zero_.angular.z = 0.0;
        this->state_last_ = RobotState::FORWARDING;
        this->state_current_ = RobotState::FORWARDING;
        this->collision_ = false;
        this->access_ = new mutex_t();

        RCLCPP_INFO(get_logger(), "Robot current state: %s", (bool)this->state_current_?"FORWARDING":"WAITING");

        // 获取node参数值。
        init_params();

        // init tf2
        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

        // pub for /robot_info
        this->robot_info_pub_ = this->create_publisher<capella_ros_msg::msg::RobotInfo>("/robot_info", 1);
        this->timer_pub_robot_info_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / this->frequency_pub_robot_info_)), std::bind(&MultiRobotsAvoidanceAction::timer_pub_robot_info_callback_,this));

        // pub for /robot_pose
        this->robot_pose_pub_ = this->create_publisher<capella_ros_msg::msg::RobotPoseWithNamespace>("robot_pose", 1);
        this->timer_pub_robot_pose_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / this->frequency_pub_pose_)), 
            std::bind(&MultiRobotsAvoidanceAction::timer_pub_robot_pose_callback_, this));

        // timer for check robots's online state
        this->timer_check_other_robots_online_state_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / this->frequency_check_online_)),
            std::bind(&MultiRobotsAvoidanceAction::timer_check_other_robots_online_state_callback_, this));

        // pub for cmd_vel_nav for smoother
        this->cmd_vel_nav_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);

        // sub for /robot_info
        rclcpp::CallbackGroup::SharedPtr cb_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt1 = rclcpp::SubscriptionOptions();
        sub_opt1.callback_group = cb_group1;
        this->higher_priority_robot_info_sub_ = this->create_subscription<capella_ros_msg::msg::RobotInfo>("/robot_info", 1, std::bind(&MultiRobotsAvoidanceAction::higher_priority_robot_info_sub_callback_, this, _1), sub_opt1);

        // sub for /robot_pose
        rclcpp::CallbackGroup::SharedPtr cb_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt2 = rclcpp::SubscriptionOptions();
        sub_opt2.callback_group = cb_group2;
        this->robot_pose_sub_ = this->create_subscription<capella_ros_msg::msg::RobotPoseWithNamespace>("robot_pose", 20, 
            std::bind(&MultiRobotsAvoidanceAction::higher_priority_robot_pose_sub_callback_, this, _1), sub_opt2);

        // sub for plan_stamped
        rclcpp::CallbackGroup::SharedPtr cb_group3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt3 = rclcpp::SubscriptionOptions();
        sub_opt3.callback_group = cb_group3;
        this->robot_plan_sub_ = this->create_subscription<capella_ros_msg::msg::PlanWithNamespace>("plan_stamped", 20,
                std::bind(&MultiRobotsAvoidanceAction::higher_priority_robot_plan_sub_callback_, this, _1), sub_opt3);

        // sub for /cmd_vel_nav_
        rclcpp::CallbackGroup::SharedPtr cb_group4 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt4 = rclcpp::SubscriptionOptions();
        sub_opt4.callback_group = cb_group2;
        this->current_robot_controller_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav_", 10, std::bind(&MultiRobotsAvoidanceAction::current_robot_controller_vel_sub_callback_, this, _1), sub_opt4);

        // pub for /plan_stamped
        this->new_plan_pub_ = this->create_publisher<capella_ros_msg::msg::PlanWithNamespace>("plan_stamped", 1);

        // sub for /plan 
        this->current_robot_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>("plan", 1, std::bind(&MultiRobotsAvoidanceAction::current_robot_plan_sub_callback_, this, _1));

        this->other_robots_infos = this->get_higher_priority_robots_infos();
    }

    MultiRobotsAvoidanceAction::~MultiRobotsAvoidanceAction()
    {
        delete access_;
    }

    void MultiRobotsAvoidanceAction::init_params()
    {
        // declare params
        this->declare_parameter<int>("priority", 0);
        this->declare_parameter<float>("collision_plan_check_threshold", 1.5);
        this->declare_parameter<float>("collision_radius_check_threshold", 3.0);
        this->declare_parameter<float>("global_pose_filter_threshold", 5.0);
        this->declare_parameter<float>("vel_x_max", 0.3);
        this->declare_parameter<float>("robot_offline_timeout", 1.5);
        this->declare_parameter<int>("frequency_pub_pose", 5);
        this->declare_parameter<int>("frequency_check_online", 5);
        this->declare_parameter<int>("frequency_pub_robot_info", 5);
        this->declare_parameter<float>("pose_and_plan_timeout", 1.0);
        this->declare_parameter<float>("time_tolerance", 0.1);
        this->declare_parameter<std::string>("dummy_namespace_name", "mk");

        // get params
        this->priority_                          = this->get_parameter_or<int>("priority", 0);
        this->collision_plan_check_threshold_    = this->get_parameter_or<float>("collision_plan_check_threshold", 1.5);
        this->collision_radius_check_threshold_  = this->get_parameter_or<float>("collision_radius_check_threshold", 3.0);
        this->global_pose_filter_threshold_      = this->get_parameter_or<float>("global_pose_filter_threshold", 5.0);
        this->vel_x_max_                         = this->get_parameter_or<float>("vel_x_max",0.3);
        this->robot_offline_timeout_             = this->get_parameter_or<float>("robot_offline_timeout",0.3);
        this->frequency_pub_pose_                = this->get_parameter_or<int>("frequency_pub_pose", 5);
        this->frequency_check_online_            = this->get_parameter_or<int>("frequency_check_online", 5);
        this->frequency_pub_robot_info_          = this->get_parameter_or<int>("frequency_pub_robot_info", 5);
        this->pose_and_plan_timeout_             = this->get_parameter_or<float>("pose_and_plan_timeout", 1.0);
        this->time_tolerance_                    = this->get_parameter_or<float>("time_tolerance", 0.1);
        this->namespace_name_                    = this->get_parameter_or<std::string>("dummy_namespace_name", "mk");

        RCLCPP_INFO(this->get_logger(), "The namespace of this node is %s.", this->namespace_name_.c_str());
    }

    void MultiRobotsAvoidanceAction::timer_pub_robot_info_callback_()
    {
        RCLCPP_DEBUG(get_logger(), "timer_pub_robot_info_callback_ begin");
        capella_ros_msg::msg::RobotInfo robot_info_msg;
        robot_info_msg.namespace_name = this->namespace_name_;
        robot_info_msg.priority = this->priority_;
        this->robot_info_pub_->publish(robot_info_msg);  
        RCLCPP_DEBUG(get_logger(), "timer_pub_robot_info_callback_ end");  
    }

    bool MultiRobotsAvoidanceAction::get_robot_pose()
    {
        RCLCPP_DEBUG(get_logger(), "get_robot_pose begin");
        std::string errMsg;
        std::string refFrame = std::string("map");
        std::string childFrame = std::string("base_link");
        geometry_msgs::msg::TransformStamped transform;

        if (!this->tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
		    tf2::durationFromSec(0.5), &errMsg))
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
            return false;
        } 
        else 
        {
            try 
            {
                transform = tf_buffer_->lookupTransform( refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(0.5));
                this->robot_pose_.header = transform.header;
                this->robot_pose_.header.stamp = this->get_clock()->now();
                this->robot_pose_.pose.position.x = transform.transform.translation.x;
                this->robot_pose_.pose.position.y = transform.transform.translation.y;
                this->robot_pose_.pose.position.z = transform.transform.translation.z;
                this->robot_pose_.pose.orientation = transform.transform.rotation;
            } 
            catch (const tf2::TransformException & e) 
            {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
                return false;
            }
        }
        RCLCPP_DEBUG(get_logger(), "get_robot_pose end");

        return true;
    }

    void MultiRobotsAvoidanceAction::timer_pub_robot_pose_callback_()
    {
        RCLCPP_DEBUG(get_logger(), "timer_pub_robot_pose_callback_ begin");
        if(get_robot_pose())
        {
            capella_ros_msg::msg::RobotPoseWithNamespace pose_with_namespace;
            pose_with_namespace.namespace_name = this->namespace_name_;
            pose_with_namespace.pose = this->robot_pose_;
            this->robot_pose_pub_->publish(pose_with_namespace);
        }
        RCLCPP_DEBUG(get_logger(), "timer_pub_robot_pose_callback_ end");
    }

    void MultiRobotsAvoidanceAction::higher_priority_robot_pose_sub_callback_(const capella_ros_msg::msg::RobotPoseWithNamespace &pose)
    {
        RCLCPP_DEBUG(get_logger(), "higher_priority_robot_pose_sub_callback_ begin");
        std::lock_guard<mutex_t> guard(*getMutex());
        for (size_t i = 0; i < this->other_robots_infos.size(); i++)
        {
            RobotInfos& robot_info = this->other_robots_infos[i];
            if (pose.namespace_name == robot_info.namespace_name)
            {
                robot_info.pose = pose.pose;
                robot_info.time_last_detected = this->get_clock()->now();

                double now_time = rclcpp::Time(robot_info.time_last_detected).seconds();
                double plan_time = rclcpp::Time(pose.pose.header.stamp).seconds();
                double delta_time = now_time - plan_time;
                if (delta_time > this->pose_and_plan_timeout_)
                {
                    RCLCPP_WARN(get_logger(), "The time of pose received from %s is too late, delta_time: %f", pose.namespace_name.c_str(), delta_time);
                }

                break;
            }
            else
            {
                if (i == this->other_robots_infos.size() - 1)
                {
                    // RCLCPP_WARN(this->get_logger(), "robot %s received a pose msg, but it's namespace %s was not in the robot infos list",
                    //     this->namespace_name_.c_str(), pose.namespace_name.c_str());
                }
            }
        }
        RCLCPP_DEBUG(get_logger(), "higher_priority_robot_pose_sub_callback_ end");
    }

    void MultiRobotsAvoidanceAction::higher_priority_robot_plan_sub_callback_(const capella_ros_msg::msg::PlanWithNamespace &plan)
    {
        RCLCPP_DEBUG(get_logger(), "higher_priority_robot_plan_sub_callback_ begin");
        std::lock_guard<mutex_t> guard(*getMutex());
        for (size_t i = 0; i < this->other_robots_infos.size(); i++)
        {
            RobotInfos& robot_info = this->other_robots_infos[i];
            if (plan.namespace_name == robot_info.namespace_name)
            {
                robot_info.path = plan.path;
                robot_info.time_last_detected = this->get_clock()->now();
                robot_info.time_last_detected_plan = robot_info.time_last_detected;

                double now_time = rclcpp::Time(robot_info.time_last_detected).seconds();
                double plan_time = rclcpp::Time(plan.path.header.stamp).seconds();
                double delta_time = now_time - plan_time;
                if (delta_time > this->pose_and_plan_timeout_)
                {
                    RCLCPP_WARN(get_logger(), "The time of plan received from %s is too late, delta_time: %f", plan.namespace_name.c_str(), delta_time);
                }
                break;
            }
            else
            {
                if (i == this->other_robots_infos.size() - 1)
                {
                    // RCLCPP_WARN(this->get_logger(), "robot %s received a plan msg, but it's namespace %s was not in the robot infos list",
                    //     this->namespace_name_.c_str(), plan.namespace_name.c_str());
                }
            }
        }
        RCLCPP_DEBUG(get_logger(), "higher_priority_robot_plan_sub_callback_ end");
    }

    void MultiRobotsAvoidanceAction::higher_priority_robot_info_sub_callback_(const capella_ros_msg::msg::RobotInfo &robot_info)
    {
        RCLCPP_DEBUG(get_logger(), "higher_priority_robot_info_sub_callback_ begin");
        std::lock_guard<mutex_t> guard(*getMutex());
        bool need_stored = true;     

        if ((this->other_robots_infos.size() == 0))
        {
            if (robot_info.priority > this->priority_)
            {
                need_stored = true;
            }
            else
            {
                need_stored = false;
            }
        }
        else
        {
            for (size_t i = 0; i < this->other_robots_infos.size(); i++)
            {
                if (robot_info.namespace_name == this->namespace_name_)
                {
                    need_stored = false;
                    break;
                }
                else if (robot_info.namespace_name == this->other_robots_infos[i].namespace_name)
                {
                    need_stored = false;
                    this->other_robots_infos[i].time_last_detected = this->get_clock()->now();
                    break;
                }
                else
                {
                    if ((robot_info.priority > this->priority_) &&(i == (this->other_robots_infos.size() - 1)))
                    {
                        need_stored = true;
                        break;
                    }
                    else
                    {
                        need_stored = false;
                    }
                }
            }
        }

        if(need_stored)
        {
            RCLCPP_INFO(this->get_logger(), "add robot info of %s to the monitoring list.", robot_info.namespace_name.c_str());
            RobotInfos ris = RobotInfos();
            ris.time_last_detected = this->get_clock()->now();
            ris.namespace_name = robot_info.namespace_name;
            ris.priority = this->priority_;
            ris.pose = geometry_msgs::msg::PoseStamped();
            ris.path = nav_msgs::msg::Path();
            this->other_robots_infos.push_back(ris);
        }
        else
        {
        }
        RCLCPP_DEBUG(get_logger(), "higher_priority_robot_info_sub_callback_ end");
    }

    void MultiRobotsAvoidanceAction::current_robot_controller_vel_sub_callback_(const geometry_msgs::msg::Twist &controller_vel)
    {
        RCLCPP_DEBUG(get_logger(), "current_robot_controller_vel_sub_callback_ begin");
        std::lock_guard<mutex_t> guard(*getMutex());
        this->twist_controller_ = controller_vel;
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "other_robots_infos.size: %zd", this->other_robots_infos.size());
        
        if (this->other_robots_infos.size() == 0)
        {
            this->state_current_ = RobotState::FORWARDING;
        }
        else
        {
            for (size_t i = 0; i < this->other_robots_infos.size(); i++)
            {
                this->collision_ = this->robot_collision_check(this->other_robots_infos[i]);
                if (this->collision_)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(), 1000, "collision occurs between %s and %s", this->namespace_name_.c_str(), other_robots_infos[i].namespace_name.c_str());
                    this->state_current_ = RobotState::WAITING;
                    break;
                }
                else
                {
                    if (i == this->other_robots_infos.size() - 1)
                    {
                        this->state_current_ = RobotState::FORWARDING;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        
                        
        if (this->state_current_ != this->state_last_)
        {
            RCLCPP_INFO(get_logger(), "Robot state changed from %s to %s", (bool)this->state_last_?"FORWARDING":"WAITING", (bool)this->state_current_?"FORWARDING":"WAITING");
        }

        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "robot state: %d", (int)this->state_current_);
        switch (this->state_current_)
        {
            case RobotState::WAITING:
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "robot %s stop ...", this->namespace_name_.c_str());
                cmd_vel_nav_pub_->publish(this->twist_zero_);
                break;
            }
            case RobotState::FORWARDING:
            default:
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "robot %s forwarding ...", this->namespace_name_.c_str());
                cmd_vel_nav_pub_->publish(this->twist_controller_);
            }
        }

        this->state_last_ = this->state_current_;
        RCLCPP_DEBUG(get_logger(), "current_robot_controller_vel_sub_callback_ end");
    }

    void MultiRobotsAvoidanceAction::current_robot_plan_sub_callback_(const nav_msgs::msg::Path &plan)
    {
        RCLCPP_DEBUG(get_logger(), "current_robot_plan_sub_callback_ begin");
        float time_delta = 0.0;
        float distance_delta = 0.0;        

        this->new_plan_output_.path = plan;
        this->new_plan_output_.namespace_name = this->namespace_name_;
        this->new_plan_output_.path.header.stamp = this->get_clock()->now();
        this->new_plan_output_.path.poses[0].header.stamp = this->new_plan_output_.path.header.stamp;
        this->new_plan_output_.path.poses[0].header.frame_id = plan.header.frame_id;

        for (size_t i = 1; i < plan.poses.size(); i++)
        {
            this->new_plan_output_.path.poses[i].header.frame_id = plan.header.frame_id;
            
            geometry_msgs::msg::Pose pose_start, pose_end;
            pose_start = this->new_plan_output_.path.poses[i-1].pose;
            pose_end = this->new_plan_output_.path.poses[i].pose;
            distance_delta = std::hypot(pose_end.position.y - pose_start.position.y, pose_end.position.x - pose_start.position.x);

            rclcpp::Time ros_time_pre, ros_time_updated;
            ros_time_pre = this->new_plan_output_.path.poses[i-1].header.stamp;
            time_delta = distance_delta / this->vel_x_max_;
            int seconds;
            unsigned int nanoseconds;
            if (time_delta < 1.0)
            {
                // RCLCPP_DEBUG(get_logger(), "time_delta: %f", time_delta);
                // RCLCPP_DEBUG(get_logger(), "time_pre.seconds: %f", ros_time_pre.seconds());
                seconds = 0;
                nanoseconds = static_cast<unsigned int>(time_delta * 1e9);
                auto duration = rclcpp::Duration(seconds, nanoseconds);
                ros_time_updated = ros_time_pre + duration;
                this->new_plan_output_.path.poses[i].header.stamp = ros_time_updated;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "the interval of time is too large, please check the /plan.");
            }
            
        }

        this->new_plan_pub_->publish(this->new_plan_output_);

        RCLCPP_DEBUG(get_logger(), "current_robot_plan_sub_callback_ end");
    }
    
    std::vector<RobotInfos> MultiRobotsAvoidanceAction::get_higher_priority_robots_infos()
    {
        RCLCPP_DEBUG(get_logger(), "get_higher_priority_robots_infos begin");  
        std::lock_guard<mutex_t> guard(*getMutex());
        return this->other_robots_infos;
        RCLCPP_DEBUG(get_logger(), "get_higher_priority_robots_infos end");  
    }

    bool MultiRobotsAvoidanceAction::pose_filter(std::string namespace_name, geometry_msgs::msg::PoseStamped poseStamped_other)
    {
        RCLCPP_DEBUG(get_logger(), "pose_filter begin");   
        // first check the timeliness of pose;
        auto now_time = this->get_clock()->now().seconds();
        RCLCPP_DEBUG(get_logger(), "sec: %d, nanosec: %d", poseStamped_other.header.stamp.sec, poseStamped_other.header.stamp.nanosec);
        RCLCPP_DEBUG(get_logger(), "pose.x: %f, pose.y: %f", poseStamped_other.pose.position.x, poseStamped_other.pose.position.y);
        auto pose_time = rclcpp::Time(poseStamped_other.header.stamp).seconds();
        float delta_time = std::abs(now_time - pose_time);
        if (delta_time > this->pose_and_plan_timeout_)
        {
            RCLCPP_WARN(this->get_logger(), "current robot %s received a stale pose of robot %s, now_time: %f, pose_time: %f, delta_time: %f, threshold: %f",
                this->namespace_name_.c_str(), namespace_name.c_str(), now_time, pose_time,  delta_time, this->pose_and_plan_timeout_);
            return true;
        }
        else
        {
            float x_current, y_current, x_other, y_other;

            x_current = this->robot_pose_.pose.position.x;
            y_current = this->robot_pose_.pose.position.y;
            x_other = poseStamped_other.pose.position.x;
            y_other = poseStamped_other.pose.position.y;

            float distance = std::hypot(y_current - y_other, x_current - x_other);
            RCLCPP_DEBUG(this->get_logger(), "the distance of current robot %s and the other robot %s is %f, threshold: %f",
                this->namespace_name_.c_str(), namespace_name.c_str(), distance, this->global_pose_filter_threshold_);

            if (distance > this->global_pose_filter_threshold_)
            {
                RCLCPP_DEBUG(get_logger(), "pose_filter end"); 
                return false;
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "pose_filter end"); 
                return true;
            }
        }
    }

    bool MultiRobotsAvoidanceAction::plan_filter(std::string namespace_name, nav_msgs::msg::Path plan_other)
    {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,  "plan_filter begin");   
        bool ret = false;
        // fix bug when plan's poses size is 0
        if (plan_other.poses.size() == 0)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Robot %s's plan is empty, ignore ...", namespace_name.c_str());
            RCLCPP_DEBUG(get_logger(), "plan_filter end1"); 
            return false;
        }
        
        float distance_updated = 0.0;
        float distance_delta = 0.0;
        float distance_to_updated = this->collision_plan_check_threshold_;
        
        auto poses_current = this->new_plan_output_.path.poses;
        if (poses_current.size() == 0)
        {
            RCLCPP_INFO(get_logger(), "Current robot's plan is empty, just wait.");
            RCLCPP_DEBUG(get_logger(), "plan_filter end2"); 
            return true;
        }
        geometry_msgs::msg::PoseStamped pose_cur_start, pose_cur_end;
        geometry_msgs::msg::PoseStamped pose_other_compare;

        pose_cur_start = poses_current[0];
        double time_delta_min = std::numeric_limits<double>::max();
        
        size_t delta_i = 0, delta_j = 0;
        size_t index_i_selected = -1, index_j_selected = -1;
        size_t size_i = poses_current.size(), size_j = plan_other.poses.size();

        std::vector<std::pair<size_t, size_t>> compare_index_vec;
        RCLCPP_DEBUG(get_logger(), "size_i: %zd", size_i);
        RCLCPP_DEBUG(get_logger(), "size_j: %zd", size_j);
        for (size_t i = index_i_selected + 1; (i < size_i) && (distance_updated < distance_to_updated);)
        {
            RCLCPP_DEBUG(get_logger(), "i: %zd", i);
            index_i_selected = i;
            pose_cur_end = poses_current[i];
            double time_cur = rclcpp::Time(pose_cur_end.header.stamp).seconds();
            RCLCPP_DEBUG(get_logger(), "stamp: %f", time_cur);
            for (size_t j = index_j_selected + 1; j < size_j;)
            {
                RCLCPP_DEBUG(get_logger(), "j: %zd", j);
                index_j_selected = j;
                double time_other = rclcpp::Time(plan_other.poses[j].header.stamp).seconds();
                RCLCPP_DEBUG(get_logger(), "stamp: %f", time_other);
                double time_delta = time_cur - time_other;
                double time_delta_abs = std::abs(time_delta);
                RCLCPP_DEBUG(get_logger(), "delta: %f", time_delta);
                
                if (time_delta_abs < time_delta_min) // 当前时间差比上一次小，继续迭代。
                {
                    // 当遇到最后一个元素时，不能再向后迭代，防止遗漏最后一个匹配结果。
                    time_delta_min = time_delta_abs;
                    if (i == size_i -1  || j == size_j -1)
                    {
                        if (time_delta_abs < this->time_tolerance_)
                        {
                            std::pair<size_t, size_t> one_pair;
                            one_pair.first = index_i_selected;
                            one_pair.second = index_j_selected;
                            RCLCPP_DEBUG(get_logger(), "push i: %zd and j: %zd",index_i_selected, index_j_selected);
                            compare_index_vec.push_back(one_pair);
                            i = size_i; // break i
                            break; // break j
                            // 用于结束遍历
                        }
                        else
                        {
                            i = size_i; // break i
                            break;  // break j 
                            // 不满足time_tolerance,不记入结果。
                        }
                    }
                    else
                    {
                        if (time_delta <= 0.0 ) // stamp i <= j => iterator i( operation: j=j,i++) ; 简化省略 delta=0的处理
                        {
                            delta_i = 1;
                            delta_j = 0;
                            i++;
                            index_j_selected = j - 1;
                            break;
                        }
                        else                    // stamp i > j => iterator j ( operation: i=i;j++)
                        {
                            delta_i = 0;
                            delta_j = 1;
                            j++;
                        }
                    }
                }
                else  // 当前时间差比上一次大，取上一次的索引作为比对结果。
                {
                    time_delta_min = std::numeric_limits<double>::max();
                    std::pair<size_t, size_t> one_pair;
                    index_i_selected = i - delta_i;
                    index_j_selected = j - delta_j;
                    one_pair.first = index_i_selected;
                    one_pair.second = index_j_selected;
                    RCLCPP_DEBUG(get_logger(), "push i: %zd and j: %zd",index_i_selected, index_j_selected);
                    compare_index_vec.push_back(one_pair);
                    break;
                }    
            } // end of loop for j

            distance_delta = std::hypot(pose_cur_end.pose.position.y - pose_cur_start.pose.position.y, 
                pose_cur_end.pose.position.x - pose_cur_start.pose.position.x);
            distance_updated += distance_delta;
            pose_cur_start = pose_cur_end;
        } // end of loop for i

        RCLCPP_DEBUG(get_logger(), "number_size: %zd", compare_index_vec.size());
        for (size_t number = 0; number < compare_index_vec.size(); number++)
        {
            RCLCPP_DEBUG(get_logger(), "number: %zd", number);
            size_t index_i, index_j;
            index_i = compare_index_vec[number].first;
            index_j = compare_index_vec[number].second;
            RCLCPP_DEBUG(get_logger(), "index_i: %zd", index_i);
            RCLCPP_DEBUG(get_logger(), "index_j: %zd", index_j);

            geometry_msgs::msg::PoseStamped pose1, pose2;
            RCLCPP_DEBUG(get_logger(), "pose_current size: %zd", poses_current.size());
            pose1 = poses_current[index_i];
            RCLCPP_DEBUG(get_logger(), "pose_other size: %zd", plan_other.poses.size());
            pose2 = plan_other.poses[index_j];

            double distance = std::hypot(pose1.pose.position.x - pose2.pose.position.x,
                pose1.pose.position.y - pose2.pose.position.y);
            if (distance < this->collision_radius_check_threshold_)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,  "======================= collsion occurs =======================");
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "pair_number: %zd, i: %zd, j: %zd", number, index_i, index_j);
                ret = true;
                double pose1_x, pose1_y, pose1_theta;
                double pose2_x, pose2_y, pose2_theta;
                double time_collision;

                // fix bug for time of collision. Do not use get_clock()->now() !!!
                auto pose_start = poses_current[0];
                double time_now = rclcpp::Time(pose_start.header.stamp).seconds();;
                double time_future = rclcpp::Time(pose1.header.stamp).seconds();
                // RCLCPP_INFO(get_logger(), "time_now: %f", time_now);
                // RCLCPP_INFO(get_logger(), "time_future: %f", time_future);
                time_collision = time_future - time_now;

                pose1_x = pose1.pose.position.x;
                pose1_y = pose1.pose.position.y;
                pose1_theta = tf2::getYaw(pose1.pose.orientation);
                pose2_x = pose2.pose.position.x;
                pose2_y = pose2.pose.position.y;
                pose2_theta = tf2::getYaw(pose2.pose.orientation);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,  "Collision will occurs after %f seconds, between %s and %s ",
                    time_collision, this->namespace_name_.c_str(), namespace_name.c_str());
                RCLCPP_DEBUG_THROTTLE(get_logger(),*get_clock(), 1000,"current robot pose: (%f, %f, %f)", pose1_x, pose1_y, pose1_theta);
                RCLCPP_DEBUG_THROTTLE(get_logger(),*get_clock(), 1000,"compare robot pose: (%f, %f, %f)", pose2_x, pose2_y, pose2_theta);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "distance: %f", distance);
                RCLCPP_DEBUG_THROTTLE(get_logger(),*get_clock(), 1000, "pose1 stamp: %f", rclcpp::Time(pose1.header.stamp).seconds());
                RCLCPP_DEBUG_THROTTLE(get_logger(),*get_clock(), 1000, "pose2 stamp: %f", rclcpp::Time(pose2.header.stamp).seconds());
                RCLCPP_DEBUG(get_logger(), "plan_filter end3"); 
                return ret;
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "----------------------- no collsion -----------------------");
                RCLCPP_DEBUG(get_logger(), "pair_number: %zd, i: %zd, j: %zd", number, index_i, index_j);
                double pose1_x, pose1_y, pose1_theta;
                double pose2_x, pose2_y, pose2_theta;

                pose1_x = pose1.pose.position.x;
                pose1_y = pose1.pose.position.y;
                pose1_theta = tf2::getYaw(pose1.pose.orientation);
                pose2_x = pose2.pose.position.x;
                pose2_y = pose2.pose.position.y;
                pose2_theta = tf2::getYaw(pose2.pose.orientation);
                RCLCPP_DEBUG(get_logger(),"current robot pose: (%f, %f, %f)", pose1_x, pose1_y, pose1_theta);
                RCLCPP_DEBUG(get_logger(),"compare robot pose: (%f, %f, %f)", pose2_x, pose2_y, pose2_theta);
                RCLCPP_DEBUG(get_logger(), "distance: %f", distance);
                RCLCPP_DEBUG(get_logger(), "pose1 stamp: %f", rclcpp::Time(pose1.header.stamp).seconds());
                RCLCPP_DEBUG(get_logger(), "pose2 stamp: %f", rclcpp::Time(pose2.header.stamp).seconds());
            }
        }
        RCLCPP_DEBUG(get_logger(), "plan_filter end4");   

        return ret;
    }

    bool MultiRobotsAvoidanceAction::robot_collision_check(RobotInfos robot_info)
    {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "robot_collision_check begin");     
        bool ret = true;
        bool bool_pose_filter, bool_plan_filter;
        double time_robot_pose = rclcpp::Time(robot_info.pose.header.stamp).seconds();
        double now_time = this->get_clock()->now().seconds();
        if (now_time - time_robot_pose > this->pose_and_plan_timeout_)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "now_time: %f, pose_time: %f", now_time, time_robot_pose);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Didn't receive a valid robot_pose of %s, just wait.", robot_info.namespace_name.c_str());
            RCLCPP_DEBUG(get_logger(), "robot_collision_check end");   
            return true; // just wait, until received a valid robot_pose. 
        }
        else
        {
            bool_pose_filter = this->pose_filter(robot_info.namespace_name, robot_info.pose);
        }
        
        bool_plan_filter = this->plan_filter(robot_info.namespace_name, robot_info.path);

        if (bool_pose_filter)
        {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Robots's pose distance <= %f, need collision check.", this->global_pose_filter_threshold_);
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "This robot %s's pose: (%f, %f)", 
                this->namespace_name_.c_str(), this->robot_pose_.pose.position.x, this->robot_pose_.pose.position.y);
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Other robot %s's pose: (%f, %f)", 
                robot_info.namespace_name.c_str(), robot_info.pose.pose.position.x, robot_info.pose.pose.position.y);

        }

        if (bool_plan_filter)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Robots's plans trigger collision.");
        }
        
        ret = ret && bool_pose_filter && bool_plan_filter;
        RCLCPP_DEBUG(get_logger(), "robot_collision_check end");   
        return ret;
    }

    void MultiRobotsAvoidanceAction::timer_check_other_robots_online_state_callback_()
    {
        RCLCPP_DEBUG(get_logger(), "timer_check_other_robots_online_state_callback_ begin");
        std::lock_guard<mutex_t> guard(*getMutex());
        size_t size = this->other_robots_infos.size();
        if (size > 0)
        {
            // RCLCPP_INFO(get_logger(), "size: %zd", size);
            for (auto iter = this->other_robots_infos.begin(); iter != this->other_robots_infos.end();)
            {
                RobotInfos& robot_info = *iter;
                auto now_time = this->get_clock()->now().seconds();
                auto last_time = rclcpp::Time(robot_info.time_last_detected).seconds();
                auto last_time_plan = rclcpp::Time(robot_info.time_last_detected_plan).seconds();

                if((std::abs(now_time - last_time_plan) > this->pose_and_plan_timeout_) && robot_info.path.poses.size() != 0)
                {
                    RCLCPP_INFO(get_logger(), "Robot %s's plan is timeout, assign empty value.", robot_info.namespace_name.c_str());
                    robot_info.path = nav_msgs::msg::Path();
                }                

                if (std::abs(now_time -  last_time> this->pose_and_plan_timeout_))
                {
                    RCLCPP_INFO(this->get_logger(), "robot %s timeout, delete...", robot_info.namespace_name.c_str());
                    RCLCPP_INFO(this->get_logger(), "now_time: %f, last_detected_time: %f, threshold: %f", 
                        now_time, last_time, this->pose_and_plan_timeout_);

                    iter = this->other_robots_infos.erase(iter);
                }
                else
                {
                    iter++;
                }
            }
        }
        RCLCPP_DEBUG(get_logger(), "timer_check_other_robots_online_state_callback_ end");        
    }

    template<typename T>
    void MultiRobotsAvoidanceAction::delete_element(std::string namespace_name_delete, std::vector<std::pair<std::string, T>> vector)
    {
        for (auto iter = vector.begin(); iter != vector.end(); iter++)
        {
            if (namespace_name_delete == (*iter).first)
            {
                (*iter).second.reset();
                vector.erase(iter);
                break;
            }
        }
    }

} // end of namespace 


RCLCPP_COMPONENTS_REGISTER_NODE(multi_robots_avoidance_action::MultiRobotsAvoidanceAction)




