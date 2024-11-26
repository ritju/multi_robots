#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "capella_ros_msg/msg/robot_info.hpp"
#include "capella_ros_msg/msg/robot_pose_with_namespace.hpp"
#include "capella_ros_msg/msg/plan_with_namespace.hpp"

#include <chrono>
#include <string>
#include <map>
#include <utility>

namespace multi_robots_avoidance_action{

enum class RobotState
{
        WAITING,
        FORWARDING
};

struct RobotInfos
{
        builtin_interfaces::msg::Time time_last_detected;
        builtin_interfaces::msg::Time time_last_detected_plan;
        std::string namespace_name;
        int priority;
        double vel_x_max;
        geometry_msgs::msg::PoseStamped pose;
        nav_msgs::msg::Path path;
};

class MultiRobotsAvoidanceAction : public rclcpp::Node
{
        public:
                explicit MultiRobotsAvoidanceAction(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

                ~MultiRobotsAvoidanceAction();

                /**
                 * @brief 获取比当前机器人优先级高的所有机器人信息。
                 *
                 * @param 无
                 * @return 返回比当前机器人优先级高的所有机器人信息。。
                 */
                std::vector<RobotInfos> get_higher_priority_robots_infos();

                /**
                 * @brief 检查是否会与高优先级机器人发生碰撞
                 *
                 * @param robot_info 高级机器人信息
                 * @return 是否会碰撞
                 */
                bool robot_collision_check(RobotInfos robot_info);
                
                /**
                 * @brief 判断高优先级的机器人pose是否在设定的碰撞检查阈值范围内(第一个碰撞判断条件)
                 *
                 * @param namespace_name 高级机器人的namespace name 
                 * @param poses_stamped_other 高级机器人的pose
                 * @return true: 需要检查碰撞，false:无需检查碰撞。
                 */
                bool pose_filter(std::string namespace_name, geometry_msgs::msg::PoseStamped poseStamped_other);

                /**
                 * @brief 判断当前机器人的plan路径是否会与满足条件1的机器人在对应时刻发生碰撞（第二个碰撞判断条件）
                 *
                 * @param namespace_name 高级机器人的namespace_name
                 * @param plan_other 高级机器人的plan
                 * @return true: 需要检查碰撞，false:无需检查碰撞。
                 */
                bool plan_filter(std::string namespace_name, nav_msgs::msg::Path plan_other); 

                typedef std::recursive_mutex mutex_t;
                mutex_t * getMutex()
                {
                        return access_;
                }

                bool get_robot_pose();
        
        private:
                // subs
                rclcpp::Subscription<capella_ros_msg::msg::RobotInfo>::SharedPtr              higher_priority_robot_info_sub_;           // 订阅高优先级机器人信息
                rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                          current_robot_plan_sub_;                   // 订阅自身plan,增加时间戳
                rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                    current_robot_controller_vel_sub_;         // 订阅自身controller速度
                                
                rclcpp::Subscription<capella_ros_msg::msg::RobotPoseWithNamespace>::SharedPtr robot_pose_sub_;
                rclcpp::Subscription<capella_ros_msg::msg::PlanWithNamespace>::SharedPtr robot_plan_sub_;
                // pubs
                rclcpp::Publisher<capella_ros_msg::msg::RobotPoseWithNamespace>::SharedPtr robot_pose_pub_;
                rclcpp::Publisher<capella_ros_msg::msg::PlanWithNamespace>::SharedPtr new_plan_pub_;
                rclcpp::Publisher<capella_ros_msg::msg::RobotInfo>::SharedPtr robot_info_pub_;
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_pub_;     // 转发cmd_vel_nav给smoother
               
                // callbacks for subs
                void higher_priority_robot_pose_sub_callback_(const capella_ros_msg::msg::RobotPoseWithNamespace &pose); 
                void higher_priority_robot_plan_sub_callback_(const capella_ros_msg::msg::PlanWithNamespace &plan);
                void higher_priority_robot_info_sub_callback_(const capella_ros_msg::msg::RobotInfo &robot_info);
                void current_robot_plan_sub_callback_(const nav_msgs::msg::Path &plan);               
                void current_robot_controller_vel_sub_callback_(const geometry_msgs::msg::Twist &controller_vel);

                void init_params(); //初始化所有参数

                template<typename T>
                void delete_element(std::string namespace_name_delete, std::vector<std::pair<std::string, T>>);  // delete element of vector when timeout

                // params
                int priority_;                            // 机器人优先级
                float collision_plan_check_threshold_;    // path列表前方多少米为碰撞检查阈值
                float collision_radius_check_threshold_;  // 碰撞检测半径, 即根据path检查路径时，以path中的pose为圆心，collision_check_radius_半径内认为会碰撞；
                float global_pose_filter_threshold_;      // the distance threshold of two robot's pose
                float vel_x_max_;                         // 高级机器人的最大线速度
                float robot_offline_timeout_;             // 机器人离线超时时间。
                int frequency_pub_pose_;                  // frequency for pub self pose
                float pose_and_plan_timeout_;             // check the validness of pose and plan
                float time_tolerance_;                    // time tolerance for comparation of /plan
                int frequency_check_online_;              // frequency for check robot online state
                int frequency_pub_robot_info_;            // frequency for pub robot info
                
                std::string namespace_name_;              // 机器人命名空间
                
                std::vector<RobotInfos> other_robots_infos;  // 需要检查的高优先级机器人信息
                RobotState state_current_, state_last_;      // 机器人当前、最近一次碰撞检测后执行状态
                bool collision_;                             // 是否会发生碰撞
                geometry_msgs::msg::Pose  current_pose_;     // 当前机器人的pose
                geometry_msgs::msg::Twist twist_controller_; // controller产生的速度
                geometry_msgs::msg::Twist twist_zero_{};     // 0速度，用于保持静止

                mutex_t * access_;

                // timers
                rclcpp::TimerBase::SharedPtr timer_check_other_robots_online_state_;
                rclcpp::TimerBase::SharedPtr timer_pub_robot_info_;
                rclcpp::TimerBase::SharedPtr timer_pub_robot_pose_;

                // timers callbacks
                void timer_check_other_robots_online_state_callback_();
                void timer_pub_robot_info_callback_();
                void timer_pub_robot_pose_callback_();

                // tf2
                std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
                std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
                
                // current robot pose
                geometry_msgs::msg::PoseStamped robot_pose_;

                // new plan for pub and compare
                capella_ros_msg::msg::PlanWithNamespace new_plan_output_;

}; // end of class
}  // end of namespace
