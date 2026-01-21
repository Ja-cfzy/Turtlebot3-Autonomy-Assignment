#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace semantic_reasoning {

class NavInterface {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    NavInterface(rclcpp::Node::SharedPtr node)
        : node_(node),
          navigation_complete_(false),
          navigation_success_(false) {
        
        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            node_, "navigate_to_pose");
        
        RCLCPP_INFO(node_->get_logger(), "Waiting for Nav2 action server...");
        
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node_->get_logger(), 
                        "Nav2 action server not available after waiting");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Nav2 action server is ready");
        }
    }
    
    bool navigateToPose(const geometry_msgs::msg::PoseStamped& goal) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available");
            return false;
        }
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal;
        
        RCLCPP_INFO(node_->get_logger(), 
                    "Sending goal to Nav2: (%.2f, %.2f)",
                    goal.pose.position.x, goal.pose.position.y);
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
                    navigation_complete_ = true;
                    navigation_success_ = false;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
                    goal_handle_ = goal_handle;
                }
            };
        
        send_goal_options.feedback_callback =
            [this](GoalHandleNavigateToPose::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                RCLCPP_DEBUG(node_->get_logger(), 
                            "Distance remaining: %.2f meters",
                            feedback->distance_remaining);
            };
        
        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult& result) {
                navigation_complete_ = true;
                
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node_->get_logger(), "Navigation succeeded!");
                        navigation_success_ = true;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(node_->get_logger(), "Navigation was aborted");
                        navigation_success_ = false;
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(node_->get_logger(), "Navigation was canceled");
                        navigation_success_ = false;
                        break;
                    default:
                        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                        navigation_success_ = false;
                        break;
                }
            };
        
        navigation_complete_ = false;
        navigation_success_ = false;
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        
        return true;
    }
    
    bool isNavigationComplete() const {
        return navigation_complete_;
    }
    
    void cancelNavigation() {
        if (goal_handle_) {
            RCLCPP_INFO(node_->get_logger(), "Cancelling navigation goal");
            action_client_->async_cancel_goal(goal_handle_);
        }
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    bool navigation_complete_;
    bool navigation_success_;
};

} // namespace semantic_reasoning