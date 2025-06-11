#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "../behavior_tree/behavior_state.hpp"

namespace tars_decision
{
class ChassisExecutor
{
typedef rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr NavActionClient;
typedef std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> FutureGoalHandle;

  public:
    // enum class ExecutorState {
    //     IDLE,     ///< Default idle state with no task
    //     RUNNING,  ///< Goal-targeted task state using Nav2
    //     ERROR
    // };

    enum class ExecutorMode {
        IDLE_MODE,
        GOAL_MODE,
        SPEED_MODE
    };

    typedef std::shared_ptr<ChassisExecutor> SharedPtr;

    ChassisExecutor(rclcpp::Node::SharedPtr node_) : ros_node(node_), 
                                                     executor_state(BehaviorState::IDLE), 
                                                     executor_mode(ExecutorMode::IDLE_MODE) {
        nav2_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ros_node, "navigate_to_pose");
        cmd_vel_pub = ros_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        zero_twist = geometry_msgs::msg::Twist();
    }

    ~ChassisExecutor() = default;

    /**
     * @brief Execute the goal-targeted task using global and local planner with actionlib
     * @param goal Given taget goal
     */
    BehaviorState Execute(const geometry_msgs::msg::Pose &goal) {
        if(!nav2_client->wait_for_action_server(std::chrono::seconds(5))) {
            // RCLCPP_ERROR(logger, "NavigateToPose server not available!");
            return BehaviorState::FAILURE;
        }
        goal_pose.pose.header.frame_id = "map";
        goal_pose.pose.header.stamp = ros_node->get_clock()->now();
        goal_pose.pose.pose.position.x = goal.position.x;
        goal_pose.pose.pose.position.y = goal.position.y;
        goal_pose.pose.pose.orientation.x = goal.orientation.x;
        goal_pose.pose.pose.orientation.y = goal.orientation.y;
        goal_pose.pose.pose.orientation.z = goal.orientation.z;
        goal_pose.pose.pose.orientation.w = goal.orientation.w;

        executor_mode = ExecutorMode::GOAL_MODE;
        executor_state = BehaviorState::RUNNING;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            [this](auto goal_handle) {
                if (!goal_handle) {
                    executor_state = BehaviorState::FAILURE;
                }
            };
        // send_goal_options.feedback_callback =
        //     [this](auto, auto feedback) {
        //         current_distance = feedback->distance_remaining;
        //         current_pose = feedback->current_pose;
        //     };
        send_goal_options.result_callback =
            [this](auto result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        executor_state = BehaviorState::SUCCESS;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                    case rclcpp_action::ResultCode::CANCELED:
                        executor_state = BehaviorState::FAILURE;
                        break;
                }
            };
        future_goal_handle = nav2_client->async_send_goal(goal_pose, send_goal_options);
    }


    /**
     * @brief Execute the velocity task with publisher
     * @param twist Given velocity
     */
    void Execute(const geometry_msgs::msg::Twist &twist) {
        if(executor_mode == ExecutorMode::GOAL_MODE) {
            Cancel();
        }
        executor_mode = ExecutorMode::SPEED_MODE;
        executor_state = BehaviorState::RUNNING;
        cmd_vel_pub->publish(twist);
    }

    /**
     * @brief Update the current chassis executor state
     * @return Current chassis executor state(same with behavior state)
     */
    BehaviorState Update() {
        int8_t status = rclcpp_action::GoalStatus::STATUS_UNKNOWN;
        switch(executor_mode)
        {
            case ExecutorMode::IDLE_MODE:
                executor_state = BehaviorState::IDLE;
                break;

            case ExecutorMode::GOAL_MODE:
                break;

            case ExecutorMode::SPEED_MODE:
                executor_state = BehaviorState::RUNNING;
                break;

            default:
                printf("ERROR!");
                // ROS_ERROR("Wrong Execution Mode");
        }
        // if(executor_state == BehaviorState::SUCCESS){
        //     printf("SUCCESS!");
        // }
        return executor_state;
    }

    /**
     * @brief Cancel the current task and deal with the mode transition
     */
    void Cancel() {
        switch (executor_mode) {
        case ExecutorMode::IDLE_MODE:
            // ROS_WARN("Nothing to be canceled.");
            executor_state = BehaviorState::IDLE;
            break;

        case ExecutorMode::GOAL_MODE:
            nav2_client->async_cancel_all_goals();
            cmd_vel_pub->publish(zero_twist);
            executor_mode = ExecutorMode::IDLE_MODE;
            executor_state = BehaviorState::IDLE;
            break;

        case ExecutorMode::SPEED_MODE:
            cmd_vel_pub->publish(zero_twist);
            executor_mode = ExecutorMode::IDLE_MODE;
            executor_state = BehaviorState::IDLE;
            break;

        default:
            printf("ERROR!");
            // ROS_ERROR("Wrong Execution Mode");
        }
    }

private:
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    geometry_msgs::msg::Twist zero_twist;

    BehaviorState executor_state;
    ExecutorMode executor_mode;

    NavActionClient nav2_client;
    FutureGoalHandle future_goal_handle;
    nav2_msgs::action::NavigateToPose::Goal goal_pose;

    geometry_msgs::msg::PoseStamped current_pose;
    double current_distance;
};
}