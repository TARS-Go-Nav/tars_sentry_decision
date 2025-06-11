#pragma once

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.hpp"
#include "../behavior_tree/behavior_node.hpp"

namespace tars_decision {

class TwistAction : public ActionNode {
public:
    TwistAction(tars_decision::ChassisExecutor::SharedPtr chassis_executor_,
              tars_decision::Blackboard::SharedPtr blackboard_,
              tars_decision::Pose goal_pose_)
              : ActionNode::ActionNode("twist_action", blackboard_),
                chassis_executor(chassis_executor_) {
        goal_pose.position.x = goal_pose_.position.x;
        goal_pose.position.y = goal_pose_.position.y;
        goal_pose.orientation.x = goal_pose_.orientation.x;
        goal_pose.orientation.y = goal_pose_.orientation.y;
        goal_pose.orientation.z = goal_pose_.orientation.z;
        goal_pose.orientation.w = goal_pose_.orientation.w;
    }

    virtual ~TwistAction() = default;

private:
    virtual void OnInitialize() {
        std::cout<<"Twist Action start!"<<std::endl;
        // 取消当前的导航命令
        chassis_executor->Cancel();
        twist_state = blackboard_ptr_->control_twist(goal_pose.orientation);
        start_time = rclcpp::Clock().now();
    }

    virtual BehaviorState Update() {
        // 获取当前时间
        auto current_time = rclcpp::Clock().now();
        // 计算经过的时间
        double elapsed_time = (current_time - start_time).seconds();
        if (elapsed_time >= 4.0) {
            return BehaviorState::SUCCESS;
        }

        if(blackboard_ptr_->control_twist(goal_pose.orientation)){
            return BehaviorState::SUCCESS;
        }
        else{
            return BehaviorState::RUNNING;
        }
    }

    virtual void OnTerminate(BehaviorState state) {
        switch (state)
        {
        case BehaviorState::IDLE:
            // ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::SUCCESS:
            std::cout<<"SUCCESS!"<<std::endl;
            // ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            // ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
            break;
        default:
            // ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
            return;
        }
    }

    ChassisExecutor::SharedPtr chassis_executor;
    geometry_msgs::msg::Pose goal_pose;
    rclcpp::Time start_time;  
    bool twist_state = false;
};

}