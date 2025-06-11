#pragma once

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.hpp"
#include "../behavior_tree/behavior_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <filesystem>
#include <fstream>

namespace tars_decision {
class WaitNormalAction : public ActionNode {
public:
    WaitNormalAction(tars_decision::ChassisExecutor::SharedPtr chassis_executor_,
               tars_decision::Blackboard::SharedPtr blackboard_,
               double wait_duration_seconds = 10.0,
               int swing_ = 0)  // 是否小陀螺 2means ai tuoluo
        : ActionNode::ActionNode("wait_action", blackboard_),
          chassis_executor(chassis_executor_),
          wait_duration(wait_duration_seconds),
          swing(swing_) {
    }

    virtual ~WaitNormalAction() = default;

private:
    virtual void OnInitialize() {
        std::cout<<"Wait Action start! Will wait for "<<wait_duration<<"seconds"<<std::endl;
        if(swing == 1){
            blackboard_ptr_->setTuoluo(1);
        }
        else if(swing == 2){
            blackboard_ptr_->setnormalAiTuoluo(1);
        }
        // 取消当前的导航命令，确保机器人停止
        chassis_executor->Cancel();
        // 记录开始时间
        start_time = rclcpp::Clock().now();
    }

    virtual BehaviorState Update() {
        // 获取当前时间
        auto current_time = rclcpp::Clock().now();
        // 计算经过的时间
        double elapsed_time = (current_time - start_time).seconds();

        if (elapsed_time >= wait_duration) {
            // 等待时间已到
            std::cout<<"Wait complete!"<<std::endl;
            if(swing) {
                blackboard_ptr_->setTuoluo(0); // 关闭小陀螺
                blackboard_ptr_->setnormalAiTuoluo(0); // 关闭小陀螺
            }
            return BehaviorState::SUCCESS;
        } else {
            // 继续等待
            std::cout<<"Waiting..." << wait_duration - elapsed_time << "seconds left"<<std::endl;
            return BehaviorState::RUNNING;
        }
    }

    virtual void OnTerminate(BehaviorState state) {
        switch (state)
        {
        case BehaviorState::IDLE:
            // chassis_executor->Cancel();
            std::cout<<"IDLE!"<<std::endl;
            break;
        case BehaviorState::RUNNING:
            std::cout<<"RUNNING!"<<std::endl;
            break;
        case BehaviorState::SUCCESS:
            std::cout<<"SUCCESS!"<<std::endl;
            break;
        case BehaviorState::FAILURE:
            std::cout<<"FAILURE!"<<std::endl;
            break;
        default:
            std::cout<<"DEFAULT!"<<std::endl;
            return;
        }
    }

    ChassisExecutor::SharedPtr chassis_executor;
    double wait_duration;  // 等待时间（秒）
    rclcpp::Time start_time;  // 开始等待的时间点
    int swing = 0;
};

}  // namespace tars_decision