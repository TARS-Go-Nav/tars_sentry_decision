#pragma once

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.hpp"
#include "../behavior_tree/behavior_node.hpp"

namespace tars_decision {

class StopAction : public ActionNode {
public:
    StopAction(tars_decision::ChassisExecutor::SharedPtr chassis_executor_,
              tars_decision::Blackboard::SharedPtr blackboard_)
              : ActionNode::ActionNode("stop_action", blackboard_),
                chassis_executor(chassis_executor_) {
    }

    virtual ~StopAction() = default;

private:
    virtual void OnInitialize() {
        std::cout<<"Stop Action start!"<<std::endl;
        // 取消当前的导航命令
        chassis_executor->Cancel();
    }

    virtual BehaviorState Update() {
        // 直接返回成功状态，不执行任何导航
        return BehaviorState::SUCCESS;
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
};

}