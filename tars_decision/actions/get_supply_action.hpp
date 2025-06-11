#pragma once

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.hpp"
#include "../behavior_tree/behavior_node.hpp"

namespace tars_decision {

class GetSupplyAction : public ActionNode {
public:
    GetSupplyAction(tars_decision::ChassisExecutor::SharedPtr chassis_executor_,
              tars_decision::Blackboard::SharedPtr blackboard_)
              : ActionNode::ActionNode("get_supply_action", blackboard_),
                chassis_executor(chassis_executor_) {
    }

    virtual ~GetSupplyAction() = default;

private:
    virtual void OnInitialize() {
        std::cout<<"Get Supply Action start!"<<std::endl;
        // 取消当前的导航命令
        chassis_executor->Cancel();
        blackboard_ptr_->setaiTuoluo(0);
        blackboard_ptr_->setTuoluo(0);
    }

    virtual BehaviorState Update() {
        if(blackboard_ptr_->getCurrentHP()>380){
            blackboard_ptr_->setGetBullet();
            blackboard_ptr_->setaiTuoluo(0);
            blackboard_ptr_->setTuoluo(0);
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
};

}