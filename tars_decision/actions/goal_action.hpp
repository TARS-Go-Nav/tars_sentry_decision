#pragma once

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.hpp"
#include "../behavior_tree/behavior_node.hpp"

namespace tars_decision
{

class GoalAction : public ActionNode
{
  public:
    GoalAction(tars_decision::ChassisExecutor::SharedPtr chassis_executor_, 
               tars_decision::Blackboard::SharedPtr blackboard_, 
               tars_decision::Pose goal_pose_,
               int aitwist_ = 0) 
               : ActionNode::ActionNode("goal_action", blackboard_), 
                 chassis_executor(chassis_executor_),
                 aitwist(aitwist_){
        goal_pose.position.x = goal_pose_.position.x;
        goal_pose.position.y = goal_pose_.position.y;
        goal_pose.orientation.x = goal_pose_.orientation.x;
        goal_pose.orientation.y = goal_pose_.orientation.y;
        goal_pose.orientation.z = goal_pose_.orientation.z;
        goal_pose.orientation.w = goal_pose_.orientation.w;
    }

    virtual ~GoalAction() = default;

private:
    virtual void OnInitialize() {
        std::cout << "Goal Action start!" << std::endl; 
        chassis_executor->Execute(goal_pose);
        if(aitwist){
            blackboard_ptr_->setaiTuoluo(1);
        }
        std::cout << "Execute: x: " << goal_pose.position.x << ", y: " << goal_pose.position.y << std::endl;
    }

    virtual BehaviorState Update() {
        chassis_executor->Execute(goal_pose);
        if(blackboard_ptr_->getDistance(blackboard_ptr_->getCurrentPose(),goal_pose)<0.25){
            return BehaviorState::SUCCESS;
        }
        else{
        return chassis_executor->Update();
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
    geometry_msgs::msg::Pose goal_pose;
    int aitwist;
};
}
