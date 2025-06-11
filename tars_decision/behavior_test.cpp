#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "behavior_tree/behavior_tree.hpp"
#include "executor/chassis_executor.hpp"
#include "actions/goal_action.hpp"
#include "actions/follow_action.hpp"
#include "actions/wait_normal_action.hpp"
#include "actions/twist_action.hpp"

char command = '0';

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr ros_node = std::make_shared<rclcpp::Node>("behavior_test_node");

    auto chassis_executor = std::make_shared<tars_decision::ChassisExecutor>(ros_node);
    auto blackboard = std::make_shared<tars_decision::Blackboard>(ros_node,"/home/xiaoshuu/ws022_decisioncar/src/tars_decision/config/rmuc_map.xml");

    auto root_node = std::make_shared<tars_decision::SequenceNode>("root_node", blackboard);
    auto game_status_selector = std::make_shared<tars_decision::SelectorNode>("game_status_selector", blackboard);

    tars_decision::Pose goal;
    goal.position.x = 3.3;
    goal.position.y = 4.0;
    goal.orientation.x = 0.0;
    goal.orientation.y = 0.0;
    goal.orientation.z = 0.0;
    goal.orientation.w = 1.0;
    
    auto goal_action = std::make_shared<tars_decision::GoalAction>(chassis_executor, blackboard, goal);
    auto follow_action = std::make_shared<tars_decision::FollowAction>(chassis_executor, blackboard);
    auto wait_normal_action = std::make_shared<tars_decision::WaitNormalAction>(chassis_executor, blackboard,5,1);
    auto twist_action = std::make_shared<tars_decision::TwistAction>(chassis_executor, blackboard, goal);
    auto get_supply_sequence = std::make_shared<tars_decision::SequenceNode>("get_supply_sequence",blackboard);

    auto false_condition_ = std::make_shared<tars_decision::PreconditionNode>(
        "false_condition", blackboard,
        [&]()
        {
            return true;
        },
        tars_decision::AbortType::BOTH);

    auto true_condition_ = std::make_shared<tars_decision::PreconditionNode>(
        "true_condition", blackboard,
        [&]()
        {
            return true;
        },
        tars_decision::AbortType::BOTH);

    std::cout << "===========================================" << std::endl
              << "========== Please Send a Command ==========" << std::endl
              << "=                                         =" << std::endl;
    std::cout << "=     [1] Go To Enemy                     =" << std::endl
              << "=     [2] Go To Enemy Base                =" << std::endl
              << "=     [3] Wait                            =" << std::endl
              << "=     [4] Twist                           =" << std::endl
              << "=     [Esc] Exit                          =" << std::endl
              << "=                                         =" << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << "> ";
    std::cin >> command;

    switch (command)
    {
        case '1':
            root_node->AddChildren(follow_action);
            break;
        case '2':
            root_node->AddChildren(goal_action);
            break;
        case '3':
            root_node->AddChildren(game_status_selector);
            game_status_selector->AddChildren(false_condition_);
            game_status_selector->AddChildren(true_condition_);
            true_condition_->SetChild(get_supply_sequence);
            get_supply_sequence->AddChildren(goal_action);
            get_supply_sequence->AddChildren(wait_normal_action);
            break;
        case '4':
            root_node->AddChildren(twist_action);
            break;
        case 27:
            return 0;
        default:
            break;
    }

    tars_decision::BehaviorTree tree(root_node);

    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        tree.Run();
        rate.sleep();
    }

    return 0;
}
