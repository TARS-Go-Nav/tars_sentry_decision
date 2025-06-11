/*
    追击识别到的敌人行为
*/
#pragma once

#include "../blackboard/blackboard.hpp"
#include "../executor/chassis_executor.hpp"
#include "../behavior_tree/behavior_state.hpp"
#include "../behavior_tree/behavior_node.hpp"

namespace tars_decision
{

class FollowAction : public ActionNode
{
  public:
    FollowAction(tars_decision::ChassisExecutor::SharedPtr chassis_executor_, 
               tars_decision::Blackboard::SharedPtr blackboard_, 
               double attack_distance_ = 0.5,
               double vehicle_dim_ = 1.0,
               double cut_radius_ = 0.15,
               int min_obs_num_ = 2,
               int occ_threshold_ = 50,
               double movement_threshold_ = 0.1,// 移动阈值，单位米 如果目标移动距离小于阈值，认为目标静止
               int wait_time_ms_ = 6000)  // 等待时间，单位毫秒
               : ActionNode::ActionNode("follow_action", blackboard_),
                 chassis_executor(chassis_executor_),
                 attack_distance(attack_distance_),
                 vehicle_dim(vehicle_dim_),
                 cut_radius(cut_radius_),
                 min_obs_num(min_obs_num_),
                 occ_threshold(occ_threshold_),
                 movement_threshold(movement_threshold_),
                 wait_time_ms(wait_time_ms_){
        double circle = 2 * 3.14159 * attack_distance;
        pose_candidate_num = ceil(circle / vehicle_dim);
    }

    virtual ~FollowAction() = default;

private:
    virtual void OnInitialize() {
        std::cout<<"Follow Action start!"<<std::endl;
        target_point = blackboard_ptr_->getEnemyPose();
        last_target_point = target_point;  // 初始化上一次的目标位置
        current_pose = blackboard_ptr_->getCurrentPose();
        vehicle_X = current_pose.position.x;
        vehicle_Y = current_pose.position.y;
        local_costmap = blackboard_ptr_->getLocalCostmap();
        global_costmap = blackboard_ptr_->getGlobalCostmap();
        current_state = BehaviorState::RUNNING;
        last_target_update_time = std::chrono::steady_clock::now();
        // 更新攻击位置
        GenerateAttackPoses();
        
        // 分析障碍物并评分
        if (!AnalyzeObstacles()) {
            std::cout<<"Failed to analyze obstacles!"<<std::endl;
            current_state = BehaviorState::FAILURE;
        }

        // 选择最佳攻击位置
        geometry_msgs::msg::Pose attack_pose;
        if (!SelectBestPose(attack_pose)) {
            std::cout<<"No suitable attack position found!"<<std::endl;
            current_state = BehaviorState::FAILURE;
        }

        // 执行导航
        if(current_state == BehaviorState::RUNNING){
            // chassis_executor->Execute(attack_pose);
            std::cout<<"Navigating to attack position:("<<attack_pose.position.x<<","<<attack_pose.position.y<<")"<<std::endl;
        }
    }

    virtual BehaviorState Update() {
        if(current_state != BehaviorState::RUNNING){
            return current_state;
        }
        
        // 先检查当前导航状态
        BehaviorState executor_state = chassis_executor->Update();
        if (executor_state == BehaviorState::RUNNING) {
            // 如果还在执行导航，就继续执行
            return BehaviorState::RUNNING;
        }

        // 获取最新的信息
        target_point = blackboard_ptr_->getEnemyPose();
        current_pose = blackboard_ptr_->getCurrentPose();
        vehicle_X = current_pose.position.x;
        vehicle_Y = current_pose.position.y;
        local_costmap = blackboard_ptr_->getLocalCostmap();
        global_costmap = blackboard_ptr_->getGlobalCostmap();

        // 检查目标是否移动
        double target_movement = std::hypot(
            target_point.position.x - last_target_point.position.x,
            target_point.position.y - last_target_point.position.y
        );

        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_target_update_time
        ).count();

        if (target_movement < movement_threshold) {
            // 目标静止，检查是否需要等待
            if (time_since_last_update < wait_time_ms) {
                // 继续执行当前的导航命令
                return chassis_executor->Update();
            }
        } else {
            // 目标移动了，更新时间戳
            last_target_update_time = current_time;
        }

        // 更新上一次的目标位置
        last_target_point = target_point;

        // 更新攻击位置
        GenerateAttackPoses();
        
        // 分析障碍物并评分
        if (!AnalyzeObstacles()) {
            std::cout<<"Failed to analyze obstacles!"<<std::endl;
            return BehaviorState::FAILURE;
        }

        // 选择最佳攻击位置
        geometry_msgs::msg::Pose attack_pose;
        if (!SelectBestPose(attack_pose)) {
            std::cout<<"No suitable attack position found!"<<std::endl;
            return BehaviorState::FAILURE;
        }

        // 执行导航
        // chassis_executor->Execute(attack_pose);
        std::cout<<"Navigating to attack position:("<<attack_pose.position.x<<","<<attack_pose.position.y<<")"<<std::endl;
        
        return chassis_executor->Update();
    }

    virtual void OnTerminate(BehaviorState state) {
        switch (state)
        {
        case BehaviorState::IDLE:
            chassis_executor->Cancel();
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

    void GenerateAttackPoses(){
        attack_poses.clear();
        attack_poses_vis.markers.clear();
        alpha = atan2(vehicle_Y - target_point.position.y, vehicle_X - target_point.position.x);
        for (int i = 0; i < pose_candidate_num; i++)
        {
        geometry_msgs::msg::Pose attack_pose;
        attack_pose.position.x = attack_distance * cos(2 * 3.14159 / pose_candidate_num * i + alpha ) + target_point.position.x;
        attack_pose.position.y = attack_distance * sin(2 * 3.14159 / pose_candidate_num * i + alpha ) + target_point.position.y;
        attack_poses.push_back(attack_pose);
        pose_scores.push_back(0.0);
        visualization_msgs::msg::Marker attack_pose_vis;
        attack_pose_vis.header.frame_id = "map";
        attack_pose_vis.ns = "attack_pose_vis";
        attack_pose_vis.id = i;
        attack_pose_vis.type = visualization_msgs::msg::Marker::SPHERE;
        attack_pose_vis.action = visualization_msgs::msg::Marker::ADD;
        attack_pose_vis.pose.position.x = attack_pose.position.x;
        attack_pose_vis.pose.position.y = attack_pose.position.y;
        attack_pose_vis.pose.position.z = 0;
        attack_pose_vis.pose.orientation.x = 0;
        attack_pose_vis.pose.orientation.y = 0;
        attack_pose_vis.pose.orientation.z = 0;
        attack_pose_vis.pose.orientation.w = 1;
        attack_pose_vis.scale.x = 0.1;
        attack_pose_vis.scale.y = 0.1;
        attack_pose_vis.scale.z = 0.1;
        attack_pose_vis.color.a = 1.0;
        // the closer to the vehicle, the redder the marker
        attack_pose_vis.color.r = 0.3 + 0.7 * fabs(pose_candidate_num / 2 - i) / (pose_candidate_num / 2);
        attack_pose_vis.color.g = 0.0;
        attack_pose_vis.color.b = 0.0;
        attack_poses_vis.markers.push_back(attack_pose_vis);
        }
    }

    bool AnalyzeObstacles(){
        if (!local_costmap.info.width || !global_costmap.info.width) {
            std::cout<<"Waiting for costmap data"<<std::endl;
            return false;
        }

        // 重置分数
        std::fill(pose_scores.begin(), pose_scores.end(), 0.0);

        // 分析代价地图
        AnalyzeCostmap(global_costmap, local_costmap);

        return true;
    }

     void AnalyzeCostmap(const nav_msgs::msg::OccupancyGrid& global_costmap, const nav_msgs::msg::OccupancyGrid& local_costmap) {
        for (unsigned int i = 0; i < global_costmap.info.width; i++)
        {
            for (unsigned int j = 0; j < global_costmap.info.height; j++)
            {
                auto global_costmap_index = i + j * global_costmap.info.width;
                auto x = i * global_costmap.info.resolution + global_costmap.info.origin.position.x;
                auto y = j * global_costmap.info.resolution + global_costmap.info.origin.position.y;
                auto distance_to_center = std::hypot(x - target_point.position.x, y - target_point.position.y);
                if (distance_to_center <= attack_distance && distance_to_center > cut_radius)
                {
                    double theta = atan2(y - target_point.position.y, x - target_point.position.x);
                    double delta = theta - alpha;
                    if (delta < 0)
                    {
                    delta += 2 * 3.14159;
                    }
                    if (delta > 2 * 3.14159)
                    {
                    delta -= 2 * 3.14159;
                    }
                    double shim = 3.14159 / pose_candidate_num; //=2*3.14159/pose_candidate_num/2;
                    int index = round((delta + shim) / (2 * 3.14159 / pose_candidate_num));
                    if (index >= pose_candidate_num)
                    {
                    index = 0;
                    }
                    pose_scores[index] += global_costmap.data[global_costmap_index]/occ_threshold;
                    if(global_costmap.data[global_costmap_index]/occ_threshold){
                    // the more points in the direction, the bigger the marker
                    attack_poses_vis.markers[index].scale.x += 0.01;
                    if (attack_poses_vis.markers[index].scale.x >= vehicle_dim)
                        attack_poses_vis.markers[index].scale.x = vehicle_dim;
                    attack_poses_vis.markers[index].scale.y += 0.01;
                    if (attack_poses_vis.markers[index].scale.y >= vehicle_dim)
                        attack_poses_vis.markers[index].scale.y = vehicle_dim;
                    attack_poses_vis.markers[index].scale.z += 0.01;
                    if (attack_poses_vis.markers[index].scale.z >= vehicle_dim)
                        attack_poses_vis.markers[index].scale.z = vehicle_dim;
                    }
                }
            }
        }
        for (unsigned int i = 0; i < local_costmap.info.width; i++)
        {
            for (unsigned int j = 0; j < local_costmap.info.height; j++)
            {
                auto local_costmap_index = i + j * local_costmap.info.width;
                auto x = i * local_costmap.info.resolution + local_costmap.info.origin.position.x;
                auto y = j * local_costmap.info.resolution + local_costmap.info.origin.position.y;
                auto distance_to_center = std::hypot(x - target_point.position.x, y - target_point.position.y);
                if (distance_to_center <= attack_distance && distance_to_center > cut_radius)
                {
                    double theta = atan2(y - target_point.position.y, x - target_point.position.x);
                    double delta = theta - alpha;
                    if (delta < 0)
                    {
                    delta += 2 * 3.14159;
                    }
                    if (delta > 2 * 3.14159)
                    {
                    delta -= 2 * 3.14159;
                    }
                    int index = round(delta / (2 * 3.14159 / pose_candidate_num));
                    if (index == pose_candidate_num)
                    {
                    index = 0;
                    }
                    pose_scores[index] += local_costmap.data[local_costmap_index]/occ_threshold;
                    if(local_costmap.data[local_costmap_index]/occ_threshold){
                    // the more points in the direction, the bigger the marker
                    attack_poses_vis.markers[index].scale.x += 0.01;
                    if (attack_poses_vis.markers[index].scale.x >= vehicle_dim)
                        attack_poses_vis.markers[index].scale.x = vehicle_dim;
                    attack_poses_vis.markers[index].scale.y += 0.01;
                    if (attack_poses_vis.markers[index].scale.y >= vehicle_dim)
                        attack_poses_vis.markers[index].scale.y = vehicle_dim;
                    attack_poses_vis.markers[index].scale.z += 0.01;
                    if (attack_poses_vis.markers[index].scale.z >= vehicle_dim)
                        attack_poses_vis.markers[index].scale.z = vehicle_dim;
                    }
                }
            }
        }
        blackboard_ptr_ -> pubAttackPoseVis(attack_poses_vis);
     }

    bool SelectBestPose(geometry_msgs::msg::Pose& selected_pose) {
        if (attack_poses.empty() || pose_scores.empty()) {
            return false;
        }

        visualization_msgs::msg::Marker final_attack_pose_vis;
        final_attack_pose_vis.header.frame_id = "map";
        final_attack_pose_vis.ns = "final_attack_pose_vis";
        final_attack_pose_vis.id = 0;
        final_attack_pose_vis.type = visualization_msgs::msg::Marker::SPHERE;
        final_attack_pose_vis.action = visualization_msgs::msg::Marker::ADD;
        final_attack_pose_vis.pose.orientation.x = 0;
        final_attack_pose_vis.pose.orientation.y = 0;
        final_attack_pose_vis.pose.orientation.z = 0;
        final_attack_pose_vis.pose.orientation.w = 1;
        final_attack_pose_vis.scale.x = 0.15;
        final_attack_pose_vis.scale.y = 0.15;
        final_attack_pose_vis.scale.z = 0.15;
        final_attack_pose_vis.color.a = 1.0;
        final_attack_pose_vis.color.r = 0.0;
        final_attack_pose_vis.color.g = 1.0;
        final_attack_pose_vis.color.b = 0.0;

        for (long unsigned int i = 0; i <= attack_poses.size() / 4; i++) // start from the nearest pose and refuse point on the opposite side
        {
        if (pose_scores[i] < min_obs_num )
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Attack pose found at %ld", i);
            selected_pose = attack_poses[i];

            final_attack_pose_vis.pose.position.x = selected_pose.position.x;
            final_attack_pose_vis.pose.position.y = selected_pose.position.y;
            blackboard_ptr_ -> pubAttackPoseVis(final_attack_pose_vis);
            return true;
        }
        if (pose_scores[attack_poses.size() - i - 1] < min_obs_num)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Attack pose found at %ld", attack_poses.size() - i - 1);
            selected_pose = attack_poses[attack_poses.size() - i - 1];

            final_attack_pose_vis.pose.position.x = selected_pose.position.x;
            final_attack_pose_vis.pose.position.y = selected_pose.position.y;
            blackboard_ptr_ -> pubAttackPoseVis(final_attack_pose_vis);
            return true;
        }
        }
        RCLCPP_WARN(rclcpp::get_logger("Attack"), "No valid attack pose found, stay in place");
        current_pose = blackboard_ptr_->getCurrentPose();
        selected_pose = current_pose;

        final_attack_pose_vis.pose.position.x = selected_pose.position.x;
        final_attack_pose_vis.pose.position.y = selected_pose.position.y;
        blackboard_ptr_ -> pubAttackPoseVis(final_attack_pose_vis);
        return true;
    }

    ChassisExecutor::SharedPtr chassis_executor;
    geometry_msgs::msg::Pose target_point;
    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Pose current_pose;
    std::vector<geometry_msgs::msg::Pose> attack_poses;

    double vehicle_dim;
    double cut_radius;
    int pose_candidate_num;
    double attack_distance;
    double movement_threshold;  
    int wait_time_ms;  
    int occ_threshold;
    int min_obs_num;
    std::vector<double> pose_scores;  // 存储每个候选点的评分
    double vehicle_X,vehicle_Y;
    double alpha = 0.0;

    nav_msgs::msg::OccupancyGrid local_costmap;
    nav_msgs::msg::OccupancyGrid global_costmap;

    geometry_msgs::msg::Pose last_target_point;  // 上一次的目标位置
    std::chrono::steady_clock::time_point last_target_update_time;  // 上次更新时间戳
    visualization_msgs::msg::MarkerArray attack_poses_vis;

    BehaviorState current_state = BehaviorState::IDLE;
};
}
