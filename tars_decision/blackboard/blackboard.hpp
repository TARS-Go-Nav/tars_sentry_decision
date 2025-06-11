#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <map>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "semantic_map.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracked_target.hpp"
#include "tars_msgs/msg/serial_receive.hpp"

namespace tars_decision {
typedef geometry_msgs::msg::Pose Pose;
typedef geometry_msgs::msg::Point Point;
typedef std::string string;

struct GameStatus{
    int game_progress = 0;
    int remain_time = 0;
    int red_outpost_hp;  // 红方前哨站血量
    int red_base_hp;     // 红方基地血量
    int blue_outpost_hp; // 蓝方前哨站血量
    int blue_base_hp;    // 蓝方基地血量
};

struct RobotStatus{
    int current_hp = 400;
    int projectile = 300;
    int armor_be_attacked = 0; // 装甲板是否被攻击 0:否 1:是
    int out_war = 1; // 是否脱战 0:否 1:是
    int low_energy = 0; // 当前剩余能量值是否小于30% 0:否 1:是
    int rfid_baolei_state = 0; // RFID 是否检测到堡垒 0:否 1:是
    int rfid_support1_state = 0; // RFID 是否检测到补给区(与兑换站不重叠) 0:否 1:是
    int rfid_support2_state = 0; // RFID 是否检测到补给区(与兑换站重叠) 0:否 1:是
};

class Blackboard {
  public:
    typedef std::shared_ptr<Blackboard> SharedPtr;
    Blackboard(rclcpp::Node::SharedPtr node_, std::string semantic_map_path_) : ros_node(node_) {
        tf_buffer = std::make_unique<tf2_ros::Buffer>(ros_node->get_clock());
        tf_buffer->setUsingDedicatedThread(true);  // 设置使用专用线程
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        local_costmap_sub = ros_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", rclcpp::SensorDataQoS(), std::bind(&Blackboard::localCostmapCallback, this, std::placeholders::_1));
        global_costmap_sub = ros_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", rclcpp::SensorDataQoS(), std::bind(&Blackboard::globalCostmapCallback, this, std::placeholders::_1));

        enemy_pose_sub = ros_node->create_subscription<auto_aim_interfaces::msg::TrackedTarget>(
            "/front/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Blackboard::visionCallback, this, std::placeholders::_1));

        back_enemy_pose_sub = ros_node->create_subscription<auto_aim_interfaces::msg::Target>(
            "/back/targeter/target", rclcpp::SensorDataQoS(), std::bind(&Blackboard::backvisionCallback, this, std::placeholders::_1));

        game_status_sub = ros_node->create_subscription<tars_msgs::msg::SerialReceive>(
            "/serial_receive", 10, std::bind(&Blackboard::gameStatusCallback, 
                                            this, std::placeholders::_1));

        attack_pose_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/attack_pose_vis", 1);
        final_attack_pose_vis_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/final_attack_pose_vis", 1);
        tuoluo_publisher_ = ros_node->create_publisher<std_msgs::msg::Int8>("/tuoluo", 10);
        ai_tuoluo_publisher_ = ros_node->create_publisher<std_msgs::msg::Int8>("/ai_tuoluo", 10);
        cmd_vel_publisher_ = ros_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_chassis", 10);
        timer_ = ros_node->create_wall_timer(std::chrono::milliseconds(10),std::bind(&Blackboard::updateCurrentPose, this));
        timer_tuoluo = ros_node->create_wall_timer(std::chrono::microseconds(10),std::bind(&Blackboard::pubTuoluo, this));

        semantic_map_handler.loadMap(semantic_map_path_);
    }

    Pose getCurrentPose() {
        updateCurrentPose();
        return current_pose;
    }

    nav_msgs::msg::OccupancyGrid getLocalCostmap(){
        return local_costmap;
    }

    nav_msgs::msg::OccupancyGrid getGlobalCostmap(){
        return global_costmap;
    }

    Pose getEnemyPose(){
        return enemy_pose;
    }

    int getDetectEnemy(){
        return detect_enemy;
    }

    void pubAttackPoseVis(visualization_msgs::msg::MarkerArray attack_pose){
        attack_pose_vis_pub_->publish(attack_pose);
    }

    void pubAttackPoseVis(visualization_msgs::msg::Marker attack_pose){
        final_attack_pose_vis_pub_->publish(attack_pose);
    }

    int getGameProgress(){
        return game_status.game_progress;
    }

    int getRemainTime(){
        return game_status.remain_time;
    }

    int getRedOutpostHp(){
        return game_status.red_outpost_hp;
    }

    int getRedBaseHp(){
        return game_status.red_base_hp;
    }

    int getBlueOutpostHp(){
        return game_status.blue_outpost_hp;
    }

    int getBlueBaseHp(){
        return game_status.blue_base_hp;
    }

    int getCurrentHP(){
        return robot_status.current_hp;
    }

    int getRealBullet(){
        return real_bullet;
    }

    int getAvailableBullet(){
        return available_bullet;
    }

    int getCaipanBullet(){
        return robot_status.projectile;
    }

    int getArmorAttacked(){
        return robot_status.armor_be_attacked;
    }

    int getOutWar(){
        return robot_status.out_war;
    }

    int getLowEnergy(){
        return robot_status.low_energy;
    }

    int getRfidBaoleiState(){
        return robot_status.rfid_baolei_state;
    }

    int getRfidSupport1State(){
        return robot_status.rfid_support1_state;
    }

    int getRfidSupport2State(){
        return robot_status.rfid_support2_state;
    }

    double getDistance(const Pose &pose1,
        const Pose &pose2) {
        const Point point1 = pose1.position;
        const Point point2 = pose2.position;
        const double dx = point1.x - point2.x;
        const double dy = point1.y - point2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 是否在红方基地区域
    bool getRedSafe(){
        return semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "red_safe_ground");
    }

    // 是否在红方坡
    bool getRedSlope(){
        return semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "red_slope");
    }

    bool getBlueSafe(){
        return semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "blue_safe_ground");
    }

    bool getBlueSlope(){
        return semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "red_slope");
    }

    // 是否在中央区域
    bool getMiddle(){
        return semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "blue_middle");
    }

    // 是否在起伏路段
    bool getUpdown(){
        return (semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "updown1")) || (semantic_map_handler.isPointInArea(current_pose.position.x, current_pose.position.y, "updown2"));
    }

    // 视觉是否识别到了敌人
    int getSeeenemy(){
        return see_enemy_stop;
    }

    // 设置是否电控小陀螺 
    void setTuoluo(int i){
        tuoluo = i;
        ai_tuoluo = 2;
    }

    // 设置是否ai小陀螺，用于同时设置基地血量较低flag
    void setaiTuoluo(int i){
        ai_tuoluo = i;
        base_bad_flag = i;
    }

    // 设置是否ai小陀螺
    void setnormalAiTuoluo(int i){
        tuoluo = 0;
        ai_tuoluo = i;
    }

    // 底盘对齐
    bool control_twist(const geometry_msgs::msg::Quaternion& goal_orientation){
        double target_angle = quaternion_to_yaw(goal_orientation);
        
        // 计算角度差值
        double angle_diff = normalize_angle(target_angle - current_yaw);
        // std::cout<<"current_yaw = "<<current_yaw<<std::endl;
        // std::cout<<"target_yaw = "<<target_angle<<std::endl;
        // std::cout<<"angle_diff = "<<angle_diff<<std::endl;
        
        // 发布速度命令
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

        // 如果角度差值小于容差，停止旋转
        if (std::abs(angle_diff) < angle_tolerance) {
            twist_msg->angular.z = 0.0;
            cmd_vel_publisher_->publish(std::move(twist_msg));
            // std::cout<<"Reach!"<<std::endl;
            return true;
        } else {
            // 根据角度差值的符号决定旋转方向
            twist_msg->angular.z = (angle_diff > 0) ? angular_speed : -angular_speed;
            // std::cout<<"Twisting!"<<std::endl;
        }
        
        cmd_vel_publisher_->publish(std::move(twist_msg));
        return false;
    }

    void setGetBullet(){
        gain_bullet += available_bullet;
        available_bullet = 0;
    }

  private:
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // std::cout<<"get local costmap!"<<std::endl;
        local_costmap = *msg;
    }

    void globalCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // std::cout<<"get global costmap!"<<std::endl;
        global_costmap = *msg;
    }

    void backvisionCallback(const auto_aim_interfaces::msg::Target::ConstSharedPtr& enemy){
        if(enemy->number.empty()){
            back_detect_enemy = 0;
        }
        else{
            back_detect_enemy = 1;
        }
    }

    void visionCallback(const auto_aim_interfaces::msg::TrackedTarget::ConstSharedPtr& enemy)
    {
        get_enemy = enemy->armors_num;

        if(get_enemy == 2){
            see_gongcheng = 1;
        }
        else{
            see_gongcheng = 0;
        }
        
        if(get_enemy){
            detect_enemy = 1;
            
            enemy_pose = getTargetPoseInMap(
                current_pose,
                enemy->position.x,enemy->position.z,-enemy->position.y,
                0,0
            );
            // std::cout<<"enemy_pose x ="<<enemy->position.x<<"enemy_pose y ="<<enemy->position.z<<"enemy_pose z ="<<-enemy->position.y<<std::endl;
        }   
        else{
            detect_enemy = 0;
        }  
        // std::cout<<"detect_enemy="<<detect_enemy<<std::endl;
    }

    void updateGetBullet(){
        // 只有当时间是60的整数倍，且与上次更新时间不同时才更新
        if (game_status.remain_time % 60 == 0 && 
            game_status.remain_time != 420 && 
            game_status.remain_time != last_bullet_update_time) {
            
            available_bullet += 100;
            last_bullet_update_time = game_status.remain_time;  // 记录本次更新时间
        }
    }

    void gameStatusCallback(const tars_msgs::msg::SerialReceive::SharedPtr msg) {
        last_blood = robot_status.current_hp;
        game_status.game_progress = msg->game_progress;
        game_status.remain_time = msg->remain_time;
        game_status.red_outpost_hp = msg->red_outpost_hp;
        game_status.red_base_hp = msg->red_base_hp;
        game_status.blue_outpost_hp = msg->blue_outpost_hp;
        game_status.blue_base_hp = msg->blue_base_hp;
        robot_status.current_hp = msg->current_hp;
        robot_status.projectile = msg->projectile;
        int info = static_cast<int>(msg->sentry_info);
        
        std::vector<uint8_t> bit_array(6);
        for (int i = 0; i < 6; ++i) {
            bit_array[i] = (info >> (i)) & 0x01;
        }
        robot_status.armor_be_attacked = bit_array[0];
        robot_status.out_war = bit_array[1];
        robot_status.low_energy = bit_array[5];
        robot_status.rfid_baolei_state = bit_array[2];
        robot_status.rfid_support1_state = bit_array[3];
        robot_status.rfid_support2_state = bit_array[4];
        // std::cout<<"low="<<robot_status.low_energy<<std::endl;


        bool on_slope = getRedSlope()||getBlueSlope();
        see_enemy_stop = 0;

        // 装甲板被攻击时进行电控小陀螺，不被攻击且脱战时停止小陀螺
        if(game_status.game_progress == 4){
            if ((robot_status.current_hp < last_blood || back_detect_enemy || detect_enemy) && !(on_slope)) {
                if(on_slope && robot_status.current_hp < last_blood){
                    ai_tuoluo = 1;
                    tuoluo = 0;
                    see_enemy_stop = 0;
                }
                else{
                    if(!getUpdown()){
                        see_enemy_stop = 1;
                        tuoluo = 1;
                        ai_tuoluo = 2;
                        last_twist_time = game_status.remain_time;
                    }
                    else{
                        see_enemy_stop = 0;
                        tuoluo = 0;
                        ai_tuoluo = 0;
                    }
                }
            }
            else if (last_twist_time - game_status.remain_time >= 10 && !getMiddle() && !base_bad_flag) {
                tuoluo = 0;
                ai_tuoluo = 0;
            }    
        }
        else{
            tuoluo = 0;
            ai_tuoluo = 0;   
        }

        updateGetBullet();
        real_bullet = 500 - (gain_bullet - robot_status.projectile);
    }

    bool updateCurrentPose() {
        try {
            auto now = ros_node->get_clock()->now();
        
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer->lookupTransform(
                    "map", "chassis_link",
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1));
            current_pose.position.x = transform_stamped.transform.translation.x;
            current_pose.position.y = transform_stamped.transform.translation.y;
            current_pose.position.z = transform_stamped.transform.translation.z;
            current_pose.orientation = transform_stamped.transform.rotation;

            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_yaw = yaw;

            if(getMiddle()){
                ai_tuoluo = 1;
            }
            else if(!base_bad_flag){
                ai_tuoluo = 0;
            }
            return true;
        } catch (tf2::TransformException &ex) {
            return false;
        }
    }

    void pubTuoluo(){
        std_msgs::msg::Int8 tuoluo_msg;
        tuoluo_msg.data = tuoluo;
        tuoluo_publisher_->publish(tuoluo_msg);

        tuoluo_msg.data = ai_tuoluo;
        ai_tuoluo_publisher_->publish(tuoluo_msg);
    }

    // 工具型函数

    // 将欧拉角(rpy)转换为四元数
    tf2::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) 
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return q;
    }
    
    // 获取目标在map坐标系下的位姿
    Pose getTargetPoseInMap(
        const Pose& robot_pose_map,  // 机器人在map中的位姿
        double target_x, double target_y, double target_z,  // 目标在机器人坐标系中的位置
        double target_yaw, double target_pitch, double target_roll = 0.0)  // 目标在机器人坐标系中的姿态
    {
        geometry_msgs::msg::PoseStamped pose_in_base;
        pose_in_base.header.frame_id = "base_frame";  // 使用参数指定的源坐标系
        pose_in_base.pose.position.x = target_x;
        pose_in_base.pose.position.y = target_y;
        pose_in_base.pose.position.z = target_z;

        try
        {
        // 执行坐标转换
        geometry_msgs::msg::PoseStamped pose_in_map;
        pose_in_map = tf_buffer->transform(pose_in_base, "map");
        return pose_in_map.pose;
        }
        catch (tf2::TransformException &ex)
        {
        }
    }

    // 将四元数转换为偏航角
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
        // 四元数转欧拉角 (yaw)
        // yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    // 规范化角度到[-π,π]区间
    double normalize_angle(double angle)
    {
        while (angle > M_PI) {
        angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
        angle += 2.0 * M_PI;
        }
        return angle;
    }

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub;
    rclcpp::Subscription<auto_aim_interfaces::msg::TrackedTarget>::SharedPtr enemy_pose_sub;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr back_enemy_pose_sub;
    rclcpp::Subscription<tars_msgs::msg::SerialReceive>::SharedPtr game_status_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr attack_pose_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_attack_pose_vis_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr tuoluo_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr ai_tuoluo_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    SemanticMapHandler semantic_map_handler;

    // 机器人自身参数
    GameStatus game_status;
    RobotStatus robot_status;
    Pose current_pose;
    int available_bullet = 0; // 可领取的弹量
    int gain_bullet = 300; // 累积获得发弹量
    int real_bullet = 500; // 假设弹仓上限500后的真实弹量

    int last_bullet_update_time = 800;
    int last_twist_time = 800;
    int last_blood = 400;

    // 各种陀螺参数
    int tuoluo = 0; // 电控小陀螺
    int ai_tuoluo = 0; // ai小陀螺
    int see_enemy_stop = 0;
    int base_bad_flag = 0;

    // 追击相关参数
    nav_msgs::msg::OccupancyGrid local_costmap, global_costmap;
    int get_enemy = 0;
    int detect_enemy = 0;
    int back_detect_enemy=0;
    Pose enemy_pose;
    bool see_gongcheng = 0;

    // 旋转指定角度相关参数
    double angular_speed = 2;
    double angle_tolerance = 0.1;
    double current_yaw;
    double angle_max = 2*M_PI;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_tuoluo;
};

}