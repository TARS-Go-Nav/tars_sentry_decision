吉林大学TARS_Go战队2025赛季哨兵决策部分 <br/>
框架基于[RoboRTS](https://github.com/RoboMaster/RoboRTS)中roborts_decision部分 <br/>
仅在ros2 humble环境下实验

## 文件架构

```
├── auto_aim_interfaces # 视觉使用的自定义信息，由于有追击行为故放在这
│   ├── msg
│   │   ├── Armor.msg
│   │   ├── Armors.msg
│   │   ├── DebugArmor.msg
│   │   ├── DebugArmors.msg
│   │   ├── DebugLight.msg
│   │   ├── DebugLights.msg
│   │   ├── Target.msg
│   │   ├── TrackedTarget.msg
│   │   └── TrackerInfo.msg
│   ├── CMakeLists.txt
│   └── package.xml
├── simple_rf # 测试行为树用的虚拟裁判系统ui
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── mainwindow.h
│   │   └── qnode.h
│   ├── package.xml
│   ├── resource
│   ├── src
│   │   ├── main.cpp
│   │   ├── mainwindow.cpp
│   │   └── qnode.cpp
│   └── ui
│       └── main_win.ui
├── tars_decision # 行为树本体
│   ├── actions # 行为节点
│   │   ├── follow_action.hpp
│   │   ├── get_supply_action.hpp
│   │   ├── goal_action.hpp
│   │   ├── stop_action.hpp
│   │   ├── twist_action.hpp
│   │   └── wait_normal_action.hpp
│   ├── behavior_tree # 行为树底层实现
│   │   ├── behavior_node.hpp
│   │   ├── behavior_state.hpp
│   │   └── behavior_tree.hpp
│   ├── blackboard 
│   │   ├── blackboard.hpp # 黑板
│   │   └── semantic_map.hpp # 语义地图
│   ├── config
│   │   ├── dog.yaml # 行为树参数文件
│   │   └── rmuc_map.xml # 语义地图配置文件
│   ├── executor
│   │   └── chassis_executor.hpp # 负责和nav2交互
│   ├── launch
│   │   └── start.launch.py # 一个launch示例
│   ├── behavior_test.cpp # 行为树示例
│   ├── CMakeLists.txt
│   └── package.xml
└── tars_msgs # 串口自定义信息
    ├── msg
    │   ├── SerialReceive.msg # 下位机发送数据
    │   └── SerialSend.msg # 给下位机的数据
    ├── CMakeLists.txt
    └── package.xml
```

## 如何食用

### 预处理

1.安装依赖
```sh
cd 工作空间目录
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
2.编译
```sh
colcon build --symlink-install
```

### 虚拟裁判系统

```sh
source install/setup.bash
ros2 run simple_rf simple_rf
```
顺利的话会看到屏幕上出现一个丑丑的ui界面，更改比赛进程至4即可开始倒计时，其余数据均可随意更改

### 行为树

```sh
source install/setup.bash
ros2 run tars_decision behavior_test_node
```
输入想测试的行为序号即可

## 说明


| 话题名 | 类型 | 说明 | 订阅/发布 |
|:-----:|:---:|:----:|:----:|
|`/local_costmap/costmap`|`nav_msgs::msg::OccupancyGrid`|追击使用的局部代价地图|订阅|
|`/global_costmap/costmap`|`nav_msgs::msg::OccupancyGrid`|追击使用的全局代价地图|订阅|
|`/front/tracker/target`|`auto_aim_interfaces::msg::TrackedTarget`|哨兵前视摄像头识别信息|订阅|
|`/back/targeter/target`|`auto_aim_interfaces::msg::Target`|哨兵后视摄像头识别信息|订阅|
|`/serial_receive`|`tars_msgs::msg::SerialReceive`|串口接收信息（包含裁判系统数据）|订阅|
|`/attack_pose_vis`|`visualization_msgs::msg::MarkerArray`|追击待选点可视化|发布|
|`/final_attack_pose_vis`|`visualization_msgs::msg::Marker`|追击最终选择点可视化|发布|
|`/tuoluo`|`std_msgs::msg::Int8`|电控小陀螺|发布|
|`/ai_tuoluo`|`std_msgs::msg::Int8`|ai小陀螺|发布|
|`/cmd_vel_chassis`|`geometry_msgs::msg::Twist`|底盘对齐时发布角速度话题|发布|

#### 行为：

##### goal action：

导航到目标点的行为，为了防止哨兵卡住所以一直发送目标点，通过和目标点距离判断是否到达目标点（不一直发送目标点的话直接通过chassis_executor->Update()获得导航状态也是ok的）

##### wait action：

普通的取消导航停在原地行为，停多久完全看你设计 <br/>

##### stop action：

普通的取消导航行为

##### get supply action：

和名字一样是在补给点使用的行为，取消导航并停止小陀螺，直到血满

##### twist action：

由于导航包基于虚假底盘坐标系（一直指向地图正方向）进行，故在导航层面对底盘角度不可控，为实现过洞在行为树中加入了底盘对齐行为，即twist action，通过直接发布角速度并同时读取真实底盘坐标系实现

##### follow action：

完全参考四川大学火锅战队的追击行为，得到视觉发来的相机坐标系中敌人坐标后转换到世界坐标系中（这部分在黑板中实现），在代价地图中分析找到最佳追击点并前往

### 黑板中控制陀螺相关参数：

哨兵有两种模式的小陀螺：一是由电控进行的高速小陀螺，在行为树中控制参数为tuoluo，该数值通过话题直接发送给串口包控制小陀螺，为1时开启电控小陀螺，为0时关闭电控小陀螺；二是通过上位机发送给下位机角速度实现的小陀螺，又称ai陀螺，小陀螺转速和电控陀螺相比慢很多，在行为树中控制参数为ai_tuoluo，该数值通过话题发送给速度转换包，为2时表示电控小陀螺开启，速度较低、为1时表示ai陀螺开启，速度中等、为0时即无小陀螺，速度较快

### 黑板中有关子弹的参数：

参数具体含义见注释，在每一个分钟增加可获得子弹量判断是否可以前往补给点补弹，同时根据实际弹舱大小计算实际可发弹量

### 语义地图：

感谢华南理工大学华南虎战队在青工会上的分享。语义地图在本仓库主要用于判断应该进行什么形式的小陀螺，例如在中央高地区域慢速小陀螺走动等。仅需在xml文件中配置好区域边界角点并将xml文件交给黑板即可使用。

关注TARS_Go战队导航组谢谢喵 <br/>
[tars cat](https://tars-cat.github.io/) <br/>
虽然之后要读研了不打rm了，但研究生阶段研究内容还是与rm强相关的，有想交流的人欢迎添加qq1305713727
