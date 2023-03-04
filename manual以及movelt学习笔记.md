# MANUAL
## 功能
为了实现键鼠和遥控器的操控
## 代码结构
本质上为先定义各个按键的组合名字
设置dbus回调函数，后更新数据的类成员
设置sendcommand的类成员函数
后定义各个按键所能实现的功能，每个功能本质上是给相应数据赋值
每个功能对应一个sendcommand
后发送出去

# Movelt

参考教程https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

## 初步了解
帮助进行运动学分析解算的一个ros包

Movelt 由**TRACLab，IKFast**进行逆运动学解算
KDL正运动学解算

路径规划：OMPL、CHOMP、SBPL、、、
碰撞检测：FCL、PCD


## 配置环境
具体教程https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
配置movelt环境途中用到了一个不同于catkin名为wstool的编译工具(不确定)
__注意：配置实例环境时候不要随意使用
```shell
echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
```
会导致rm_ws又要手动配置，开启示例时候应该手动配置环境变量__

## 打开实例

在开始学习之前，先打开官方提供的实例
```shell
mon launch panda_moveit_config demo.launch rviz_tutorial:=true
```

## RViz中的交互
不知道为何我没法让其碰撞变红
超出了工作空间也并没有发生啥
利用下方的joint选项卡可以只控制单个关节 “null space exploration” 的调节可以让爪子保持不动只动关节

<table><tr><td bgcolor=DarkSeaGreen>本质的运动计划便是确定一个起点和终点，然后过程确保机械臂不会碰到他自己，勾选show tail选项可以看到运动轨迹</td></tr></table>

### 各种可视化
打开Trajectory Slider可以查看各个时间点机械臂的状态
使用笛卡尔路径，再点击plan可以实现笛卡尔运动让爪子进行线性运动

## Movelt C++的接口学习

### 运行代码
第一个shell
```
mon launch panda_moveit_config demo.launch
```
第二个shell
```
mon launch moveit_tutorials move_group_interface_tutorial.launch
```

### 代码理解
#### 类的理解

*JointModelGroup*用于储存各个计划组和关节模型组
```c++
static const std::string PLANNING_GROUP = "panda_arm";
```
*MoveGroupInterface*为了方便改变计划组的名字而设置

```c++
moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
```
*PlanningSceneInterface*用于添加或者删除碰撞箱和障碍物
```c++
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
```
*Raw pointers*用于指向计划组来提升性能
```c++
const moveit::core::JointModelGroup* joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
```

#### 获取信息
利用ROS_INFO_NAMED获取机器人的参考系和执行器link名称还可以获得机器人所有族的名称

#### 开始演示
```
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
```

##### Planning the goal
本质上使用以下模板是设定末端执行器姿势的goal坐标
```c++
geometry_msgs::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.2;
target_pose1.position.z = 0.5;
move_group_interface.setPoseTarget(target_pose1);
```
然后调用planner计算轨迹,同时这一步骤也储存了刚刚写的计划组
```c++
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
```
##### Visualizing plans
下一段代码就是用于在rivz上进行可视化可称为Visualizing plans
```c++
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```
##### Executing
后调用进行计算
```c++
move_group_interface.execute(my_plan);
```
**以上便是计划并且进行计划的代码解释**
最后进行移动
```c++
move_group_interface.move();
```
**总结：此之上就是调用各个接口在代码层面实现计划并且运动的方法，使用*two-step plan+execute*的方法**

### 设定joint space
设定一个指针，用于指向储存joint group的加速度/速度/位置的向量
```c++
moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
```
后创建向量使用接口获取所有的数据，使指针指向这个向量
```c++
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
```
修改了一个关节的值时候在进行可视化
```c++
joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians
move_group_interface.setJointValueTarget(joint_group_positions);
```
此段用于设置加速度和线速度，或者也可用moveit_config里的joint_limits.yaml里进行修改，或是在rviz中都可修改
```c++
move_group_interface.setMaxVelocityScalingFactor(0.05);
move_group_interface.setMaxAccelerationScalingFactor(0.05);

success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
```
在rviz上可视化的代码与上面提到的基本一致
```c++
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```
之后为了让其正常运行，要定义实际上存在的物理量影响因素(可能？)此为一个赋值的过程
```c++
moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;//hixbox?
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
```
后便是要将其设置为实际的tolerance
```c++
moveit_msgs::Constraints test_constraints;
test_constraints.orientation_constraints.push_back(ocm);
move_group_interface.setPathConstraints(test_constraints);//the key codes
```
### Enforce Planning in Joint Space
在Moveit中有两种计划问题的方式分别为*joint space*和*cartesian space*在参数文件**ompl_planning.yaml**中修改
```xml
enforce_joint_model_state_space:true
```
可以强制所有的计划都使用*joint space*模式

#### 两种模式具有的特点
##### *cartesian space*
作为默认模式具有方向约束采样(不太懂啥意思)，便可以调用IK来作为采样生成器
##### *joint space*
其拒绝采样，所以计划时间会被大大增加

#### Enforcing
因为要应用就的目标计划，所以重新设定开始位置
```c++
moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
geometry_msgs::Pose start_pose2;
start_pose2.orientation.w = 1.0;
start_pose2.position.x = 0.55;
start_pose2.position.y = -0.05;
start_pose2.position.z = 0.8;
start_state.setFromIK(joint_model_group, start_pose2);
move_group_interface.setStartState(start_state);
```
直接应用
```c++
move_group_interface.setPoseTarget(target_pose1);
```
后将计划时间设定为10s让其能够有充足时间计算完成
```c++
move_group_interface.setPlanningTime(10.0);

success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
```
最后在Rviz中可视化
```c++
visual_tools.deleteAllMarkers();
visual_tools.publishAxisLabeled(start_pose2, "start");
visual_tools.publishAxisLabeled(target_pose1, "goal");
visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("next step");
```
用path constraint完成后记得及时清除
```c++
move_group_interface.clearPathConstraints();
```


