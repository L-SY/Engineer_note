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
## 初步了解
帮助进行运动学分析解算的一个ros包

Movelt 由**TRACLab，IKFast**进行逆运动学解算
KDL正运动学解算

路径规划：OMPL、CHOMP、SBPL、、、
碰撞检测：FCL、PCD