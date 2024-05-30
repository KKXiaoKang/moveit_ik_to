## kuavo机器人moveit目标检测规划（工厂模式规划生产/规划消费模式）
* moveit_setting_pkg : moveit配置文件夹
```bash
biped_s4:                机器人模型功能包 
kuavo4_arm_traj_planner: moveit pythonAPI 调取服务发送轨迹功能包
kuavo40_moveit_config:   moveit 配置功能包（新带机器人虚拟关节手臂）
traj_planner：           轨迹规划工厂模式功能包
``` 
---
## 启动方式
```bash 
roslaunch kuavo40_moveit_config demo.launch # 启动moveit配置仿真器，等待You can start planning now! 这个字符出现才可以规划

cd ~/moveit_ik_to/src/traj_planner        # 通过pyton3 启动对应的脚本 目前目标检测
python3 test_axis_pose_moveit_executor.py # 输入目标检测结果的末端pose 但是每次规划都是从 当前手臂位置 开始
```