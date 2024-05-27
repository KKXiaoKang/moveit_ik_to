## kuavo机器人moveit目标检测规划（工厂模式规划生产/规划消费模式）
* moveit_setting_pkg : moveit配置文件夹
```bash
biped_s4:                机器人模型功能包 
kuavo4_arm_traj_planner: moveit pythonAPI 调取服务发送轨迹功能包
kuavo40_moveit_config:   moveit 配置功能包（新带机器人虚拟关节手臂）
traj_planner：           轨迹规划工厂模式功能包
``` 
---