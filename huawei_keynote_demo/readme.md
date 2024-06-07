## 文件解析
```bash
├── huawei_keynote_framework_0    # 里面的moveit规划专为固定点的提供 规划冗余度为0 
├── huawei_keynote_framework_0.1  # 里面的moveit规划专为视觉抓取提供 规划冗余度为0.1
└── readme.md

├── open_moveit_noCV_control.sh    # 无视觉moveit固定点脚本 冗余度0 
├── open_moveit_vision_control.sh  # 纯视觉遥控moveit脚本  冗余度0.1 
├── open_qiangnao_hand_control.sh  # 灵巧手 + 头部 追踪脚本 

```
## 使用方法
```bash
# 新建三个teminal窗口
./open_moveit_noCV_control.sh 
./open_moveit_vision_control.sh
./open_qiangnao_hand_control.sh
```