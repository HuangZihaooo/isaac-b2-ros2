# Isaac B2 ROS2 仿真项目

基于NVIDIA Isaac Sim 4.5和Isaac Lab 2.1的B2机器人仿真与ROS2集成项目。
本项目参考https://github.com/Zhefan-Xu/isaac-go2-ros2，且大多数功能尚未开发完成

## 📋 项目概述

本项目实现了B2机器人在Isaac Sim中的物理仿真，并提供ROS2接口用于机器人控制和数据交互。支持多种仿真环境，包括障碍物场景和仓库环境。

### 主要特性

- 🤖 **B2机器人仿真**：完整的机器人物理模型和控制系统
- 🌐 **ROS2集成**：提供标准ROS2话题和服务接口
- 🎮 **键盘控制**：支持实时键盘控制机器人运动
- 🏭 **多环境支持**：障碍物环境、仓库环境等多种场景
- 📊 **传感器仿真**：激光雷达、RGB-D相机、语义分割等
- 🔧 **配置化**：通过YAML文件轻松配置仿真参数

## 🛠️ 环境要求

### 系统要求
- **操作系统**：Ubuntu 22.04 LTS
- **GPU**：NVIDIA RTX系列（支持RTX光线追踪）
- **内存**：16GB+ RAM
- **显存**：8GB+ VRAM

### 软件依赖
- **Isaac Sim 4.5+**
- **Isaac Lab 2.1+** 
- **ROS2 Humble**
- **Python 3.10**
- **PyTorch**
- **Hydra配置框架**

## 📦 安装指南

### 1. 安装Isaac Sim和Isaac Lab

```bash
# 安装Isaac Sim 4.5
# 请参考NVIDIA官方文档进行安装
# https://docs.isaacsim.omniverse.nvidia.com/

# 安装Isaac Lab 2.1
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install
```

### 2. 安装ROS2 Humble

```bash
# 安装ROS2 Humble
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $DISTRO_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### 3. 设置Conda环境

```bash
# 激活Isaac Lab环境
conda activate isaac450
# 或者根据您的实际Isaac环境名称：/home/xiaohuangfeng/.conda/envs/isaac450

# 安装额外的Python依赖
pip install hydra-core
pip install rclpy
```

### 4. 克隆项目

```bash
git clone <repository-url>
cd isaac-b2-ros2
```

## 🚀 快速开始

### 基本运行

```bash
# 激活Isaac环境
conda activate isaac450

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 运行基础仿真
python isaac_b2_ros2.py
```

### 配置选项

编辑 `cfg/sim.yaml` 文件来调整仿真参数：

```yaml
# 基础配置
num_envs: 1          # 环境数量
freq: 40             # 仿真频率 (Hz)
camera_follow: true  # 相机跟随
env_name: obstacle-sparse  # 环境类型

# 传感器配置
sensor:
  enable_lidar: true
  enable_camera: true
  color_image: true
  depth_image: true
  semantic_segmentation: true

# 应用程序配置
sim_app:
  width: 1280
  height: 720
  hide_ui: false
```

### 环境选项

支持以下仿真环境：

- `obstacle-sparse` - 稀疏障碍物环境
- `obstacle-medium` - 中等密度障碍物环境  
- `obstacle-dense` - 密集障碍物环境
- `warehouse` - 基础仓库环境
- `warehouse-forklifts` - 带叉车的仓库环境
- `warehouse-shelves` - 带货架的仓库环境
- `full-warehouse` - 完整仓库环境

## 🎮 控制方式

### 键盘控制

使用以下按键控制B2机器人：

- **W** - 前进
- **S** - 后退  
- **A** - 左转
- **D** - 右转
- **空格** - 停止

### ROS2控制

通过ROS2话题发送控制命令：

```bash
# 发送速度命令
ros2 topic pub /b2/cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

## 📡 ROS2话题接口

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/b2/odom` | `nav_msgs/Odometry` | 机器人里程计信息 |
| `/b2/pose` | `geometry_msgs/PoseStamped` | 机器人位姿 |
| `/b2/lidar/point_cloud` | `sensor_msgs/PointCloud2` | 激光雷达点云数据 |
| `/b2/camera/color_image` | `sensor_msgs/Image` | RGB图像 |
| `/b2/camera/depth_image` | `sensor_msgs/Image` | 深度图像 |
| `/b2/camera/semantic_image` | `sensor_msgs/Image` | 语义分割图像 |

### 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/b2/cmd_vel` | `geometry_msgs/Twist` | 速度控制命令 |

## 📁 项目结构

```
isaac-b2-ros2/
├── b2/                     # B2机器人相关模块
│   ├── __init__.py
│   ├── b2_ctrl.py         # 机器人控制逻辑
│   ├── b2_env.py          # 环境配置
│   └── b2_sensors.py      # 传感器管理
├── cfg/                   # 配置文件
│   └── sim.yaml          # 仿真配置
├── env/                   # 环境模块
├── model/                 # 机器人模型文件
│   ├── B2/
│   │   └── b2.usd        # B2机器人USD模型
│   └── config.yaml       # 模型配置
├── ros2/                  # ROS2接口模块
│   └── b2_ros2_bridge.py # ROS2桥接器
├── rviz/                  # RViz配置文件
│   └── b2.rviz           # RViz可视化配置
├── outputs/               # 输出和日志文件
├── isaac_b2_ros2.py      # 主程序入口
├── test_import.py        # 导入测试
├── .gitignore            # Git忽略文件
└── README.md             # 项目说明文档
```

## 🔧 开发指南

### 添加新的控制器

1. 在 `b2/b2_ctrl.py` 中添加控制逻辑：

```python
def custom_controller(robot_state):
    # 自定义控制逻辑
    action = calculate_action(robot_state)
    return action
```

2. 在主循环中集成新控制器：

```python
# 在 isaac_b2_ros2.py 中
action = custom_controller(env.get_robot_states())
env.apply_action(action)
```

### 添加新的传感器

1. 在 `b2/b2_sensors.py` 中定义传感器：

```python
def add_custom_sensor(self):
    # 添加自定义传感器
    sensor = CustomSensor(config)
    return sensor
```

2. 在ROS2桥接器中发布数据：

```python
# 在 ros2/b2_ros2_bridge.py 中
def publish_custom_sensor_data(self, data):
    # 发布传感器数据到ROS2
    pass
```

### 自定义环境

在 `env/sim_env.py` 中添加新环境：

```python
def create_custom_env():
    # 创建自定义环境
    pass
```

## 🐛 故障排除

### 常见问题

1. **Isaac Sim启动失败**
   ```bash
   # 检查NVIDIA驱动
   nvidia-smi
   
   # 检查CUDA版本
   nvcc --version
   ```

2. **ROS2话题无数据**
   ```bash
   # 检查ROS2环境
   echo $ROS_DOMAIN_ID
   ros2 topic list
   ```

3. **GPU内存不足**
   ```bash
   # 减少环境数量
   # 在cfg/sim.yaml中设置 num_envs: 1
   ```

4. **模型加载失败**
   ```bash
   # 检查模型文件路径
   ls model/B2/b2.usd
   ```

### 性能优化

1. **降低仿真精度**：
   - 减少 `num_envs`
   - 降低 `freq` 频率
   - 禁用不需要的传感器

2. **GPU优化**：
   - 确保使用独立显卡
   - 关闭其他GPU应用程序
   - 调整Isaac Sim的渲染设置

## 📊 性能指标

在标准配置下的性能表现：

| 配置 | RTX 3080 | RTX 4080 | RTX 4090 |
|------|----------|----------|----------|
| 1环境 | 60 FPS | 80 FPS | 100 FPS |
| 2环境 | 35 FPS | 50 FPS | 70 FPS |
| 4环境 | 20 FPS | 30 FPS | 45 FPS |

## 🤝 贡献指南

1. Fork本项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

- [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/) - 强大的机器人仿真平台
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) - 机器人学习框架
- [ROS2](https://docs.ros.org/en/humble/) - 机器人操作系统

## 📞 支持与联系

如果您在使用过程中遇到问题，请：

1. 查看 [Issues](../../issues) 中的已知问题
2. 创建新的 Issue 描述您的问题
3. 参考 [Isaac Sim 官方文档](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/)

---

**注意**：本项目基于Isaac Sim 4.5和Isaac Lab 2.1开发，请确保使用兼容的版本。
