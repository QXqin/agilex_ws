# LIMO 机器人启动与配置指南

## 📦 前置依赖安装
在开始前请确保已完成以下依赖安装：

```bash
sudo apt-get install ros-melodic-rosbridge-server \

ros-melodic-web-video-server \

ros-melodic-rosapi \

ros-melodic-rosauth \

python3-websocket

pip3 install tornado
```
## 🚀 一键启动配置
### 1. 文件部署
- 将 `limo_startup.launch` 复制到：  
  `~/agilex_ws/src/limo_ros/limo_bringup/launch/`
- 其他组件文件放置于桌面目录

### 2. 权限设置
```bash
chmod +x LIMO_Startup.desktop

chmod +x start_limo.sh
```
### 3. 启动方式（任选其一）
- **桌面快捷方式**：双击 `LIMO_Startup.desktop`
- **终端命令启动**：
```bash
roslaunch limo_bringup limo_startup.launch
```
## 🌐 网站服务启动流程
> ⚠️ 每次使用前需执行环境初始化：
> ```bash
> source ~/agilex_ws/devel/setup.bash
> roscore
> ```

### 核心服务启动命令
| 模块 | 命令 |
|------|------|
| 底盘控制 | ```roslaunch limo_base limo_base.launch``` |
| ROS通信桥 | ```roslaunch rosbridge_server rosbridge_websocket.launch``` |
| 视频流服务 | ```rosrun web_video_server web_video_server _port:=8080``` |
| USB摄像头 | ```rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv``` |

## 🧭 导航与坐标系统
### 坐标上报配置
```bash
启动导航系统
roslaunch limo_bringup limo_navigation_diff.launch

初始化底盘通信（关闭TF变换发布）
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false

启动位置上报服务
roslaunch limo_reporting pose_reporter.launch
```
### 🔁 多点巡航系统
启动路径巡航程序：
```bash
rosrun single_point single_point_nav.py
```
