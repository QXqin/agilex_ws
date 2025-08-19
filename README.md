# LIMO ����������������ָ��

## ? ǰ��������װ
�ڿ�ʼǰ��ȷ�����������������װ��

bash

sudo apt-get install ros-melodic-rosbridge-server \

ros-melodic-web-video-server \

ros-melodic-rosapi \

ros-melodic-rosauth \

python3-websocket

pip3 install tornado

## ? һ����������
### 1. �ļ�����
- �� `limo_startup.launch` ���Ƶ���  
  `~/agilex_ws/src/limo_ros/limo_bringup/launch/`
- ��������ļ�����������Ŀ¼

### 2. Ȩ������
bash

chmod +x LIMO_Startup.desktop

chmod +x start_limo.sh

### 3. ������ʽ����ѡ��һ��
- **�����ݷ�ʽ**��˫�� `LIMO_Startup.desktop`
- **�ն���������**��
bash

roslaunch limo_bringup limo_startup.launch

## ? ��վ������������
> ?? ÿ��ʹ��ǰ��ִ�л�����ʼ����
> ```bash
> source ~/agilex_ws/devel/setup.bash
> roscore
> ```

### ���ķ�����������
| ģ�� | ���� |
|------|------|
| ���̿��� | `roslaunch limo_base limo_base.launch` |
| ROSͨ���� | `roslaunch rosbridge_server rosbridge_websocket.launch` |
| ��Ƶ������ | `rosrun web_video_server web_video_server _port:=8080` |
| USB����ͷ | `rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv` |

## ? ����������ϵͳ
### �����ϱ�����
bash

��������ϵͳ
roslaunch limo_bringup limo_navigation_diff.launch

��ʼ������ͨ�ţ��ر�TF�任������
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false

����λ���ϱ�����
roslaunch limo_reporting pose_reporter.launch

### ? ���Ѳ��ϵͳ
����·��Ѳ������
bash

rosrun single_point single_point_nav.py

