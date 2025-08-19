#!/usr/bin/env python3
import rospy
from flask import Flask, request, jsonify
from threading import Thread
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json

class TaskReceiver:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('task_receiver_server', anonymous=True)
        
        # 创建任务发布者
        self.task_pub = rospy.Publisher('/task_command', String, queue_size=10)
        
        # 创建目标点发布者
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # 设置 Flask 应用
        self.app = Flask(__name__)
        self.setup_routes()
        
        # 服务器配置
        self.host = rospy.get_param('~host', '0.0.0.0')
        self.port = rospy.get_param('~port', 5000)
        
        # 启动 Flask 服务器线程
        self.server_thread = Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        rospy.loginfo(f"任务接收服务已启动，监听 {self.host}:{self.port}")
    
    def setup_routes(self):
        """设置 API 路由"""
        @self.app.route('/task', methods=['POST'])
        def receive_task():
            """接收任务信息"""
            try:
                # 解析 JSON 数据
                data = request.get_json()
                rospy.loginfo(f"收到任务请求: {json.dumps(data, indent=2)}")
                
                # 验证必要字段
                if 'task_id' not in data or 'command' not in data:
                    return jsonify({"status": "error", "message": "缺少必要字段: task_id 或 command"}), 400
                
                # 处理不同命令类型
                if data['command'] == 'navigate':
                    return self.handle_navigation(data)
                elif data['command'] == 'inspect':
                    return self.handle_inspection(data)
                elif data['command'] == 'stop':
                    return self.handle_stop(data)
                else:
                    return jsonify({"status": "error", "message": f"未知命令: {data['command']}"}), 400
                    
            except Exception as e:
                rospy.logerr(f"处理任务请求时出错: {str(e)}")
                return jsonify({"status": "error", "message": str(e)}), 500
        
        @self.app.route('/health', methods=['GET'])
        def health_check():
            """健康检查端点"""
            return jsonify({"status": "ok", "message": "服务运行正常"})
    
    def handle_navigation(self, data):
        """处理导航任务"""
        # 验证必要字段
        if 'target' not in data:
            return jsonify({"status": "error", "message": "导航任务缺少目标位置"}), 400
        
        # 发布导航目标
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = data['target']['x']
        goal.pose.position.y = data['target']['y']
        goal.pose.position.z = 0.0
        
        # 设置方向（如果有）
        if 'orientation' in data['target']:
            goal.pose.orientation.z = data['target']['orientation']['z']
            goal.pose.orientation.w = data['target']['orientation']['w']
        else:
            goal.pose.orientation.w = 1.0  # 默认朝向
        
        self.goal_pub.publish(goal)
        
        # 发布任务命令
        task_msg = String()
        task_msg.data = json.dumps({
            "task_id": data['task_id'],
            "command": "navigate",
            "target": data['target']
        })
        self.task_pub.publish(task_msg)
        
        return jsonify({
            "status": "success",
            "message": f"导航任务已接收: 前往 ({data['target']['x']}, {data['target']['y']})"
        })
    
    def handle_inspection(self, data):
        """处理巡检任务"""
        # 验证必要字段
        if 'points' not in data or not data['points']:
            return jsonify({"status": "error", "message": "巡检任务缺少路径点"}), 400
        
        # 发布任务命令
        task_msg = String()
        task_msg.data = json.dumps({
            "task_id": data['task_id'],
            "command": "inspect",
            "points": data['points']
        })
        self.task_pub.publish(task_msg)
        
        return jsonify({
            "status": "success",
            "message": f"巡检任务已接收: {len(data['points'])}个路径点"
        })
    
    def handle_stop(self, data):
        """处理停止任务"""
        # 发布停止命令
        task_msg = String()
        task_msg.data = json.dumps({
            "task_id": data['task_id'],
            "command": "stop"
        })
        self.task_pub.publish(task_msg)
        
        return jsonify({
            "status": "success",
            "message": "停止任务已接收"
        })
    
    def run_server(self):
        """运行 Flask 服务器"""
        self.app.run(host=self.host, port=self.port, threaded=True)

if __name__ == '__main__':
    try:
        receiver = TaskReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务接收服务已关闭")