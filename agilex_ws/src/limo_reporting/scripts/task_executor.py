#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import json
import actionlib
import threading
import requests
import time
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Point

class TaskExecutor:
    def __init__(self):
        rospy.init_node('task_executor')
        
        # ��ʼ����������״̬����������ִ�У�
        self.task_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        self.current_pose = None
        self.current_task = None
        self.task_active = False
        self.task_paused = False
        
        # Configuration parameters
        self.robot_id = rospy.get_param('~robot_id', 'limo-001')
        self.api_url = rospy.get_param('~api_url', 'http://39.107.253.156/xunjian/api/report/coordinates')
        self.report_interval = rospy.get_param('~report_interval', 1.0)  # λ���ϱ����
        
        # ��ʼ�����������ڶ���֮ǰ��
        self.status_pub = rospy.Publisher('/task_status', String, queue_size=10)
        
        # ���Ļ���
        rospy.Subscriber('/task_commands', String, self.command_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Create action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        rospy.loginfo("Task executor node started")

    def pose_callback(self, msg):
        """Handle pose updates"""
        with self.pose_lock:
            self.current_pose = msg.pose.pose

    def command_callback(self, msg):
        """Handle received commands"""
        try:
            command = json.loads(msg.data)
            cmd_type = command.get('command')
            data = command.get('data', {})
            
            if cmd_type == 'start':
                self.handle_start_command(data)
            else:
                rospy.logwarn("Unknown command type: %s", cmd_type)
                
        except Exception as e:
            rospy.logerr("Error processing command: %s", str(e))

    def handle_start_command(self, data):
        """Handle task start command"""
        with self.task_lock:
            if self.task_active:
                rospy.logwarn("Task already in progress. Ignoring new start command.")
                return
                
            rospy.loginfo("Starting new task: TaskID=%s", data['taskId'])
            self.current_task = data
            self.task_active = True
            self.task_paused = False
            
            # Publish task start status
            self.publish_task_status("RUNNING")
            
            # ����λ���ϱ��߳�
            self.report_thread = threading.Thread(target=self.report_loop)
            self.report_thread.daemon = True
            self.report_thread.start()
            
            # Start task execution thread
            task_thread = threading.Thread(target=self.execute_task)
            task_thread.daemon = True
            task_thread.start()

    def report_loop(self):
        """Periodically report pose during task execution"""
        rate = rospy.Rate(1.0 / self.report_interval)
        
        while not rospy.is_shutdown() and self.task_active:
            if self.current_pose:
                try:
                    self.report_position()
                except Exception as e:
                    rospy.logerr("Failed to report pose: %s" % str(e))
            rate.sleep()

    def report_position(self):
        """Report position to backend"""
        with self.pose_lock:
            if not self.current_pose:
                return
                
            position = self.current_pose.position
            orientation = self.current_pose.orientation
            
            # Prepare data
            data = {
                "robotId": self.robot_id,
                "taskId": self.current_task['taskId'],
                "positionX": position.x,
                "positionY": position.y,
                "positionZ": position.z,
                "orientationX": orientation.x,
                "orientationY": orientation.y,
                "orientationZ": orientation.z,
                "orientationW": orientation.w
            }
            
            # Send request
            headers = {"Content-Type": "application/json"}
            try:
                response = requests.post(self.api_url, json=data, headers=headers, timeout=3.0)
                
                if response.status_code == 200:
                    rospy.loginfo("Pose reported successfully")
                else:
                    rospy.logwarn("Pose report failed: status code %d" % response.status_code)
            except Exception as e:
                rospy.logerr("HTTP request failed: %s" % str(e))

    def execute_task(self):
        """Execute task logic"""
        try:
            # ��ȡС����ǰλ����Ϊ��ʼ��
            start_point = self.get_current_position()
            if not start_point:
                rospy.logerr("Failed to get current position, aborting task")
                self.publish_task_status("FAILED", "Cannot get current position")
                return
                
            # �����������
            task_id = self.current_task['taskId']
            task_cycle = self.current_task['taskCycle']
            task_laps = self.current_task.get('taskLaps', 1)  # Ĭ��1Ȧ
            obstacle_method = self.current_task.get('obstacleHandlingMethod', 'skip')  # Ĭ������
            waypoints = self.current_task['taskPoints']
            
            # ��С����ǰλ����Ϊ·�����
            full_path = [{
                'x': start_point.x,
                'y': start_point.y,
                'is_start': True
            }] + waypoints
            
            rospy.loginfo("Task parameters: Cycles=%d, Waypoints=%d", 
                         task_cycle, len(full_path))
            
            # ִ������Ȧ��
            for cycle in range(task_cycle):
                if not self.task_active:
                    break
                    
                rospy.loginfo("Starting cycle %d/%d", cycle+1, task_cycle)
                
                # ������������·���㣨������ʼ�㣩
                for i, point in enumerate(full_path):
                    if not self.task_active:
                        break
                        
                    # ������ʼ�㣨������㣩
                    if i == 0 and point.get('is_start'):
                        rospy.loginfo("Starting from current position (%.2f, %.2f)", 
                                     point['x'], point['y'])
                        continue
                        
                    # �����ͣ״̬
                    if self.task_paused:
                        rospy.loginfo("Task paused, waiting for resume...")
                        self.publish_task_status("PAUSED")
                        while self.task_paused and self.task_active:
                            rospy.sleep(0.5)
                        
                    if not self.task_active:
                        break
                        
                    # ������·����
                    rospy.loginfo("Navigating to waypoint %d/%d: (%.2f, %.2f)", 
                                 i, len(full_path), point['x'], point['y'])
                    success = self.navigate_to_point(point)
                    
                    # ����·����ʱ�ϱ�λ�ã�ȷ��׼ȷ��
                    self.report_position()
                    
                    if not success:
                        rospy.logwarn("Failed to reach waypoint %d", i)
                        
                        # �����ϰ�����ʽ����
                        if obstacle_method == "retry":
                            rospy.loginfo("Retrying navigation...")
                            success = self.navigate_to_point(point)
                        elif obstacle_method == "skip":
                            rospy.loginfo("Skipping waypoint")
                            continue
                        elif obstacle_method == "abort":
                            rospy.logwarn("Aborting task")
                            self.task_active = False
                            break
                
            # �������
            if self.task_active:
                rospy.loginfo("Task %s completed", task_id)
                self.publish_task_status("COMPLETED")
        except Exception as e:
            rospy.logerr("Task execution error: %s", str(e))
            self.publish_task_status("FAILED", str(e))
        finally:
            with self.task_lock:
                self.task_active = False
                self.current_task = None

    def get_current_position(self):
        """��ȡС����ǰλ�� - ֱ��ʹ�õ�ǰpose"""
        with self.pose_lock:
            if self.current_pose and self.current_pose.position:
                return self.current_pose.position
            
        # �����ǰpose�����ã����Եȴ�λ�ø���
        rospy.logwarn("Current pose not available, waiting for position update...")
        try:
            # �ȴ�λ�ø���
            msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=3.0)
            with self.pose_lock:
                self.current_pose = msg.pose.pose
                return self.current_pose.position
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for position update")
            return None

    def navigate_to_point(self, point):
        """Navigate to specified point"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = float(point['x'])
        goal.target_pose.pose.position.y = float(point['y'])
        goal.target_pose.pose.orientation.w = 1.0  # Ĭ�ϳ���
        
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        
        return self.move_base_client.get_result() is not None

    def publish_task_status(self, status, message=""):
        """Publish task status"""
        # ȷ��status_pub�ѳ�ʼ��
        if not hasattr(self, 'status_pub') or self.status_pub is None:
            rospy.logerr("status_pub not initialized!")
            return
            
        if not self.current_task:
            return
            
        status_data = {
            "robotId": self.current_task.get('robotId', ''),
            "taskId": self.current_task.get('taskId', ''),
            "status": status,
            "timestamp": rospy.get_time(),
            "message": message
        }
        self.status_pub.publish(json.dumps(status_data))

if __name__ == '__main__':
    try:
        executor = TaskExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task executor node shutdown")