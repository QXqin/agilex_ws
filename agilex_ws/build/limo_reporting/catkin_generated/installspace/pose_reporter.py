#!/usr/bin/env python2
import rospy
import threading
import requests
import json
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseReporter:
    def __init__(self):
        rospy.init_node('pose_reporter')
        
        # Configuration parameters
        self.robot_id = rospy.get_param('~robot_id', 'limo-001')
        self.task_id = rospy.get_param('~task_id', 'default-task')
        self.api_url = rospy.get_param('~api_url', 'http://39.107.253.156/xunjian/api/report/coordinates')
        self.report_interval = rospy.get_param('~report_interval', 1.0)  # Reporting interval in seconds
        
        # Current pose
        self.current_pose = None
        self.pose_lock = threading.Lock()
        
        # Subscribe to pose updates
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Start reporting thread
        self.report_thread = threading.Thread(target=self.report_loop)
        self.report_thread.daemon = True
        self.report_thread.start()
        
        rospy.loginfo("Pose reporter node started")

    def pose_callback(self, msg):
        """Handle pose updates"""
        with self.pose_lock:
            self.current_pose = msg.pose.pose

    def report_loop(self):
        """Periodically report pose"""
        rate = rospy.Rate(1.0 / self.report_interval)
        
        while not rospy.is_shutdown():
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
                #"taskId": self.task_id,
                "taskId": 1,
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
                    rospy.loginfo("Pose reported successfully: %s" % json.dumps(data))
                else:
                    rospy.logwarn("Pose report failed: status code %d, response: %s" % 
                                  (response.status_code, response.text))
            except Exception as e:
                rospy.logerr("HTTP request failed: %s" % str(e))

if __name__ == '__main__':
    try:
        reporter = PoseReporter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose reporter node shutdown")