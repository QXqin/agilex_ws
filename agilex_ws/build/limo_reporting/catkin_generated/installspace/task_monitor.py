#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import json
from std_msgs.msg import String

class TaskMonitor:
    def __init__(self):
        rospy.init_node('task_monitor')
        
        # Subscribe to task status
        rospy.Subscriber('/task_status', String, self.status_callback)
        
        rospy.loginfo("Task monitor node started")

    def status_callback(self, msg):
        """Handle task status updates"""
        try:
            status_data = json.loads(msg.data)
            task_id = status_data.get('taskId', 'Unknown task')
            status = status_data.get('status', 'Unknown status')
            message = status_data.get('message', '')
            
            if status == "RUNNING":
                rospy.loginfo("Task %s is running", task_id)
            elif status == "PAUSED":
                rospy.logwarn("Task %s paused: %s", task_id, message)
            elif status == "COMPLETED":
                rospy.loginfo("Task %s completed successfully", task_id)
            elif status == "FAILED":
                rospy.logerr("Task %s failed: %s", task_id, message)
            else:
                rospy.logwarn("Unknown task status: %s", status)
                
        except Exception as e:
            rospy.logerr("Error processing task status: %s", str(e))

if __name__ == '__main__':
    try:
        monitor = TaskMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task monitor node shutdown")