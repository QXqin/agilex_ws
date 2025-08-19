#!/usr/bin/env python2
import rospy
import threading
import requests
import json
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty, EmptyResponse


class PoseReporter:
    def __init__(self):
        rospy.init_node('pose_reporter')

        # Initialize parameters
        self.load_params()

        # Current pose
        self.current_pose = None
        self.pose_lock = threading.Lock()

        # Subscribe to pose updates
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Create service
        self.reload_service = rospy.Service('~reload_params', Empty, self.reload_callback)

        # Start reporting thread
        self.report_thread = threading.Thread(target=self.report_loop)
        self.report_thread.daemon = True
        self.report_thread.start()

        rospy.loginfo("Pose reporter node started | API: %s" % self.api_url)

    # Parameter loading method
    def load_params(self):
        """Load or reload parameters"""
        self.robot_id = rospy.get_param('~robot_id', 'limo-4831')
        self.task_id = rospy.get_param('~task_id', 'default-task')
        self.api_url = rospy.get_param('~api_url', 'http://10.110.143.157:8812/xunjian/api/report/coordinates')
        self.report_interval = rospy.get_param('~report_interval', 1.0)

        # Print parameters for debugging
        rospy.loginfo("===== Current Parameters =====")
        rospy.loginfo("robot_id: %s", self.robot_id)
        rospy.loginfo("task_id: %s", self.task_id)
        rospy.loginfo("api_url: %s", self.api_url)
        rospy.loginfo("report_interval: %.1fs", self.report_interval)
        rospy.loginfo("=============================")

    # Fixed callback signature and logic
    def reload_callback(self, req):
        """Parameter reload service callback"""
        rospy.logwarn("Received parameter reload request!")
        try:
            # Delete all related parameters
            for param_name in ['robot_id', 'task_id', 'api_url', 'report_interval']:
                try:
                    rospy.delete_param('~' + param_name)
                    rospy.loginfo("Deleted parameter: ~%s", param_name)
                except Exception as e:
                    rospy.logwarn("Parameter deletion failed: %s", str(e))

            # Reload parameters
            self.load_params()
            return EmptyResponse()
        except Exception as e:
            rospy.logerr("Parameter reload failed: %s", str(e))
            return EmptyResponse()

    def pose_callback(self, msg):
        """Handle pose updates"""
        with self.pose_lock:
            self.current_pose = msg.pose.pose
            rospy.logdebug("Received new pose: (%.3f, %.3f)",
                           self.current_pose.position.x,
                           self.current_pose.position.y)

    def report_loop(self):
        """Periodically report pose"""
        rate = rospy.Rate(1.0 / self.report_interval)

        while not rospy.is_shutdown():
            if self.current_pose:
                try:
                    self.report_position()
                except Exception as e:
                    rospy.logerr("Reporting failed: %s" % str(e))
            else:
                rospy.logwarn("No valid pose data, skipping report")

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
                "taskId": self.task_id,
                "positionX": position.x,
                "positionY": position.y,
                "positionZ": position.z,
                "orientationX": orientation.x,
                "orientationY": orientation.y,
                "orientationZ": orientation.z,
                "orientationW": orientation.w
            }

            # API validation
            if not self.api_url.startswith("http"):
                rospy.logerr("Invalid API address: %s", self.api_url)
                return

            # Send request
            headers = {"Content-Type": "application/json"}
            try:
                response = requests.post(self.api_url, json=data, headers=headers, timeout=3.0)

                if response.status_code == 200:
                    rospy.loginfo("[SUCCESS] Reported to API: %s \nData: %s",
                                  self.api_url, json.dumps(data, indent=2))
                else:
                    rospy.logwarn("Reporting failed (%d): %s",
                                  response.status_code, response.text)
            except Exception as e:
                rospy.logerr("HTTP request error: %s", str(e))


if __name__ == '__main__':
    try:
        rospy.loginfo("==== Pose reporter node starting ====")
        reporter = PoseReporter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose reporter node shutdown")