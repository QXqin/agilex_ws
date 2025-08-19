#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String

# Nodes to check
NODES_TO_CHECK = {
    "limo_base_node": "limo_base",
    "rosbridge_websocket": "rosbridge",
    "usb_cam": "usb_camera",
    "web_video_server": "web_video_server",
    "move_base": "navigation",
    "single_point_nav": "single_point_nav",
    "pose_reporter": "pose_reporter"
}

def get_running_nodes():
    try:
        result = subprocess.check_output(["rosnode", "list"]).decode("utf-8")
        return result.splitlines()
    except Exception as e:
        rospy.logerr("Failed to get node list: %s", e)
        return []

def monitor():
    pub = rospy.Publisher("/limo_status", String, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        running_nodes = get_running_nodes()
        status_report = []
        all_ok = True

        for node, desc in NODES_TO_CHECK.items():
            if any(node in n for n in running_nodes):
                msg = "[OK] {} ({})".format(desc, node)
            else:
                msg = "[FAIL] {} ({})".format(desc, node)
                all_ok = False
            rospy.loginfo(msg)
            status_report.append(msg)

        # Publish status report to /limo_status
        report = "\n".join(status_report)
        pub.publish(report)

        if all_ok:
            rospy.loginfo("All key nodes are running normally.")
        else:
            rospy.logwarn("Some nodes are not running properly.")

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("limo_status_monitor")
    rospy.loginfo("Starting LIMO status monitor...")
    monitor()