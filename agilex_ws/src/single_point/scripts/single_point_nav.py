#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
multi_point_nav_singlefile.py
Single-file Flask + ROS (Melodic/Python2) server to accept multi-point navigation tasks,
drive move_base sequentially with retries/timeouts, and return results.

- Serve a simple web UI at /
- POST JSON array to /api/task/cruise: [{"x": 1.0, "y": 2.0}, {"x":3.0,"y":4.0}]
- Publishes status JSON strings to /navigation_status
"""
from __future__ import print_function
import rospy
import actionlib
import threading
import socket
import json
import os
import time
import requests
from flask import Flask, request, jsonify, Response
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
import json as pyjson
from geometry_msgs.msg import Twist

# Flask app (will run in background thread)
app = Flask(__name__)


# Navigator class encapsulates action client logic
class MultiPointNavigator(object):
    def __init__(self):
        # initialize parameters (get from param server if present)
        self.retry_limit = rospy.get_param('~retry_limit', 3)
        self.nav_timeout = rospy.get_param('~nav_timeout', 10.0)  # seconds
        self.client_name = rospy.get_param('~move_base_action', 'move_base')
        rospy.loginfo("MultiPointNavigator: waiting for action server '%s' ..." % self.client_name)
        self.client = actionlib.SimpleActionClient(self.client_name, MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Action server '%s' not available after 10s" % self.client_name)
            raise rospy.ROSException("move_base action server not available")
        rospy.loginfo("Connected to move_base action server")
        # Publisher to broadcast navigation status for frontend/other nodes
        self.status_pub = rospy.Publisher('/navigation_status', String, queue_size=10)

    def navigate_to(self, x, y):
        """
        Send a MoveBaseGoal to move_base and wait for result with retry.
        Returns dict of result: {"status":"arrived"/"failed"/"timeout", "x":x, "y":y}
        """
        rospy.loginfo("navigate_to: target=(%.3f, %.3f), retries=%d, timeout=%.1fs" %
                      (x, y, self.retry_limit, self.nav_timeout))
        for attempt in range(1, self.retry_limit + 1):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = float(x)
            goal.target_pose.pose.position.y = float(y)
            goal.target_pose.pose.orientation.w = 1.0

            rospy.loginfo("Sending goal attempt %d/%d to (%.3f, %.3f)" % (attempt, self.retry_limit, x, y))
            self.client.send_goal(goal)

            finished = self.client.wait_for_result(rospy.Duration(self.nav_timeout))
            if not finished:
                rospy.logwarn(
                    "Attempt %d: navigation timeout after %.1fs, canceling goal" % (attempt, self.nav_timeout))
                self.client.cancel_goal()
                # small backoff
                time.sleep(0.5)
                continue

            state = self.client.get_state()
            rospy.loginfo("move_base returned state=%d" % state)
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrived at (%.3f, %.3f)" % (x, y))
                res = {"status": "arrived", "x": x, "y": y}
                self.status_pub.publish(json.dumps(res))
                return res
            else:
                rospy.logwarn("Attempt %d: move_base did not succeed (state=%d)" % (attempt, state))
                time.sleep(0.5)
                continue

        # all retries exhausted
        rospy.logerr("Failed to reach (%.3f, %.3f) after %d attempts" % (x, y, self.retry_limit))
        res = {"status": "failed", "x": x, "y": y}
        self.status_pub.publish(json.dumps(res))
        return res


# global navigator instance (initialized in main)
navigator = None
cmd_pub = None
paused = False
pause_condition = threading.Condition()
# ----------------------------
# Flask routes
# ----------------------------

# Simple UI page embedded as string (keeps single-file)
INDEX_HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Multi-point Nav - Simple UI</title>
<style>
  body { font-family: Arial, sans-serif; margin: 20px; }
  textarea { width: 100%; height: 120px; font-family: monospace; }
  input, button { padding: 8px; font-size: 14px; }
  .row { margin-bottom: 10px; }
  .status { white-space: pre-wrap; background: #f8f8f8; padding: 8px; border: 1px solid #ddd; }
</style>
</head>
<body>
  <h2>Multi-point Navigation Test UI</h2>
  <p>Enter an array of points (JSON). Example format:</p>
  <pre>[{"x": 1.0, "y": 2.0}, {"x": 2.5, "y": 1.2}]</pre>
  <div class="row">
    <textarea id="points" placeholder='[{"x":1.0,"y":2.0}]'>[{"x":1.0,"y":0.0}]</textarea>
  </div>
  <div class="row">
    <button onclick="submit()">Submit Task</button>
    <button onclick="example()">Load Example</button>
  </div>
  <div class="row">
    <div>Server URL: <code id="url"></code></div>
  </div>
  <div class="row">
    <div class="status" id="result">Status / results will appear here</div>
  </div>
<script>
function getUrl() {
  return window.location.origin + '/api/task/cruise';
}
document.getElementById('url').innerText = getUrl();

function example() {
  document.getElementById('points').value = '[{"x": 1.0, "y": 0.0}, {"x": 0.5, "y": -1.0}]';
}

function submit() {
  var t = document.getElementById('points').value;
  var parsed;
  try {
    parsed = JSON.parse(t);
  } catch (e) {
    document.getElementById('result').innerText = 'JSON parse error: ' + e;
    return;
  }
  fetch('/api/task/cruise', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(parsed)
  }).then(function(resp) {
    if (!resp.ok) throw new Error('HTTP ' + resp.status + ' ' + resp.statusText);
    return resp.json();
  }).then(function(j) {
    document.getElementById('result').innerText = JSON.stringify(j, null, 2);
  }).catch(function(err) {
    document.getElementById('result').innerText = 'Request failed: ' + err;
  });
}
</script>
</body>
</html>
"""


@app.route('/', methods=['GET'])
def index():
    return Response(INDEX_HTML, mimetype='text/html')


@app.route('/api/task/cruise', methods=['GET', 'POST'])
    def api_cruise():
        global navigator,paused
        if navigator is None:
            return jsonify({"error": "Navigator not ready"}), 500

        try:
            data = request.get_json(force=True)
        except Exception as e:
            return jsonify({"error": "Invalid JSON", "detail": str(e)}), 400

        # allow {"taskId": "123", "points": [...] } OR plain [ {...}, {...} ]
        # if paused=True, is resume
        if isinstance(data, dict) and data.get("resume", False):
            with pause_condition:
                paused = False
                pause_condition.notify_all()
            rospy.loginfo("Resumed paused task")
            return jsonify({"status": "resumed"})

        # or is new task
        if isinstance(data, dict):
            points = data.get("points", [])
            task_id = data.get("taskId", "default_task")
        elif isinstance(data, list):
            points = data
            task_id = "default_task"
        else:
            return jsonify({"error": "Expected JSON array or object with taskId/points"}), 400

        if not isinstance(points, list):
            return jsonify({"error": "Points must be a list"}), 400

        t = threading.Thread(target=worker_task, args=(points, task_id))
        t.setDaemon(True)
        t.start()

        return jsonify({"status": "accepted", "points_count": len(points), "taskId": task_id})

    def worker_task(points_list, task_id):
        global paused, pause_condition
        results = []
        for i, pt in enumerate(points_list):
            # --- pause check ---
            with pause_condition:
                while paused:
                    rospy.loginfo("Task paused, waiting...")
                    pause_condition.wait()
            # --- end pause check ---

            try:
                x = float(pt.get('x') or pt.get('positionX') or pt.get('positonX'))
                y = float(pt.get('y') or pt.get('positionY') or pt.get('positonY'))
            except:
                results.append({"point": i + 1, "status": "invalid_data"})
                continue
            res = navigator.navigate_to(x, y)
            results.append({
                "point": i + 1,
                "status": res.get("status"),
                "x": res.get("x"),
                "y": res.get("y")
            })
        rospy.loginfo("Mission complete: %s" % results)

        try:
            # hard-coded carId
            car_id = "LIMO4831"

            callback_url = "http://10.110.162.3:5000//xunjian/api/report/taskCompleted"

            payload = {
                "mission_result": results,
                "carId": car_id,
                "taskId": task_id
            }

            payload_utf8 = pyjson.dumps(payload, ensure_ascii=False).encode('utf-8')
            headers = {'Content-Type': 'application/json; charset=utf-8'}

            r = requests.post(callback_url, data=payload_utf8, headers=headers, timeout=5)
            rospy.loginfo(
                "Reported task completion to backend: %d %s" % (r.status_code, r.text.encode('utf-8', 'replace')))
        except Exception as e:
            rospy.logerr("Failed to callback backend: %s", e)

# global navigator instance (initialized in main)
navigator = None
cmd_pub = None   # publisher for /cmd_vel

# ----------------------------
# Flask routes
# ----------------------------

@app.route('/api/task/pause', methods=['POST'])
def api_pause():
    global navigator, cmd_pub
    if navigator is None:
        return jsonify({"error": "Navigator not ready"}), 500

    try:
        data = request.get_json(force=True)
    except Exception as e:
        return jsonify({"error": "Invalid JSON", "detail": str(e)}), 400

    task_id = data.get("taskId", "unknown")

    # set paused state
    with pause_condition:
        paused = True

    # Cancel current navigation
    try:
        navigator.client.cancel_all_goals()
        rospy.loginfo("All goals canceled for task %s" % task_id)
    except Exception as e:
        rospy.logwarn("Failed to cancel goals: %s" % str(e))

    # Publish zero velocity to stop motion
    if cmd_pub is not None:
        stop_msg = Twist()
        cmd_pub.publish(stop_msg)
        rospy.loginfo("Published zero velocity to /cmd_vel")

    return jsonify({"status": "paused", "taskId": task_id})

# ----------------------------
# Helpers: find free port and run Flask in thread
# ----------------------------
def find_free_port(start=5000, end=5050):
    for p in range(start, end + 1):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(('0.0.0.0', p))
            s.close()
            return p
        except:
            s.close()
            continue
    return None


def run_flask(port):
    # use_reloader=False is important to avoid forking which would re-init rospy
    app.run(host='0.0.0.0', port=port, threaded=True, use_reloader=False)


# ----------------------------
# Main
# ----------------------------
if __name__ == '__main__':
    try:
        rospy.init_node('multi_point_nav_http_node', anonymous=True, disable_signals=True)
        # create navigator AFTER init_node
        navigator = MultiPointNavigator()
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # start flask thread
        port = rospy.get_param('~http_port', None)
        if port is None:
            port = find_free_port(5000, 5050)
            if port is None:
                port = 5000
        flask_thread = threading.Thread(target=run_flask, args=(port,))
        flask_thread.daemon = True
        flask_thread.start()
        rospy.loginfo("Flask UI available at http://%s:%d", socket.gethostname(), port)

        # Keep ROS alive; signals disabled above so use spin
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("Fatal error:", e)
        raise