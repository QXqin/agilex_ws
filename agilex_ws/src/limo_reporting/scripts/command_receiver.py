#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import threading
import json
import time
from flask import Flask, request, jsonify
from std_msgs.msg import String

class CommandReceiver:
    def __init__(self):
        rospy.init_node('command_receiver')
        
        # Configuration parameters
        self.host = rospy.get_param('~host', '0.0.0.0')
        self.port = rospy.get_param('~port', 5000)
        
        # Create command publisher
        self.command_pub = rospy.Publisher('/task_commands', String, queue_size=10)
        
        # Create Flask application
        self.app = Flask(__name__)
        
        # Add routes
        self.app.add_url_rule('/api/task/start', 'handle_start', self.handle_start, methods=['POST'])
        
        # Start Flask server thread
        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()
        
        rospy.loginfo("Command receiver node started on %s:%d", self.host, self.port)

    def run_flask(self):
        """Run Flask server"""
        self.app.run(host=self.host, port=self.port, threaded=True)

    def validate_start_request(self, data):
        """Validate task start request data"""
        # 
        required_fields = ['robotId', 'taskId', 'taskCycle', 'taskPoints']
        
        # Check required fields
        missing = [field for field in required_fields if field not in data]
        if missing:
            return False, "Missing required fields: {}".format(', '.join(missing))
        
        # Validate field types
        if not isinstance(data['taskCycle'], int) or data['taskCycle'] <= 0:
            return False, "Task cycle must be a positive integer"
            
        # 
        if 'taskLaps' not in data:
            data['taskLaps'] = 1  # 
        elif not isinstance(data['taskLaps'], int) or data['taskLaps'] <= 0:
            return False, "Task laps must be a positive integer"
            
        # 
        if 'obstacleHandlingMethod' not in data:
            data['obstacleHandlingMethod'] = "skip"  
            
        # Validate waypoints
        if not isinstance(data['taskPoints'], list) or len(data['taskPoints']) < 2:
            return False, "Waypoints must be a list with at least 2 points"
            
        # Validate each waypoint
        for i, point in enumerate(data['taskPoints']):
            if 'x' not in point or 'y' not in point:
                return False, "Waypoint {} is missing x or y coordinate".format(i+1)
                
            try:
                float(point['x'])
                float(point['y'])
            except ValueError:
                return False, "Waypoint {} coordinates must be numbers".format(i+1)
        
        return True, "Validation passed"

    def handle_start(self):
        """Handle task start command"""
        # Get JSON data
        try:
            data = request.get_json()
            if not data:
                return jsonify({"msg": "No data provided", "code": 400}), 400
        except Exception as e:
            return jsonify({"msg": "JSON parsing error: {}".format(str(e)), "code": 400}), 400
        
        # Validate request data
        is_valid, error_msg = self.validate_start_request(data)
        if not is_valid:
            rospy.logwarn("Invalid task start request: %s", error_msg)
            return jsonify({"msg": error_msg, "code": 400}), 400
        
        rospy.loginfo("Received task start command: TaskID=%s, Cycles=%d", 
                      data['taskId'], data['taskCycle'])
        
        try:
            # Add timestamps and status information
            command = {
                "command": "start",
                "timestamp": time.time(),
                "data": data
            }
            
            # Publish to ROS topic
            self.command_pub.publish(json.dumps(command))
            
            # Return success response
            return jsonify({
                "msg": "Task start command received",
                "code": 200,
                "taskId": data['taskId'],
                "startTime": time.strftime("%Y-%m-%d %H:%M:%S"),
                "waypointCount": len(data['taskPoints'])
            })
        except Exception as e:
            rospy.logerr("Error processing task start command: %s", str(e))
            return jsonify({
                "msg": "Internal server error: {}".format(str(e)),
                "code": 500
            }), 500

if __name__ == '__main__':
    try:
        receiver = CommandReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Command receiver node shutdown")