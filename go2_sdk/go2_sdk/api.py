import rclpy
from rclpy.node import Node
from flask import Flask, jsonify, request
from flask_cors import CORS
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import threading
from typing import Optional

class Go2APINode(Node):
    """
    A ROS2 node that provides a REST API for interacting with the Unitree Go2 robot.
    """
    def __init__(self):
        super().__init__("go2_api_node")

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/om/pose',
            self.pose_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.pose_data: Optional[PoseWithCovarianceStamped] = None

        self.app = Flask(__name__)
        CORS(self.app)
        self.register_routes()

        self.api_thread = threading.Thread(target=self.run_flask_app)
        self.api_thread.start()

        self.get_logger().info("Go2 API Node initialized")

    def run_flask_app(self):
        """
        Run the Flask application in a separate thread.
        """
        self.app.run(host='0.0.0.0', port=5000, use_reloader=False)

    def register_routes(self):
        """
        Register the API routes.
        """
        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            return jsonify({"status": "OK", "message": "Go2 API is running"}), 200

        @self.app.route('/api/pose', methods=['GET'])
        def get_pose():
            """
            Get the current pose of the robot.
            Returns a JSON object with the robot's position and orientation.
            """
            if self.pose_data is None:
                return jsonify({"error": "Pose data not available"}), 404

            pose = {
                "position": {
                    "x": self.pose_data.pose.position.x,
                    "y": self.pose_data.pose.position.y,
                    "z": self.pose_data.pose.position.z
                },
                "orientation": {
                    "x": self.pose_data.pose.orientation.x,
                    "y": self.pose_data.pose.orientation.y,
                    "z": self.pose_data.pose.orientation.z,
                    "w": self.pose_data.pose.orientation.w
                },
                "covariance": self.pose_data.pose.covariance
            }

            return jsonify(pose), 200

        @self.app.route('/api/move_to_pose', methods=['POST'])
        def move_to_pose():
            """
            Move the robot to a specified pose.
            Expects a JSON object with 'position' and 'orientation'.
            """
            data = request.json
            if not data or 'position' not in data or 'orientation' not in data:
                return jsonify({"error": "Invalid input"}), 400

            position = data['position']
            orientation = data['orientation']

            if not all(k in position for k in ('x', 'y', 'z')) or not all(k in orientation for k in ('x', 'y', 'z', 'w')):
                return jsonify({"error": "Position and orientation must contain x, y, z, and w"}), 400

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = position['x']
            pose_msg.pose.position.y = position['y']
            pose_msg.pose.position.z = position['z']
            pose_msg.pose.orientation.x = orientation['x']
            pose_msg.pose.orientation.y = orientation['y']
            pose_msg.pose.orientation.z = orientation['z']
            pose_msg.pose.orientation.w = orientation['w']

            self.pose_publisher.publish(pose_msg)

            self.get_logger().info(f"Moving to pose: {pose_msg.pose}")

            return jsonify({"status": "success", "message": "Moving to specified pose"}), 200

    def pose_callback(self, msg: PoseStamped):
        """
        Callback function for PoseStamped messages.
        Updates the internal pose data and logs the received pose.

        Parameters:
        -----------
        msg : geometry_msgs.msg.PoseStamped
            The incoming PoseStamped message containing the robot's pose.
        """
        self.pose_data = PoseWithCovarianceStamped()
        self.pose_data.header = msg.header
        self.pose_data.pose.pose = msg.pose
        self.pose_data.pose.covariance = [0.0] * 36

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback function for AMCL pose updates.
        Updates the internal pose data and logs the received AMCL pose.

        Parameters:
        -----------
        msg : geometry_msgs.msg.PoseWithCovarianceStamped
            The incoming AMCL pose message containing the robot's pose with covariance.
        """
        self.pose_data.pose.covariance = msg.pose.covariance

def main(args=None):
    """
    Main function to initialize the ROS2 node and start the API.
    """
    rclpy.init(args=args)

    go2_api_node = Go2APINode()

    try:
        rclpy.spin(go2_api_node)
    except KeyboardInterrupt:
        pass
    finally:
        go2_api_node.destroy_node()
        rclpy.shutdown()
