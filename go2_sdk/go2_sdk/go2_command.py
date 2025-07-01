import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request, RequestHeader, RequestIdentity
import json
class CmdVelToSportNode(Node):
    """
    A ROS2 node that converts cmd_vel (geometry_msgs/Twist) messages
    to Unitree Go2 sport commands for robot movement control.
    """
    def __init__(self):
        super().__init__("cmd_vel_to_sport_node")
        # Sport API constants (from ros2_sport_client.h)
        self.ROBOT_SPORT_API_ID_MOVE = 1008
        self.ROBOT_SPORT_API_ID_BALANCESTAND = 1002
        self.ROBOT_SPORT_API_ID_STOPMOVE = 1003
        # Create subscriber for cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10
        )
        # Create publisher for sport request
        self.sport_publisher = self.create_publisher(
            Request,
            "/api/sport/request",
            10
        )
        # Control parameters
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.movement_timeout = 1.0  # seconds
        # Timer to check for movement timeout
        self.last_cmd_time = self.get_clock().now()
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info("CmdVel to Sport Node initialized")
        self.get_logger().info("Subscribing to: cmd_vel")
        self.get_logger().info("Publishing to: /api/sport/request")
    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for cmd_vel messages.
        Converts Twist message to Unitree sport move command.
        """
        # Update last command time
        self.last_cmd_time = self.get_clock().now()
        # Extract velocities from Twist message
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        # Clamp velocities to maximum limits
        linear_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x))
        linear_y = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_y))
        angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z))
        # Create sport request message
        request_msg = Request()
        # Set header information
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.ROBOT_SPORT_API_ID_MOVE
        # Create parameter JSON (following SportClient::Move format)
        move_params = {
            "x": float(linear_x),
            "y": float(linear_y),
            "z": float(angular_z)
        }
        request_msg.parameter = json.dumps(move_params)
        # Publish the request
        self.sport_publisher.publish(request_msg)
        self.get_logger().debug(
            f"Published move command: vx={linear_x:.3f}, vy={linear_y:.3f}, vyaw={angular_z:.3f}"
        )
    def check_timeout(self):
        """
        Check if no cmd_vel message has been received for a while.
        If timeout occurs, send stop command to ensure robot safety.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if time_since_last_cmd > self.movement_timeout:
            # Send stop command
            self.send_stop_command()
    def send_stop_command(self):
        """
        Send a stop movement command to the robot.
        """
        request_msg = Request()
        # Set header information for stop command
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.ROBOT_SPORT_API_ID_STOPMOVE
        # Stop command doesn"t need parameters
        request_msg.parameter = ""
        # Publish the stop request
        self.sport_publisher.publish(request_msg)
        self.get_logger().debug("Published stop command due to timeout")
    def send_balance_stand_command(self):
        """
        Send a balance stand command to prepare the robot for movement.
        Call this before starting movement commands.
        """
        request_msg = Request()
        # Set header information for balance stand
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.ROBOT_SPORT_API_ID_BALANCESTAND
        # Balance stand doesn"t need parameters
        request_msg.parameter = ""
        # Publish the balance stand request
        self.sport_publisher.publish(request_msg)
        self.get_logger().info("Published balance stand command")
def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    try:
        node = CmdVelToSportNode()
        # Send balance stand command at startup
        node.send_balance_stand_command()
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Send stop command before shutdown
        if "node" in locals():
            node.send_stop_command()
        # Cleanup
        rclpy.shutdown()
if __name__ == "__main__":
    main()
