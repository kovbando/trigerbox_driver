import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import socket
import threading


class TriggerIdPublisher(Node):
    def __init__(self):
        super().__init__('triggerid_publisher')

        # Declare parameters for port and topic
        self.declare_parameter('udp_port', 5555)
        self.declare_parameter('output_topic', '/triggerid')

        # Get parameter values
        udp_port = self.get_parameter('udp_port').value
        output_topic = self.get_parameter('output_topic').value

        # Create a publisher
        self.publisher_ = self.create_publisher(Int32, output_topic, 10)
        self.get_logger().info(f"Publishing to topic: {output_topic}")

        # Set up UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', udp_port))
        self.get_logger().info(f"Listening for UDP packets on port: {udp_port}")

        # Start a thread to handle incoming data
        self.running = True
        self.listener_thread = threading.Thread(target=self.listen_for_data)
        self.listener_thread.start()

    def listen_for_data(self):
        while self.running:
            try:
                # Blocking call to receive data
                data, _ = self.sock.recvfrom(1024)  # Buffer size of 1024 bytes
                message = data.decode('ascii').strip()
                self.get_logger().debug(f"Received message: {message}")

                # Try to convert the message to an integer
                try:
                    trigger_id = int(message)
                    msg = Int32()
                    msg.data = trigger_id
                    if self.running:  # Ensure we only publish if the node is still running
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published trigger ID: {trigger_id}")
                except ValueError:
                    self.get_logger().warn(f"Invalid data received: {message}")
            except socket.error as e:
                if self.running:  # Avoid logging errors during shutdown
                    self.get_logger().error(f"Socket error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error in listener thread: {e}")

    def destroy_node(self):
        # Stop the listener thread and close the socket
        self.get_logger().info("Stopping listener thread...")
        self.running = False
        try:
            self.listener_thread.join(timeout=5)  # Wait for the thread to finish
            if self.listener_thread.is_alive():
                self.get_logger().warn("Listener thread did not terminate in time.")
        except Exception as e:
            self.get_logger().error(f"Error while stopping listener thread: {e}")

        self.get_logger().info("Closing UDP socket...")
        try:
            self.sock.close()
        except Exception as e:
            self.get_logger().error(f"Error while closing socket: {e}")

        super().destroy_node()
        self.get_logger().info("Node destroyed successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = TriggerIdPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TriggerIdPublisher node.")
    finally:
        node.destroy_node()
        # Ensure rclpy.shutdown() is only called once
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()