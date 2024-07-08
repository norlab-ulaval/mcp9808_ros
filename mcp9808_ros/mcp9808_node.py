import adafruit_mcp9808
import board
import busio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from rclpy.executors import SingleThreadedExecutor



class MCP9808Node(Node):
    def __init__(self):
        super().__init__("mcp9808_node")

        # Logger
        self.logger = self.get_logger()

        #I2C setup
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Parameters
        self.pub_rate = self.declare_parameter("pub_rate", 50).value
        self.frame_id = self.declare_parameter('frame_id', "mcp9808").value

        # mcp9808 instance
        self.mcp = adafruit_mcp9808.MCP9808(self.i2c)

        # Publisher
        self.temp_pub_ = self.create_publisher(
            Temperature,
            "temperature",
            10,
        )

        # timer for publisher
        self.pub_clk_ = self.create_timer(
            1 / self.pub_rate,
            self.publish_cback,
        )

        self.temperature_msg = Temperature()
        self.temperature_msg.header.frame_id = self.frame_id
        self.temperature_msg.variance = 0.01

    def publish_cback(self):
        stamp = self.get_clock().now().to_msg()
        temperature_value = self.mcp.temperature
        self.temperature_msg.header.stamp = stamp
        self.temperature_msg.temperature = temperature_value
        self.temp_pub_.publish(self.temperature_msg)


def main(args=None):
    rclpy.init(args=args)
    mcp9808_node = MCP9808Node()

    executor = SingleThreadedExecutor()
    executor.add_node(mcp9808_node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        print("\n>> Received ctrl-c ... bye\n")

    finally:
        executor.shutdown()
        mcp9808_node.destroy_node()


if __name__ == "__main__":
    main()
