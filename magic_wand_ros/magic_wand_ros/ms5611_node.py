from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Header
from magic_wand_ros.barometer import MS5611


class MS5611Node(Node):
    """A ROS2 node for the MS5611 barometer sensor"""

    def __init__(self):
        """Constructor for the MS5611Node class"""
        super().__init__("ms5611_node")
        self.bus = SMBus(88)  # MINT Lab udev rule
        self.i2c_address = 0x77
        self.ms5611 = MS5611(bus=self.bus, i2c_address=self.i2c_address)
        self.get_logger().info("MS5611Node initialized")

        self.declare_parameter("enable_logging", False)
        self.enable_logging = (
            self.get_parameter("enable_logging").get_parameter_value().bool_value
        )

        self.publisher_pressure = self.create_publisher(
            FluidPressure, "/magic_wand/pressure", 10
        )
        self.publisher_temperature = self.create_publisher(
            Temperature, "/magic_wand/temperature", 10
        )
        self.timer = self.create_timer(
            (timer_period := 0.033), self.publish_barometric_data
        )

        # Resolution RMS
        self._pressure_rms = None
        self._temperature_rms = None

        # Resolution RMS according to the OSR
        if self.ms5611._CONVERT_D1_COMMAND == 0x40:
            self._pressure_rms = 0.065
        elif self.ms5611._CONVERT_D1_COMMAND == 0x42:
            self._pressure_rms = 0.042
        elif self.ms5611._CONVERT_D1_COMMAND == 0x44:
            self._pressure_rms = 0.027
        elif self.ms5611._CONVERT_D1_COMMAND == 0x46:
            self._pressure_rms = 0.018
        elif self.ms5611._CONVERT_D1_COMMAND == 0x48:
            self._pressure_rms = 0.012

        if self.ms5611._CONVERT_D2_COMMAND == 0x50:
            self._temperature_rms = 0.012
        elif self.ms5611._CONVERT_D2_COMMAND == 0x52:
            self._temperature_rms = 0.008
        elif self.ms5611._CONVERT_D2_COMMAND == 0x54:
            self._temperature_rms = 0.005
        elif self.ms5611._CONVERT_D2_COMMAND == 0x56:
            self._temperature_rms = 0.003
        elif self.ms5611._CONVERT_D2_COMMAND == 0x58:
            self._temperature_rms = 0.002

        # Degree sign for temperature
        self._degree_sign = "\N{DEGREE SIGN}"

    def publish_barometric_data(self):
        """Publishes barometric data to the ROS2 topic"""
        pressure, temperature = self.ms5611.read_sensor()
        pressure_msg = FluidPressure()
        temperature_msg = Temperature()

        timestamp = self.get_clock().now().to_msg()
        frame_id = "barometer_link"

        pressure_msg.header = Header(stamp=timestamp, frame_id=frame_id)
        pressure_msg.fluid_pressure = pressure
        pressure_msg.variance = self._pressure_rms

        temperature_msg.header = Header(stamp=timestamp, frame_id=frame_id)
        temperature_msg.temperature = temperature
        temperature_msg.variance = self._temperature_rms

        self.publisher_pressure.publish(pressure_msg)
        self.publisher_temperature.publish(temperature_msg)

        if self.enable_logging:
            self.get_logger().info(
                f"Pressure: {pressure} hPa, Temperature: {temperature} {self._degree_sign}C"
            )


def main(args=None):
    rclpy.init(args=args)
    ms5611_node = MS5611Node()
    rclpy.spin(ms5611_node)
    ms5611_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
