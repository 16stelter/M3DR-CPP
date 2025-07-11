#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import rosbag2_py
import sys

class MarkerPublisher(Node):
    def __init__(self, marker_msg):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, '/coverage_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.marker_msg = marker_msg
        self.published = False

    def timer_callback(self):
        if not self.published:
            self.publisher.publish(self.marker_msg)
            self.get_logger().info('Published specified /coverage_marker message.')
            self.published = True
            self.timer.cancel()

def get_nth_marker_message(bag_path, index):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/coverage_marker':
            count += 1
            if count == index:
                from rclpy.serialization import deserialize_message
                from rosidl_runtime_py.utilities import get_message
                msg_type = get_message(type_map[topic])
                return deserialize_message(data, msg_type)
    return None

def main():
    if len(sys.argv) != 3:
        print("Usage: ros2 run <your_package> <your_script> <bag_path> <message_index>")
        return

    bag_path = sys.argv[1]
    try:
        index = int(sys.argv[2])
    except ValueError:
        print("Invalid message index. Must be an integer.")
        return

    rclpy.init()
    marker_msg = get_nth_marker_message(bag_path, index)
    if marker_msg is None:
        print(f"Message #{index} not found.")
        return

    node = MarkerPublisher(marker_msg)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

