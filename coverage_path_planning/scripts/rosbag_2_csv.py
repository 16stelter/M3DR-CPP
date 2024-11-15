import csv
import argparse
import rclpy
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict
from rclpy.serialization import deserialize_message

def parse_duration(duration):
    # Convert a dictionary {'sec': x, 'nanosec': y} to a float
    if isinstance(duration, dict):
        sec = duration.get('sec', 0)
        nanosec = duration.get('nanosec', 0)
        return sec + nanosec * 1e-9
    return duration  # If it's not a dictionary, return it as is

def bag_to_csv(bag_file, topic, csv_file):
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic information to retrieve message type
    topic_types = reader.get_all_topics_and_types()
    topic_type = None
    for t in topic_types:
        if t.name == topic:
            topic_type = t.type
            break

    if topic_type is None:
        raise ValueError(f"Topic '{topic}' not found in the bag file.")

    msg_class = get_message(topic_type)

    headers = None
    with open(csv_file, 'w', newline='') as file:
        writer = None

        while reader.has_next():
            (current_topic, serialized_msg, t) = reader.read_next()
            if current_topic != topic:
                continue  # Skip messages not in the specified topic

            # Deserialize the message
            msg = deserialize_message(serialized_msg, msg_class)
            msg_dict = message_to_ordereddict(msg)

            if headers is None:
                # Identify numeric list fields and add "sum" columns for each
                headers = []
                sum_fields = {}
                for key, value in msg_dict.items():
                    headers.append(key)  # Add the field as usual
                    if isinstance(value, list) and all(isinstance(x, (int, float)) for x in value):
                        sum_field_name = f"{key}_sum"
                        sum_fields[key] = sum_field_name
                        headers.append(sum_field_name)  # Add the sum field right after the list field
                
                writer = csv.DictWriter(file, fieldnames=headers)
                writer.writeheader()

            # Prepare the row, adding sum fields if applicable
            row = {}
            for key, value in msg_dict.items():
                # Check for duration fields and convert them:
                if isinstance(value, dict) and 'sec' in value and 'nanosec' in value:
                    row[key] = parse_duration(value)
                else:
                    row[key] = str(value) if isinstance(value, list) else value
                if key in sum_fields:
                    row[sum_fields[key]] = sum(value)  # Add the sum of the list to the new field

            writer.writerow(row)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS2 bag topic to CSV.")
    parser.add_argument("bag_file", type=str, help="Path to the ROS2 bag file (directory for MCAP storage).")
    parser.add_argument("topic", type=str, help="Topic name to extract.")
    parser.add_argument("csv_file", type=str, help="Output CSV file path.")
    args = parser.parse_args()

    bag_to_csv(args.bag_file, args.topic, args.csv_file)
