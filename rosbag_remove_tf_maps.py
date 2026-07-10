import sys
import rosbag
from tf2_msgs.msg import TFMessage

def remove_tf_map_lio(input_bag_path, output_bag_path):
    print("Opening the bagfile, please wait...")
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        total_msgs = inbag.get_message_count()
    processed_msgs = 0
    removed_transforms = 0

    with rosbag.Bag(output_bag_path, 'w') as outbag:
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                if topic in ('/tf', '/tf_static'):
                    filtered = [
                        transform for transform in msg.transforms
                        if transform.child_frame_id not in ('map_lio', 'map_imu', 'map_raw')
                    ]
                    removed_transforms += len(msg.transforms) - len(filtered)
                    if filtered:
                        new_msg = TFMessage()
                        new_msg.transforms = filtered
                        outbag.write(topic, new_msg, t)
                else:
                    outbag.write(topic, msg, t)
                processed_msgs += 1
                if processed_msgs % 1000 == 0 or processed_msgs == total_msgs:
                    percent = (processed_msgs / total_msgs) * 100
                    print(f"\rProgress: {percent:.2f}% ({processed_msgs}/{total_msgs})", end='', flush=True)
    print(f"\nDone. Removed {removed_transforms} transforms with child_frame_id in ('map_lio', 'map_imu', 'map_raw').")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python rosbag_remove_tf_maps.py <input_bag> <output_bag>")
        sys.exit(1)
    remove_tf_map_lio(sys.argv[1], sys.argv[2])
