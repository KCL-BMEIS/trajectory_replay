import argparse
import csv
import glob
import os
import sqlite3
from typing import Any

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class RosbagReader:
    def __init__(self, rosbag: str) -> None:
        self.database = sqlite3.connect(rosbag)
        self.cursor = self.database.cursor()

        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

    def __del__(self):
        self.database.close()

    def get_data(self, topic_name: str) -> Any:
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]


def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rosbag_path",
        type=str,
        required=True,
        help="Script searches for .db3 file in <rosbag_path>.",
    )
    parser.add_argument(
        "--lbr_state_topic",
        type=str,
        default="/lbr/state",
        help="The topic name. Prefixed with robot name.",
    )

    return parser.parse_args()


def main() -> None:
    args = args_factory()
    rosbag = glob.glob(os.path.join(args.rosbag_path, "*.db3"))
    if len(rosbag) != 1:
        raise RuntimeError(f"Expected only 1 rosbag, got {len(rosbag)}.")
    rosbag = rosbag[0]
    print(f"Opening rosbag from {rosbag}.")
    rosbag_reader = RosbagReader(rosbag)

    print(f"Reading data from topic: {args.lbr_state_topic}.")
    lbr_states = rosbag_reader.get_data(args.lbr_state_topic)
    joint_names = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
    with open("lbr_states.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerow(joint_names)
        for timestamp, lbr_state in lbr_states:
            writer.writerow(lbr_state.measured_joint_position)


if __name__ == "__main__":
    main()
