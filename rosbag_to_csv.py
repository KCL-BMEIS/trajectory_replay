import sqlite3
import csv
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


if __name__ == "__main__":
    rosbag_reader = RosbagReader(
        "rosbag2_2023_02_01-11_21_54/rosbag2_2023_02_01-11_21_54_0.db3"
    )
    data = rosbag_reader.get_data("/link_transform_publisher_node/link_transform")

    with open("data.csv", "w") as f:
        writer = csv.writer(f)
        for timestamp, transform_stamped in data[::100]:
            # to csv
            writer.writerow(
                [
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z,
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w,
                ]
            )