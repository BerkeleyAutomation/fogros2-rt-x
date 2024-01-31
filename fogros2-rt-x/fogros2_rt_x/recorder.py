import socket

import rclpy
from rosbag2_py import Recorder, RecordOptions, StorageOptions
from rclpy.node import Node
import time

class DatasetRecorder(Node):
    """
    A class for replaying datasets in ROS2.

    Args:
        dataset_name (str): The name of the dataset to be replayed.

    Attributes:
        publisher (Publisher): The publisher for sending step information.
        dataset (Dataset): The loaded RLDS dataset.
        logger (Logger): The logger for logging information.
        feature_spec (DatasetFeatureSpec): The feature specification for the dataset.
        episode (Episode): The current episode being replayed.
    """

    def __init__(self):
        super().__init__("fogros2_rt_x_recorder")
        self.episode_counter = 1
        self.storage_options = StorageOptions(
            uri=f"rosbags/episode_{self.episode_counter}",
            storage_id="sqlite3"
        )

        self.record_options = RecordOptions()
        self.record_options.all = True
        self.recorder = Recorder()

        try:
            self.recorder.record(self.storage_options, self.record_options)
        except KeyboardInterrupt:
            pass


def main(args=None):

    rclpy.init(args=args)
    node = DatasetRecorder()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()