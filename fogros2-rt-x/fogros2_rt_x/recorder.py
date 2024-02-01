import socket

import rclpy
from rosbag2_py import Recorder, RecordOptions, StorageOptions
from rclpy.node import Node
import time
from .replayer_common import episode_recorder

def init_recorder():
    global episode_recorder
    storage_options = StorageOptions(
        uri=f"rosbags/episode_{1}",
        storage_id="sqlite3"
    )
    record_options = RecordOptions()
    record_options.all = True
    episode_recorder.record(storage_options, record_options)


def start_new_recorder(episode_counter):
    global episode_recorder
    print("CURRENT EPISODE IS:", episode_counter)
    storage_options = StorageOptions(
        uri=f"rosbags/episode_{episode_counter}",
        storage_id="sqlite3"
    )
    record_options = RecordOptions()
    record_options.all = True
    # episode_recorder = Recorder()
    episode_recorder.record(storage_options, record_options)

def restart_recorder():
    global episode_recorder
    print("ATTR:", dir(episode_recorder))
    episode_recorder.cancel()
    episode_recorder.stop()
    

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
        self.logger = self.get_logger()
        self.episode_counter = 1
        init_recorder()
        while True:
            self.episode_counter += 1
            start_new_recorder(self.episode_counter)

def main(args=None):

    rclpy.init(args=args)
    node = DatasetRecorder()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()