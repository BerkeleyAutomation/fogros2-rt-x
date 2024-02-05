import socket

import rclpy
from rosbag2_py import Recorder, RecordOptions, StorageOptions
from rclpy.node import Node
import time
from std_srvs.srv import Empty
from threading import Thread


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

        self.new_episode_notification_service = self.create_service(Empty, 'new_episode_notification_service', self.new_episode_notification_service_callback)
        self.new_dataset_notification_service = self.create_service(Empty, 'new_dataset_notification_service', self.new_dataset_notification_service_callback)
        self.episode_recorder = Recorder()

        self.logger = self.get_logger()
        self.episode_counter = 1
        self.dataset_counter = 1
        self.init_recorder()
        self.logger.info("Recording started")
    
    def new_dataset_notification_service_callback(self, request, response):
        self.logger.info("Received request to start new dataset")
        self.stop_recorder()
        self.episode_counter = 1
        self.dataset_counter += 1
        self.start_new_recorder()
        return response

    def new_episode_notification_service_callback(self, request, response):
        self.logger.info("Received request to start new episode")
        self.stop_recorder()
        self.episode_counter += 1
        self.start_new_recorder()
        return response
    
    def init_recorder(self):
        self.start_new_recorder()

    def start_new_recorder(self):
        self.logger.info(f"starting episode #: {self.episode_counter}")
        storage_options = StorageOptions(
            uri=f"rosbags/dataset_{self.dataset_counter}/episode_{self.episode_counter}",
            storage_id="sqlite3"
        )
        record_options = RecordOptions()
        record_options.all = True
        self.thread = Thread(target=self.episode_recorder.record, args=(storage_options, record_options)).start()

    def stop_recorder(self):
        self.episode_recorder.cancel()
        
def main(args=None):

    rclpy.init(args=args)
    node = DatasetRecorder()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()