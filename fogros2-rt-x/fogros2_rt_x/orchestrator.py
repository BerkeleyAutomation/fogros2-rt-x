

import rclpy
from rclpy.node import Node
from .dataset_utils import *
from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger
from envlogger import step_data
import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer
from .dataset_spec import DatasetFeatureSpec
from .plugins.orchestrator_base import PerPeriodTopicOrchestrator


# this node is only used for getting the dataset config
class PlaceholderNode(Node):
    def __init__(self):
        super().__init__("fogros2_rt_x_placeholder")
        self.declare_parameter("dataset_name", "berkeley_fanuc_manipulation")
        self.dataset_name = self.get_parameter("dataset_name").value
        self.config = get_dataset_plugin_config_from_str(self.dataset_name)

def main(args=None):
    rclpy.init(args=args)

    placeholder_node = PlaceholderNode()
    stream_orchestrator = get_orchestrator_from_str(placeholder_node.dataset_name)

    rclpy.spin(stream_orchestrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stream_orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
