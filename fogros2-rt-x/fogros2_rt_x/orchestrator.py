

import rclpy
from rclpy.node import Node
from .dataset_utils import *
from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger
from envlogger import step_data
import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer
from .dataset_spec import DatasetFeatureSpec




def main(args=None):
    rclpy.init(args=args)
    from .plugins.berkeley_fanuc_manipulation import ORCHESTRATOR
    stream_orchestrator = ORCHESTRATOR

    rclpy.spin(stream_orchestrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stream_orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
