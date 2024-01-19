# Copyright 2022 The Regents of the University of California (Regents)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import socket
from .exporter import DatasetExporter

import rclpy
from rclpy.node import Node
from .dataset_utils import *
from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger

import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer
from .dataset_spec import DatasetFeatureSpec
from .plugins.conf_base import *


from .database_connector import SqliteConnector
from std_srvs.srv import Empty
import random
import pickle 

class DatasetRecorder(Node):
    """
    A class for recording datasets in the fogros2-rt-x package.

    This class is responsible for recording datasets based on the provided observation and action specifications.

    Args:
        None

    Attributes:
        observation_spec (ObservationSpec): The specification for the observation.
        action_spec (ActionSpec): The specification for the action.
        feature_spec (DatasetFeatureSpec): The specification for the dataset features.
        dataset_config (DatasetConfig): The configuration for the dataset.
        last_action (Any): The last recorded action.
        last_observation (Any): The last recorded observation.
        last_step (Any): The last recorded step.
        writer (TFDSBackendWriter): The writer for the dataset.
        subscription (Subscription): The subscription for receiving step messages.

    Methods:
        __init__(): Initializes the DatasetRecorder object.
        listener_callback(step_msg): Callback function for processing step messages.
    """

    def __init__(self):
        super().__init__("fogros2_rt_x_recorder")

        self.declare_parameter("dataset_name", "berkeley_fanuc_manipulation")
        self.dataset_name = self.get_parameter("dataset_name").value
        self.config = get_dataset_plugin_config_from_str(self.dataset_name)
        
        self.feature_spec = self.config.get_dataset_feature_spec()
        self.last_action = None
        self.last_observation = None
        self.last_step = None

        # TODO: more params
        self.storage_backend = SqliteConnector("fogros_rt_x.db")
        columns = {
            "episode_id": "INTEGER",
            "step": "BLOB",
            "observation": "BLOB",
            "action": "BLOB",
            "should_export": "INTEGER"
        }
        self.storage_backend.create_table(table_name=self.dataset_name, columns=columns)


        self.subscription = self.create_subscription(
            Step, "step_info", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.new_episode_notification_service = self.create_service(
            Empty,
            "new_episode_notification_service",
            self.new_episode_notification_service_callback,
        )

        # a random integer to identify the episode
        self.episode_id = random.randint(0, 1000000)
        self.step_counter = 0
        self.record_as_new_step = True

    def new_episode_notification_service_callback(self, request, response):
        self.get_logger().info("Received new_episode_notification_service request")
        self.episode_id = random.randint(0, 1000000)
        self.record_as_new_step = True
        return response

    def record_step(self):
        self.storage_backend.insert_data(
            table_name=self.dataset_name,
            data={
                "episode_id": self.episode_id,
                "step": pickle.dumps(self.last_step),
                "observation": pickle.dumps(self.last_observation),
                "action": pickle.dumps(self.last_action),
                "should_export": 1,
            },
        )

    def listener_callback(self, step_msg):
        """
        Callback function for processing step messages.

        This function is called whenever a step message is received. It converts the step message to a step tuple,
        records the step data, and updates the dataset writer.

        Args:
            step_msg (Step): The step message received.

        Returns:
            None
        """
        self.get_logger().warning(f"Received step: {str(step_msg)[:100]}")

        (
            self.last_observation,
            self.last_action,
            self.last_step,
        ) = self.feature_spec.convert_ros2_msg_to_step_tuple(step_msg)

        self.record_step()


def main(args=None):
    rclpy.init(args=args)

    dataset_recorder = DatasetRecorder()

    rclpy.spin(dataset_recorder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dataset_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
