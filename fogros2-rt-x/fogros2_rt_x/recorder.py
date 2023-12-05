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

import rclpy
from rclpy.node import Node
from .dataset_utils import *
from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger
from envlogger import step_data
import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer
from .dataset_spec import DatasetFeatureSpec
from .dataset_conf import *
from .backend_writer import CloudBackendWriter
import dm_env
from .database_connector import BaseDataBaseConnector, BigQueryConnector

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

        self.observation_spec = OBSERVATION_SPEC
        self.action_spec = ACTION_SPEC
        self.step_spec = STEP_SPEC

        self.feature_spec = DatasetFeatureSpec(
            observation_spec=self.observation_spec,
            action_spec=self.action_spec,
            step_spec=self.step_spec,
        )

        self.dataset_config = self.feature_spec.to_dataset_config(
            dataset_name=DATASET_NAME
        )

        self.last_action = None
        self.last_observation = None
        self.last_step = None

        self.metadata_db = BigQueryConnector(
            project_name=BIG_QUERY_PROJECT,
            dataset_name=DATASET_NAME,
            table_name="metadata",
        )

        self.writer = CloudBackendWriter(
            data_directory=SAVE_PATH,
            max_episodes_per_file=1,
            ds_config=self.dataset_config,
            logger = self.get_logger(),
            metadata_database=self.metadata_db,
        )

        self.subscription = self.create_subscription(
            Step, "step_info", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

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

        if self.last_step["is_last"]:
            step_type = dm_env.StepType.LAST
        elif self.last_step["is_first"]:
            step_type = dm_env.StepType.FIRST
        else:
            step_type = dm_env.StepType.MID

        timestep = dm_env.TimeStep(
            step_type=step_type,
            reward=self.last_step["reward"],
            discount=self.last_step["discount"],
            observation=self.last_observation,
        )

        data = step_data.StepData(
            timestep=timestep, action=self.last_action, custom_data=None
        )

        if self.last_step["is_first"]:
            self.writer.record_step(data, is_new_episode=True)
        else:
            self.writer.record_step(data, is_new_episode=False)


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
