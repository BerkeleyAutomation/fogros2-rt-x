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
from .dataset_spec import DatasetFeatureSpec
from .plugins.conf_base import *
import time


class DatasetReplayer(Node):
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
        super().__init__("fogros2_rt_x_replayer")

        self.declare_parameter("dataset_name", "berkeley_fanuc_manipulation")
        dataset_name = self.get_parameter("dataset_name").value
        self.config = get_dataset_plugin_config_from_str(dataset_name)
        self.feature_spec = self.config.get_dataset_feature_spec()

        self.declare_parameter("per_episode_interval", 5)  # second
        self.per_episode_interval = self.get_parameter("per_episode_interval").value
        self.declare_parameter("per_step_interval", 0.2)  # second
        self.per_step_interval = self.get_parameter("per_step_interval").value

        self.declare_parameter(
            "replay_type", "as_single_topic"
        )  # as_single_topic | as_separate_topics
        replay_type = self.get_parameter("replay_type").value

        self.dataset = load_rlds_dataset(dataset_name)
        self.logger = self.get_logger()
        self.logger.info("Loading Dataset " + str(get_dataset_info([dataset_name])))

        self.episode = next(iter(self.dataset))

        if replay_type == "as_separate_topics":
            self.topic_name_to_publisher_dict = dict()
            self.init_publisher_separate_topics()
        elif replay_type == "as_single_topic":
            self.init_publisher_single_topic()
        else:
            raise ValueError(
                "Invalid replay_type: "
                + str(replay_type)
                + ". Must be one of: as_separate_topics, as_single_topic."
            )

    def init_publisher_separate_topics(self):
        for observation in self.feature_spec.observation_spec:
            publisher = self.create_publisher(
                observation.ros_type, observation.ros_topic_name, 10
            )
            self.topic_name_to_publisher_dict[observation.ros_topic_name] = publisher

        for action in self.feature_spec.action_spec:
            publisher = self.create_publisher(
                action.ros_type, action.ros_topic_name, 10
            )
            self.topic_name_to_publisher_dict[action.ros_topic_name] = publisher

        for step in self.feature_spec.step_spec:
            publisher = self.create_publisher(step.ros_type, step.ros_topic_name, 10)
            self.topic_name_to_publisher_dict[step.ros_topic_name] = publisher

        self.create_timer(
            self.per_episode_interval, self.timer_callback_separate_topics
        )

    def init_publisher_single_topic(self):
        self.publisher = self.create_publisher(Step, "step_info", 10)
        callback = self.timer_callback_single_topic
        self.create_timer(self.per_episode_interval, callback)

    def timer_callback_separate_topics(self):
        for step in self.episode["steps"]:
            for observation in self.feature_spec.observation_spec:
                if observation.tf_name not in step["observation"]:
                    self.logger.warn(
                        f"Observation {observation.tf_name} not found in step data"
                    )
                    continue
                msg = observation.convert_tf_tensor_data_to_ros2_msg(
                    step["observation"][observation.tf_name]
                )
                self.logger.info(
                    f"Publishing observation {observation.tf_name} on topic {observation.ros_topic_name}"
                )
                self.topic_name_to_publisher_dict[observation.ros_topic_name].publish(
                    msg
                )

            for action in self.feature_spec.action_spec:
                if type(step["action"]) is not dict:
                    # action is only one tensor/datatype, not a dictionary
                    msg = action.convert_tf_tensor_data_to_ros2_msg(step["action"])
                else:
                    if action.tf_name not in step["action"]:
                        self.logger.warn(
                            f"Action {action.tf_name} not found in step data"
                        )
                        continue
                    msg = action.convert_tf_tensor_data_to_ros2_msg(
                        step["action"][action.tf_name]
                    )
                self.logger.info(
                    f"Publishing action {action.tf_name} on topic {action.ros_topic_name}"
                )
                self.topic_name_to_publisher_dict[action.ros_topic_name].publish(msg)

            for step_feature in self.feature_spec.step_spec:
                if step_feature.tf_name not in step:
                    self.logger.warn(
                        f"Step {step_feature.tf_name} not found in step data"
                    )
                    continue
                msg = step_feature.convert_tf_tensor_data_to_ros2_msg(
                    step[step_feature.tf_name]
                )
                self.logger.info(
                    f"Publishing step {step_feature.tf_name} on topic {step_feature.ros_topic_name}"
                )
                self.topic_name_to_publisher_dict[step_feature.ros_topic_name].publish(
                    msg
                )

            time.sleep(self.per_step_interval)

        self.episode = next(iter(self.dataset))

    def timer_callback_single_topic(self):
        for step in self.episode["steps"]:
            msg = self.feature_spec.convert_tf_step_to_ros2_msg(
                step, step["action"], step["observation"]
            )
            self.publisher.publish(msg)
        self.episode = next(iter(self.dataset))


def main(args=None):
    # tf.experimental.numpy.experimental_enable_numpy_behavior()

    rclpy.init(args=args)
    node = DatasetReplayer()

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
