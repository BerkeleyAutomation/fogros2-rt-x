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
import rosbag2_py as rosbag
from rclpy.node import Node
from .dataset_utils import *
from .dataset_spec import DatasetFeatureSpec, FeatureSpec
from .dataset_spec import tf_feature_definition_to_ros_msg_class_str
from .plugins.conf_base import *
import time
import tensorflow_datasets as tfds

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

        self.declare_parameter("per_episode_interval", 5)  # second
        self.per_episode_interval = self.get_parameter("per_episode_interval").value
        self.declare_parameter("per_step_interval", 0.2)  # second
        self.per_step_interval = self.get_parameter("per_step_interval").value

        self.declare_parameter(
            "replay_type", "as_separate_topics"
        )  # as_single_topic | as_separate_topics | both
        replay_type = self.get_parameter("replay_type").value

        self.declare_parameter("enable_bagging", True)

        self.dataset = load_rlds_dataset(dataset_name)
        self.logger = self.get_logger()
        self.dataset_info = get_dataset_info([dataset_name])
        self.logger.info("Loading Dataset " + str(self.dataset_info))

        self.episode = next(iter(self.dataset))
        self.dataset_features = self.dataset_info[0][1].features
        self.step_features = self.dataset_features["steps"]
        self.topics = list()
        self.episode_counter = 1
        self.init_topics_from_features(self.step_features)
        

        if replay_type == "as_separate_topics":
            self.topic_name_to_publisher_dict = dict()
            self.topic_name_to_recorder_dict = dict()
            self.init_publisher_separate_topics()
            # self.init_recorder_separate_topics()
        # elif replay_type == "as_single_topic":
        #     self.init_publisher_single_topic()
        # elif replay_type == "both":
        #     self.topic_name_to_publisher_dict = dict()
        #     self.init_publisher_separate_topics()
        #     self.init_publisher_single_topic()
        else:
            raise ValueError(
                "Invalid replay_type: "
                + str(replay_type)
                + ". Must be one of: as_separate_topics, as_single_topic."
            )

    def init_topics_from_features(self, features):
        for name, tf_feature in features.items():
            if isinstance(tf_feature, tfds.features.FeaturesDict):
                self.init_topics_from_features(tf_feature)
            else:
                if tf_feature.shape == () and tf_feature.dtype.is_bool:
                    # print("Adding to topics:", name, Scalar(dtype=tf.bool))
                    self.topics.append(FeatureSpec(name, Scalar(dtype=tf.bool)))
                else:
                    # print("Adding to topics:", name, tf_feature)
                    self.topics.append(FeatureSpec(name, tf_feature))
        

    def init_publisher_separate_topics(self):
        for topic in self.topics:
            publisher = self.create_publisher(
                topic.ros_type, topic.ros_topic_name, 10
            )
            self.topic_name_to_publisher_dict[topic.ros_topic_name] = publisher

        self.create_timer(
            self.per_episode_interval, self.timer_callback_separate_topics
        )

    def init_publisher_single_topic(self):
        self.publisher = self.create_publisher(Step, "step_info", 10)
        callback = self.timer_callback_single_topic
        self.create_timer(self.per_episode_interval, callback)

    def timer_callback_separate_topics(self):
        for step in self.episode["steps"]:
            for topic in self.topics:
                if topic.tf_name in step:
                    # Fetch from step data
                    msg = topic.convert_tf_tensor_data_to_ros2_msg(
                        step[topic.tf_name]
                    )
                    self.logger.info(
                        f"Publishing step {topic.tf_name} on topic {topic.ros_topic_name}"
                    )
                if type(step["observation"]) is dict and topic.tf_name in step["observation"]:
                    # Fetch from observation data
                    msg = topic.convert_tf_tensor_data_to_ros2_msg(
                        step["observation"][topic.tf_name]
                    )
                    self.logger.info(
                        f"Publishing observation {topic.tf_name} on topic {topic.ros_topic_name}"
                    )
                if type(step["action"]) is dict and topic.tf_name in step["action"]:
                    # Fetch from action data
                    msg = topic.convert_tf_tensor_data_to_ros2_msg(
                        step["action"][topic.tf_name]
                    )
                    self.logger.info(
                        f"Publishing action {topic.tf_name} on topic {topic.ros_topic_name}"
                    )
                
                self.topic_name_to_publisher_dict[topic.ros_topic_name].publish(msg)
            
            # self.check_last_step_update_recorder(step)
            time.sleep(self.per_step_interval)

        self.episode = next(iter(self.dataset))
        self.create_timer(
            self.per_episode_interval, self.timer_callback_separate_topics
        )
    
    def init_recorder_separate_topics(self):
        for topic in self.topics:
            ros_msg_type_string = tf_feature_definition_to_ros_msg_class_str(topic.tf_type)
            
            writer = rosbag.SequentialWriter()
            storage_options = rosbag._storage.StorageOptions(
                uri=f"bags/episode{self.episode_counter}/{topic.ros_topic_name}_bag", 
                storage_id="sqlite3"
            )
            converter_options = rosbag._storage.ConverterOptions("","")
            writer.open(storage_options, converter_options)

            topic_info = rosbag._storage.TopicMetadata(
                name=topic.ros_topic_name, 
                type=ros_msg_type_string, 
                serialization_format="cdr"
            )
            writer.create_topic(topic_info)

            subscription = self.create_subscription(
                topic.ros_type,
                topic.ros_topic_name, 
                self.create_topic_callback_function(topic.ros_topic_name), 
                10
            )

            self.topic_name_to_recorder_dict[topic.ros_topic_name] = (writer, subscription)

    def create_topic_callback_function(self, topic_name):
        self.logger.info(f"Creating topic callback function for {topic_name}")
        def topic_callback(self, msg=''):
            recorder, _ = self.topic_name_to_recorder_dict[topic_name]
            recorder.write(
                topic_name, 
                rclpy.serialization.serialize_message(msg), 
                self.get_clock().now().nanoseconds
            )
        return topic_callback
    
    def check_last_step_update_recorder(self, step):
        if step["is_last"]:
            self.logger.info(f"End of episode {self.episode_counter}")
            self.episode_counter += 1
            self.init_recorder_separate_topics()

    # def timer_callback_single_topic(self):
    #     for step in self.episode["steps"]:
    #         msg = self.feature_spec.convert_tf_step_to_ros2_msg(
    #             step, step["action"], step["observation"]
    #         )
    #         self.publisher.publish(msg)
    #     self.episode = next(iter(self.dataset))


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
