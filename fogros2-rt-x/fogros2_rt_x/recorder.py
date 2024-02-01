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
from rclpy.serialization import serialize_message
from .dataset_utils import *
from .dataset_spec import DatasetFeatureSpec, FeatureSpec
from .dataset_spec import tf_feature_definition_to_ros_msg_class_str
from .plugins.conf_base import *
from .replayer_common import in_episode
import time
import tensorflow_datasets as tfds

class DatasetRecorder(Node):
    """
    A class for recording dataset episodes in ROS2.

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

        self.declare_parameter("dataset_name", "berkeley_fanuc_manipulation")
        dataset_name = self.get_parameter("dataset_name").value

        self.dataset = load_rlds_dataset(dataset_name)
        self.logger = self.get_logger()
        self.dataset_info = get_dataset_info([dataset_name])
        self.episode = next(iter(self.dataset))
        self.dataset_features = self.dataset_info[0][1].features
        self.step_features = self.dataset_features["steps"]
        self.topics = list()
        self.episode_counter = 0
        self.init_topics_from_features(self.step_features)
        
        self.writer = rosbag.SequentialWriter()
        self.topic_name_to_subscription_dict = dict()
        self.topic_callback_dict = dict()
        self.init_writter_from_topics()

        time.sleep(10)
        self.init_writter_from_topics()

    def init_topics_from_features(self, features):
        for name, tf_feature in features.items():
            if isinstance(tf_feature, tfds.features.FeaturesDict):
                self.init_topics_from_features(tf_feature)
            else:
                if tf_feature.shape == () and tf_feature.dtype.is_bool:
                    self.topics.append(FeatureSpec(name, Scalar(dtype=tf.bool)))
                else:
                    self.topics.append(FeatureSpec(name, tf_feature))
    
    def add_topic_callback_function_to_dict(self, topic_name):
        def topic_callback(self, msg):
            self.writer.write(
                topic_name, 
                serialize_message(msg), 
                self.get_clock().now().nanoseconds
            )
        def self_wrapper(instance):
            def msg_wrapper(msg):
                return topic_callback(instance, msg)
            return msg_wrapper
            
        self.topic_callback_dict[topic_name] = self_wrapper
    
    def init_writter_from_topics(self):
        self.episode_counter += 1
        storage_options = rosbag._storage.StorageOptions(
            uri=f"rosbags/episode_{self.episode_counter}", 
            storage_id="sqlite3"
        )
        converter_options = rosbag._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        for topic in self.topics:
            ros_type_str = tf_feature_definition_to_ros_msg_class_str(topic.tf_type)

            topic_info = rosbag._storage.TopicMetadata(
                name=topic.ros_topic_name, 
                type=ros_type_str, 
                serialization_format="cdr"
            )
            self.writer.create_topic(topic_info)

            self.add_topic_callback_function_to_dict(topic.ros_topic_name)

            subscription = self.create_subscription(
                topic.ros_type, 
                topic.ros_topic_name, 
                self.topic_callback_dict[topic.ros_topic_name](self), 
                10
            )
            subscription

def main(args=None):

    rclpy.init(args=args)
    node = DatasetRecorder()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()