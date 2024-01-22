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
# from .dataset_utils import *
# from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger

# import tensorflow_datasets as tfds
# from envlogger.backends import tfds_backend_writer
# from .dataset_spec import DatasetFeatureSpec
# from .plugins.conf_base import *


from .database_connector import SqliteConnector
from std_srvs.srv import Empty
import random
import pickle 

from pathlib import Path
import pprint
from rosbags.highlevel import AnyReader

import ros2_numpy
from .dataset_spec import ros_multi_array_to_tf_dtype_map

# from __future__ import annotations

import importlib
import numpy 
import tensorflow as tf

NATIVE_CLASSES: dict = {}

def to_native_class(msg):
    """Convert rosbags message to native message.

    Args:
        msg: Rosbags message.

    Returns:
        Native message.

    """
    if isinstance(msg, (list)):
        return [to_native_class(x) for x in msg]
    msgtype: str = msg.__msgtype__
    if msgtype not in NATIVE_CLASSES:
        pkg, name = msgtype.rsplit('/', 1)
        NATIVE_CLASSES[msgtype] = getattr(importlib.import_module(pkg.replace('/', '.')), name)

    fields = {}
    for name, field in msg.__dataclass_fields__.items():
        if 'ClassVar' in field.type:
            continue
        value = getattr(msg, name)
        if '__msg__' in field.type:
            value = to_native_class(value)
        elif isinstance(value, list):
            value = [to_native_class(x) for x in value]
        elif isinstance(value, numpy.ndarray):
            value = value.tolist()
        fields[name] = value

    return NATIVE_CLASSES[msgtype](**fields)

class DatasetRecorder(Node):
    def __init__(self):
        super().__init__("fogros2_rt_x_recorder")
        observation_topics = []
        # create reader instance and open for reading
        self.reader = AnyReader([Path('./rosbag2_2024_01_22-02_59_18')])
        self.reader.open()
        self.messages = self.reader.messages(connections=self.reader.connections)
        self.topics = self.reader.topics

        self.generate_tensorflow_configuration_file(
            observation_topics = ["/wrist_image", "/image", "/end_effector_state", "/state"],
            action_topics = ["/action"],
            step_topics = ["/language_embedding", "/language_instruction", "/discount", "/reward"],
        )

    def get_the_first_message_of_the_topic(self, reader, topic):
        for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == topic:
                return msg

    def msg_to_numpy(self, msg, topic_type):
        if topic_type.endswith("MultiArray"):
            data = msg.data
            data_type = ros_multi_array_to_tf_dtype_map[topic_type.replace("/msg", "")]
            # TODO: check if empty
            # if len(data) == 0:
            #     print(f"[error] empty array for {tf_feature}, fill in with zeros")
            #     return numpy.zeros(tf_feature.shape, dtype=tf_feature.np_dtype)

            # Retrieve the shape information from the MultiArrayLayout
            original_shape = [dim.size for dim in msg.layout.dim]
            # Convert the data into a TensorFlow tensor
            tensor = tf.constant(data, dtype=data_type)

            # Reshape the tensor to its original shape
            reshaped_tensor = tf.reshape(tensor, original_shape)
            # Convert the data to a numpy array and then reshape it
            return reshaped_tensor.numpy()
        else:
            return ros2_numpy.numpify(msg)


    def get_tf_configuration(self, topic_name):
        msg = self.get_the_first_message_of_the_topic(self.reader, topic_name)
        topic_type = self.topics[topic_name].msgtype
        print(msg.__msgtype__)
        msg = to_native_class(msg)
        print(msg)

        data = self.msg_to_numpy(msg, topic_type)

        print(data)
        return data
        


    def generate_tensorflow_configuration_file(
            self,
            observation_topics = [],
            action_topics = [],
            step_topics = [],
    ):
        for topic_name in observation_topics:
            print(self.get_tf_configuration(topic_name))


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
