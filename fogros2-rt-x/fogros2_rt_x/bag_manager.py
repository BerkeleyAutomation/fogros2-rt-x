from rosbags.highlevel import AnyReader

from pathlib import Path
import pprint

import tensorflow as tf
from .dataset_spec import (
    ros_multi_array_to_tf_dtype_map,
    ros_class_to_tf_dtype_map,
    FeatureSpec,
    msg_to_numpy,
    ros2_msg_data_to_tf_tensor_data,
)
from .dataset_utils import to_native_class
from .conf_base import *



class BagManager:
    def __init__(self, 
                 orchestrator, 
                 observation_topics=[], 
                 action_topics=[], 
                 step_topics=[],
                 ):
        self.observation_topics = observation_topics
        self.action_topics = action_topics
        self.step_topics = step_topics
        self.orchestrator = orchestrator

        # create reader instance and open for reading
        self.reader = AnyReader([Path("./datasets/rosbag2_2024_01_22-02_59_18")])
        self.reader.open()

        self.topic_name_to_tf_feature_map = {}

    def __del__(self):
        self.reader.close()

    def get_the_first_message_of_the_topic(self, reader, topic):
        for connection, timestamp, rawdata in reader.messages(
            connections=reader.connections
        ):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == topic:
                return msg

    def get_tf_configuration(self, topic_name):
        msg = self.get_the_first_message_of_the_topic(self.reader, topic_name)
        topic_type = self.reader.topics[topic_name].msgtype.replace("/msg", "")

        # special case
        if topic_type == "std_msgs/String":
            return Text()
        elif topic_type in ros_class_to_tf_dtype_map:
            return Scalar(dtype=ros_class_to_tf_dtype_map[topic_type])
        elif topic_type == "sensor_msgs/Image":
            msg = to_native_class(msg)
            data = msg_to_numpy(msg, topic_type)
            tensor_shape = data.shape
            tensor_dtype = data.dtype
            return Image(
                shape=tensor_shape,
                dtype=tensor_dtype,
            )
        else:
            msg = to_native_class(msg)
            data = msg_to_numpy(msg, topic_type)
            tensor_shape = data.shape
            tensor_dtype = data.dtype
            return Tensor(
                shape=tensor_shape,
                dtype=tensor_dtype,
            )

    def generate_tensorflow_configuration_file(
        self,
    ):
        observatio_spec = []
        action_spec = []
        step_spec = []
        for topic_name in self.observation_topics:
            tf_feature = self.get_tf_configuration(topic_name)
            observatio_spec.append(FeatureSpec(topic_name, tf_feature))
            self.topic_name_to_tf_feature_map[topic_name] = tf_feature
        for topic_name in self.action_topics:
            tf_feature = self.get_tf_configuration(topic_name)
            action_spec.append(FeatureSpec(topic_name, tf_feature))
            self.topic_name_to_tf_feature_map[topic_name] = tf_feature
        for topic_name in self.step_topics:
            tf_feature = self.get_tf_configuration(topic_name)
            step_spec.append(FeatureSpec(topic_name, tf_feature))
            self.topic_name_to_tf_feature_map[topic_name] = tf_feature

        return observatio_spec, action_spec, step_spec


    def _get_data_from_raw_data(self, rawdata, connection):
        msg = self.reader.deserialize(rawdata, connection.msgtype)
        msg = to_native_class(msg)
        data = ros2_msg_data_to_tf_tensor_data(
            msg, self.topic_name_to_tf_feature_map[connection.topic]
        )
        return data
    
    def iterate_through_all_messages(self):
        for connection, timestamp, rawdata in self.reader.messages(
            connections=self.reader.connections
        ):
            self.orchestrator.on_timestamp(timestamp, connection.topic)
            if connection.topic in self.observation_topics:
                data = self._get_data_from_raw_data(rawdata, connection)
                self.orchestrator.on_observation_topic(connection.topic, data)
                
            if connection.topic in self.action_topics:
                data = self._get_data_from_raw_data(rawdata, connection)
                self.orchestrator.on_action_topic(connection.topic, data)

            if connection.topic in self.step_topics:
                data = self._get_data_from_raw_data(rawdata, connection)
                self.orchestrator.on_step_topic(connection.topic, data)

