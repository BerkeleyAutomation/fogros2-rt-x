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
    """
    Class for managing ROS bag files.

    Args:
        bag_path (str): The path to the bag file.

    Attributes:
        bag_path (str): The path to the bag file.
        reader (AnyReader): The reader instance for reading the bag file.
        topic_name_to_tf_feature_map (dict): A dictionary mapping topic names to TensorFlow feature configurations.

    Methods:
        get_metadata: Get the metadata of the bag file.
        get_the_first_message_of_the_topic: Get the first message of a specific topic.
        get_tf_configuration: Get the TensorFlow configuration for a specific topic.
        generate_tensorflow_configuration_file: Generate a TensorFlow configuration file based on the specified topics.
        _get_data_from_raw_data: Get the data from raw data and convert it to TensorFlow tensor data.
        iterate_through_all_messages: Iterate through all messages in the bag file.

    """

    def __init__(
            self, bag_path
            ):
        """
        Initialize the BagManager.

        Args:
            bag_path (str): The path to the bag file.

        """
        self.bag_path = bag_path
        # create reader instance and open for reading
        self.reader = AnyReader([Path(bag_path)])
        self.reader.open()

        self.topic_name_to_tf_feature_map = {}

    def __del__(self):
        """
        Clean up resources when the BagManager is deleted.

        """
        self.reader.close()

    def get_metadata(self):
        """
        Get the metadata of the bag file.

        Returns:
            dict: A dictionary containing the metadata of the bag file.

        """
        metadata = {}
        metadata["bag_path"] = self.bag_path
        metadata["duraction"] = self.reader.duration
        metadata["start_time"] = self.reader.start_time
        metadata["end_time"] = self.reader.end_time
        metadata["message_count"] = self.reader.message_count
        
        # iterate through each topic and put the first message to metadata
        # TODO: later generate a gif 
        for topic_name in self.reader.topics:
            topic_name_in_sql = topic_name.replace("/", "_")
            msg = self.get_the_first_message_of_the_topic(self.reader, topic_name)
            topic_type = self.reader.topics[topic_name].msgtype.replace("/msg", "")
            # metadata[topic_name_in_sql + "_topic_type"] = topic_type
            print( self.reader.topics[topic_name])
            metadata[topic_name_in_sql + "_num_msgs"] = self.reader.topics[topic_name].msgcount
            # metadata[topic_name + "qos"] = self.reader.topics[topic_name].qos_profile
            if msg is not None:
                if topic_type == "sensor_msgs/Image":
                    # save the image as local file 
                    msg = to_native_class(msg)
                    import cv2 
                    import numpy as np
                    data = msg_to_numpy(msg, topic_type)
                    path = "/tmp/test" + topic_name_in_sql + self.bag_path.replace("/", "") + ".png"
                    cv2.imwrite(path, data)
                    metadata[topic_name_in_sql + "_sample"] = path
                elif topic_type == "std_msgs/String":
                    msg = to_native_class(msg)
                    metadata[topic_name_in_sql + "_sample"] = msg.data
                
        return metadata
    
    def get_the_first_message_of_the_topic(self, reader, topic):
        """
        Get the first message of a specific topic.

        Args:
            reader (AnyReader): The reader instance for reading the bag file.
            topic (str): The name of the topic.

        Returns:
            Any: The first message of the topic.

        """
        for connection, timestamp, rawdata in reader.messages(
            connections=reader.connections
        ):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == topic:
                return msg

    def get_tf_configuration(self, topic_name):
        """
        Get the TensorFlow configuration for a specific topic.

        Args:
            topic_name (str): The name of the topic.

        Returns:
            TensorFlowFeature: The TensorFlow feature configuration for the topic.

        """
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
        observation_topics,
        action_topics,
        step_topics,
    ):
        """
        Generate a TensorFlow configuration file based on the specified topics.

        Args:
            observation_topics (list): A list of observation topics.
            action_topics (list): A list of action topics.
            step_topics (list): A list of step topics.

        Returns:
            tuple: A tuple containing the observation spec, action spec, and step spec.

        """
        observatio_spec = []
        action_spec = []
        step_spec = []
        for topic_name in observation_topics:
            tf_feature = self.get_tf_configuration(topic_name)
            observatio_spec.append(FeatureSpec(topic_name, tf_feature))
            self.topic_name_to_tf_feature_map[topic_name] = tf_feature
        for topic_name in action_topics:
            tf_feature = self.get_tf_configuration(topic_name)
            action_spec.append(FeatureSpec(topic_name, tf_feature))
            self.topic_name_to_tf_feature_map[topic_name] = tf_feature
        for topic_name in step_topics:
            tf_feature = self.get_tf_configuration(topic_name)
            step_spec.append(FeatureSpec(topic_name, tf_feature))
            self.topic_name_to_tf_feature_map[topic_name] = tf_feature

        return observatio_spec, action_spec, step_spec


    def _get_data_from_raw_data(self, rawdata, connection):
        """
        Get the data from raw data and convert it to TensorFlow tensor data.

        Args:
            rawdata (bytes): The raw data.
            connection (Connection): The connection object.

        Returns:
            TensorFlow tensor data: The converted TensorFlow tensor data.

        """
        msg = self.reader.deserialize(rawdata, connection.msgtype)
        msg = to_native_class(msg)
        data = ros2_msg_data_to_tf_tensor_data(
            msg, self.topic_name_to_tf_feature_map[connection.topic]
        )
        return data
    
    def iterate_through_all_messages(
            self, 
            orchestrator,
            observation_topics,
            action_topics,
            step_topics,
        ):
        """
        Iterate through all messages in the bag file.

        Args:
            orchestrator (Orchestrator): The orchestrator object.
            observation_topics (list): A list of observation topics.
            action_topics (list): A list of action topics.
            step_topics (list): A list of step topics.

        """
        for connection, timestamp, rawdata in self.reader.messages(
            connections=self.reader.connections
        ):
            orchestrator.on_timestamp(timestamp, connection.topic)
            if connection.topic in observation_topics:
                data = self._get_data_from_raw_data(rawdata, connection)
                orchestrator.on_observation_topic(connection.topic, data)
                
            if connection.topic in action_topics:
                data = self._get_data_from_raw_data(rawdata, connection)
                orchestrator.on_action_topic(connection.topic, data)

            if connection.topic in step_topics:
                data = self._get_data_from_raw_data(rawdata, connection)
                orchestrator.on_step_topic(connection.topic, data)

