



from rosbags.highlevel import AnyReader

from pathlib import Path
import pprint

import ros2_numpy
import tensorflow as tf
from .dataset_spec import ros_multi_array_to_tf_dtype_map, ros_class_to_tf_dtype_map, FeatureSpec
from .dataset_utils import to_native_class
from .conf_base import *

class BagManager():
    def __init__(self):
        # create reader instance and open for reading
        self.reader = AnyReader([Path('./datasets/rosbag2_2024_01_22-02_59_18')])
        self.reader.open()
        # self.messages = self.reader.messages(connections=self.reader.connections)
        # self.topics = self.reader.topics
        
    def get_the_first_message_of_the_topic(self, reader, topic):
        for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == topic:
                return msg

    def msg_to_numpy(self, msg, topic_type):
        if topic_type in ros_multi_array_to_tf_dtype_map:
            data = msg.data
            data_type = ros_multi_array_to_tf_dtype_map[topic_type]
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
        topic_type = self.reader.topics[topic_name].msgtype.replace("/msg", "")

        # special case
        if topic_type == "std_msgs/String":
            return Text()
        elif topic_type in ros_class_to_tf_dtype_map:
            return Scalar(dtype = ros_class_to_tf_dtype_map[topic_type])
        else:
            msg = to_native_class(msg)
            data = self.msg_to_numpy(msg, topic_type)
            tensor_shape = data.shape
            tensor_dtype = data.dtype
            return Tensor(
                shape=tensor_shape,
                dtype=tensor_dtype,
            )

    def generate_tensorflow_configuration_file(
            self,
            observation_topics = [],
            action_topics = [],
            step_topics = [],
    ):
        observatio_spec = []
        action_spec = []
        step_spec = []
        for topic_name in observation_topics:
            observatio_spec.append(FeatureSpec(topic_name, self.get_tf_configuration(topic_name)))
        for topic_name in action_topics:
            action_spec.append(FeatureSpec(topic_name, self.get_tf_configuration(topic_name)))
        for topic_name in step_topics:
            step_spec.append(FeatureSpec(topic_name, self.get_tf_configuration(topic_name)))
        
        return observatio_spec, action_spec, step_spec