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

import numpy as np
import tensorflow as tf
import tensorflow_datasets as tfds
from fogros2_rt_x_msgs.msg import Step, Observation, Action
from cv_bridge import CvBridge
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
import ros2_numpy

tf_dtype_to_ros_class_map = {
    tf.float32: "std_msgs/Float32",
    tf.float64: "std_msgs/Float64",
    tf.int8: "std_msgs/Int8",
    tf.int16: "std_msgs/Int16",
    tf.int32: "std_msgs/Int32",
    tf.int64: "std_msgs/Int64",
    tf.uint8: "std_msgs/UInt8",
    tf.uint16: "std_msgs/UInt16",
    tf.uint32: "std_msgs/UInt32",
    tf.uint64: "std_msgs/UInt64",
    tf.bool: "std_msgs/Bool",
    tf.string: "std_msgs/String",
}

ros_class_to_tf_dtype_map = {v: k for k, v in tf_dtype_to_ros_class_map.items()}

tf_tensor_dtype_to_ros_multi_array_map = {
    tf.float32: "std_msgs/Float32MultiArray",
    tf.float64: "std_msgs/Float64MultiArray",
    tf.int8: "std_msgs/Int8MultiArray",
    tf.int16: "std_msgs/Int16MultiArray",
    tf.int32: "std_msgs/Int32MultiArray",
    tf.int64: "std_msgs/Int64MultiArray",
    tf.uint8: "std_msgs/UInt8MultiArray",
    tf.uint16: "std_msgs/UInt16MultiArray",
    tf.uint32: "std_msgs/UInt32MultiArray",
    tf.uint64: "std_msgs/UInt64MultiArray",
    tf.bool: "std_msgs/BoolMultiArray",
    tf.string: "std_msgs/StringMultiArray",
}

ros_multi_array_to_tf_dtype_map = {v: k for k, v in tf_tensor_dtype_to_ros_multi_array_map.items()}



def msg_to_numpy(msg, topic_type):
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
        

def ros2_msg_data_to_tf_tensor_data(ros2_attribute, tf_feature):
    """
    This function converts ROS (Robot Operating System) message attributes to TensorFlow dataset features.

    Parameters:
    ros2_attribute (any): The ROS attribute to be converted.
    tf_feature (tfds.core.dataset_info.FeatureConnector): The TensorFlow feature to be converted.

    Returns:
    any: The TensorFlow feature.
    """
    if isinstance(tf_feature, tfds.features.Image):
        if ros2_attribute == Image():
            print("[error] empty image, fill in with zeros")
            return np.zeros(tf_feature.shape, dtype=np.uint8)
        bridge = CvBridge()
        image_message = bridge.imgmsg_to_cv2(ros2_attribute)
        return image_message
    elif isinstance(tf_feature, tfds.features.Text):
        return ros2_attribute.data
    elif isinstance(tf_feature, tfds.features.Scalar):
        return ros2_attribute.data
    elif isinstance(tf_feature, tfds.features.Tensor):
        ros2_type = ros2_attribute.__class__.__name__
        if ros2_type.endswith("MultiArray"):
            # a workaround for the table querying
            # TODO: write a function instead of hash table checking
            ros2_type = "std_msgs/" + ros2_type 
        return msg_to_numpy(ros2_attribute, ros2_type)
    else:
        raise NotImplementedError(
            f"feature type {type(tf_feature)} for {tf_feature} not implemented"
        )


def cast_tensor_to_class_type(tensor, class_type):
    print(tensor.numpy(), tensor.numpy().shape)
    print(bool(tensor.numpy()))
    return class_type(tensor.numpy())


def tf_tensor_data_to_ros2_attribute_data(
    tensor, spec_attribute, ros2_type
):  # aka in numpy
    """
    This function converts TensorFlow tensors to ROS (Robot Operating System) message attributes.

    Parameters:
    tensor (tf.Tensor): The TensorFlow tensor to be converted.
    ros2_attribute (any): The ROS attribute to be converted.

    Returns:
    any: The ROS attribute.
    """
    print(type(spec_attribute), spec_attribute, ros2_type)
    if isinstance(spec_attribute, tfds.features.Image):
        bridge = CvBridge()
        converted_tensor = tensor
        converted_msg = bridge.cv2_to_imgmsg(converted_tensor.numpy())
        return converted_msg
    elif isinstance(spec_attribute, tfds.features.Text):
        msg = ros2_type()
        msg.data = str(tensor)
        return msg
    elif isinstance(spec_attribute, tfds.features.Scalar):
        msg = ros2_type()
        # https://stackoverflow.com/questions/9452775/converting-numpy-dtypes-to-native-python-types
        data = tensor.numpy().item()
        msg.data = data
        return msg
    elif isinstance(spec_attribute, tfds.features.Tensor):
        # convert tensor to ros2 std_msgs/MultiArray

        tensor_list = tf.reshape(tensor, [-1]).numpy().tolist()  # flatten it
        # dim = MultiArrayDimension()
        # dim.label = 'tensor_size'
        # print(tensor.shape, tensor.size)
        # dim.size = tensor.size
        # dim.stride = tensor.size  # not important (?)

        # Determine the shape and calculate strides
        shape = tensor.shape
        strides = [1]
        for s in reversed(shape[:-1]):
            strides.insert(0, strides[0] * s)

        # Create MultiArrayDimension for each dimension
        dims = []
        for i, (dim_size, stride) in enumerate(zip(shape, strides)):
            dim = MultiArrayDimension()
            dim.label = f"dim_{i}"
            dim.size = dim_size
            dim.stride = stride
            dims.append(dim)

        layout = MultiArrayLayout()
        layout.dim = dims
        layout.data_offset = 0

        # create a ros2 std_msgs/MultiArray
        multiarray = ros2_type()
        # set the data field of the ros2 std_msgs/MultiArray
        multiarray.layout = layout
        multiarray.data = tensor_list

        return multiarray
    else:
        raise NotImplementedError(
            f"feature type {type(spec_attribute)} for {spec_attribute} not implemented"
        )


from pydoc import locate


def get_ROS_class(ros_message_type, srv=False):
    """
    Returns the ROS message class from ros_message_type.
    :return AnyMsgClass: Class of the ROS message.
    """
    try:
        package_name, message_name = ros_message_type.split("/")
    except ValueError:
        raise ValueError(
            "ros_message_type should be in the shape of package_msgs/Message"
            + " (it was "
            + ros_message_type
            + ")"
        )
    if not srv:
        msg_class = locate("{}.msg.{}".format(package_name, message_name))
    else:
        msg_class = locate("{}.srv.{}".format(package_name, message_name))
    if msg_class is None:
        if srv:
            msg_or_srv = ".srv"
        else:
            msg_or_srv = ".msg"
        raise ValueError(
            "ros_message_type could not be imported. ("
            + ros_message_type
            + ', as "from '
            + package_name
            + msg_or_srv
            + " import "
            + message_name
            + '" failed.'
        )
    return msg_class


def tf_feature_definition_to_ros_msg_class_str(feature):
    if isinstance(feature, tfds.features.Image):
        return f"sensor_msgs/Image"
    elif isinstance(feature, tfds.features.Text):
        return f"std_msgs/String"
    elif isinstance(feature, tfds.features.Scalar):
        return tf_dtype_to_ros_class_map[feature.dtype]
    elif isinstance(feature, tfds.features.Tensor):
        return tf_tensor_dtype_to_ros_multi_array_map[feature.dtype]
    else:
        raise NotImplementedError(
            f"feature type {type(feature)} for {feature} not implemented"
        )


def tf_feature_definition_to_ros_msg_class(feature):
    return get_ROS_class(tf_feature_definition_to_ros_msg_class_str(feature))


def tf_feature_definition_to_ros_msg_str(name, feature, default_val=None):
    """
    This function converts TensorFlow dataset features to ROS (Robot Operating System) message definitions.

    Parameters:
    name (str): The name of the feature.
    feature (tfds.core.dataset_info.FeatureConnector): The TensorFlow feature to be converted.

    Returns:
    str: The ROS message definition.

    Raises:
    NotImplementedError: If the feature type is not implemented.
    """

    return f"{tf_feature_definition_to_ros_msg_class_str(feature)} {name}{' ' + str(default_val) if default_val else ''}"


class FeatureSpec:
    def __init__(
        self,
        tf_name,
        tf_type,
        ros_topic_name=None,
        default_value=None,
        is_triggering_topic=False,
    ) -> None:
        self.tf_name = tf_name
        self.tf_type = tf_type
        self.ros_topic_name = ros_topic_name if ros_topic_name else tf_name
        self.ros_type = tf_feature_definition_to_ros_msg_class(tf_type)
        self.is_triggering_topic = is_triggering_topic
        self.default_value = default_value

    def convert_tf_tensor_data_to_ros2_msg(self, tensor_data):
        """
        This function converts TensorFlow tensors to ROS (Robot Operating System) message attributes.

        Parameters:
        tf_data (tf.Tensor): The TensorFlow tensor to be converted.

        Returns:
        any: The ROS attribute.
        """
        return tf_tensor_data_to_ros2_attribute_data(
            tensor_data, self.tf_type, self.ros_type
        )


class DatasetFeatureSpec:
    def __init__(self, observation_spec, action_spec, step_spec):
        """
        Initializes a DatasetFeatureSpec object.

        Args:
            observation_spec (dict): The specification of observation features.
            action_spec (dict): The specification of action features.
            step_spec (dict): The specification of step features.
        """
        self.observation_spec = observation_spec
        self.action_spec = action_spec
        self.step_spec = step_spec

        self.observation_tf_dict = self.spec_to_tf_type_dict(observation_spec)
        self.action_tf_dict = self.spec_to_tf_type_dict(action_spec)
        self.step_tf_dict = self.spec_to_tf_type_dict(step_spec)

        self.check_spec(observation_spec)
        self.check_spec(action_spec)
        self.check_spec(step_spec)

    def check_spec(self, spec):
        """
        Checks the feature specification.

        Args:
            spec (list): The feature specification.

        Raises:
            ValueError: If the feature specification is invalid.
        """
        if not isinstance(spec, list):
            raise ValueError(f"spec must be a list, got {type(spec)} instead: {spec}")
        for feature in spec:
            if not isinstance(feature, FeatureSpec):
                raise ValueError(
                    f"feature must be a FeatureSpec, got {type(feature)} instead: {feature}"
                )

    def check_triggering_topic(self):
        # check if there is a triggering topic
        triggering_topic_count = 0
        for feature in self.action_spec:
            if feature.is_triggering_topic:
                triggering_topic_count += 1
        if triggering_topic_count > 1:
            raise ValueError(
                f"feature spec can only have one triggering topic, got {triggering_topic_count} instead"
            )

        if triggering_topic_count == 0:
            raise ValueError(
                f"feature spec must have one triggering topic, got {triggering_topic_count} instead"
            )

    def spec_to_tf_type_dict(self, spec):
        """
        Converts the feature specification to a dictionary.

        Args:
            spec (list): The feature specification.

        Returns:
            dict: The feature specification in dictionary format.
        """
        return {f.tf_name: f.tf_type for f in spec}

    def convert_ros2_msg_to_step_tuple(self, ros2_msg):
        """
        Converts a ROS2 message to a step tuple.

        Args:
            ros2_msg: The ROS2 message.

        Returns:
            tuple: The step tuple containing observation, action, and step information.
        """
        observation = dict()
        action = dict()
        step = dict()

        for k, v in self.observation_tf_dict.items():
            observation[k] = ros2_msg_data_to_tf_tensor_data(
                getattr(ros2_msg.observation, k), v
            )
        for k, v in self.action_tf_dict.items():
            action[k] = ros2_msg_data_to_tf_tensor_data(getattr(ros2_msg.action, k), v)

        for k, v in self.step_tf_dict.items():
            step[k] = ros2_msg_data_to_tf_tensor_data(getattr(ros2_msg, k), v)

        return observation, action, step

    def convert_tf_step_to_ros2_msg(self, step, action, observation):
        """
        Converts a step tuple to a ROS2 message.

        Args:
            step (dict): The step information.
            action (dict): The action information.
            observation (dict): The observation information.

        Returns:
            ros2_msg: The converted ROS2 message.
        """
        ros2_msg = Step()
        ros2_msg.observation = Observation()
        ros2_msg.action = Action()
        for feature in self.step_spec:
            if feature.tf_name == "discount":
                # TODO: missing field
                from std_msgs.msg import Float64

                num = Float64()
                num.data = 1.0
                setattr(ros2_msg, feature.tf_name, num)
                continue
            ros2_msg_type = type(getattr(ros2_msg, feature.tf_name))
            converted_value_to_ros2 = tf_tensor_data_to_ros2_attribute_data(
                step[feature.tf_name], feature.tf_type, ros2_msg_type
            )
            setattr(ros2_msg, feature.tf_name, converted_value_to_ros2)
        for feature in self.observation_spec:
            ros2_msg_type = type(getattr(ros2_msg.observation, feature.tf_name))
            converted_value_to_ros2 = tf_tensor_data_to_ros2_attribute_data(
                observation[feature.tf_name], feature.tf_type, ros2_msg_type
            )
            setattr(ros2_msg.observation, feature.tf_name, converted_value_to_ros2)

        if type(action) is not dict:
            feature = self.action_spec[0]  # only one element in feature spec
            ros2_msg_type = type(getattr(ros2_msg.action, feature.tf_name))
            converted_value_to_ros2 = tf_tensor_data_to_ros2_attribute_data(
                action, feature.tf_type, ros2_msg_type
            )
            setattr(ros2_msg.action, feature.tf_name, converted_value_to_ros2)
        else:
            for feature in self.action_spec:
                ros2_msg_type = type(getattr(ros2_msg.action, feature.tf_name))
                converted_value_to_ros2 = tf_tensor_data_to_ros2_attribute_data(
                    action[feature.tf_name], feature.tf_type, ros2_msg_type
                )
                setattr(ros2_msg.action, feature.tf_name, converted_value_to_ros2)

        return ros2_msg

    def tf_spec_definition_to_ros2_msg_definition(self, spec):
        """
        Converts a TensorFlow feature specification to a ROS2 message definition.

        Args:
            spec (dict): The TensorFlow feature specification.

        Returns:
            str: The ROS2 message definition.
        """
        return "\n".join(
            [
                tf_feature_definition_to_ros_msg_str(
                    name, feature, default_val=feature.default_value
                )
                for name, feature in spec.items()
            ]
        )

    def feature_spec_list_to_ros2_msg_definition(self, feature_spec_list):
        """
        Converts a list of feature specifications to a ROS2 message definition.

        Args:
            feature_spec_list (list): The list of feature specifications.

        Returns:
            str: The ROS2 message definition.
        """
        return "\n".join(
            [
                tf_feature_definition_to_ros_msg_str(
                    spec.tf_name, spec.tf_type, default_val=spec.default_value
                )
                for spec in feature_spec_list
            ]
        )


# def cast_tensor_to_class_type(tensor, class_type):
#     return class_type(tensor.numpy())

# def cast_value_to_class_type(tensor, class_type):
#     return class_type(tensor)

# def tf_step_to_ros2_msg(step):
#     ros2_step = Step()
#     ros2_step.observation = Observation()
#     ros2_step.action = Action()
#     ros2_step.reward = cast_tensor_to_class_type(step['reward'], type(ros2_step.reward))
#     ros2_step.is_first = cast_tensor_to_class_type(step['is_first'], type(ros2_step.is_first))
#     ros2_step.is_last = cast_tensor_to_class_type(step['is_last'], type(ros2_step.is_last))
#     ros2_step.is_terminal = cast_tensor_to_class_type(step['is_terminal'], type(ros2_step.is_terminal))
#     for k, v in step['observation'].items():
#         if k == 'image':
#             continue
#         field_type = type(getattr(ros2_step.observation, k))
#         print(field_type, k, str(field_type))
#         if str(field_type) == "<class 'array.array'>":
#             # TODO: assume all float32
#             setattr(ros2_step.observation, k, [float(x) for x in v.numpy()])
#         elif str(field_type) == "<class 'str'>":
#             setattr(ros2_step.observation, k, str(v.numpy()))
#         else:
#             setattr(ros2_step.observation, k, cast_value_to_class_type(v, field_type))
#         # print(type(getattr(ros2_step.observation, k)), v.numpy(), type(v.numpy()[0]))
#         # setattr(ros2_step.observation, k, [float(x) for x in v.numpy()])
#         # setattr(ros2_step.observation, k, cast_value_to_class_type(v, getattr(ros2_step.observation, k)))
#     for k, v in step['action'].items():
#         field_type = type(getattr(ros2_step.action, k))
#         print(field_type, k, str(field_type))
#         if str(field_type) == "<class 'array.array'>":
#             # TODO: assume all float32
#             setattr(ros2_step.action, k, [float(x) for x in v.numpy()])
#         elif str(field_type) == "<class 'str'>":
#             setattr(ros2_step.action, k, str(v.numpy()))
#         else:
#             setattr(ros2_step.action, k, cast_value_to_class_type(v, field_type))
#     return ros2_step
