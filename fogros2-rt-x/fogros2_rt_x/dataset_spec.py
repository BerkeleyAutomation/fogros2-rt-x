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


import tensorflow as tf
import tensorflow_datasets as tfds
from fogros2_rt_x_msgs.msg import Step, Observation, Action
from cv_bridge import CvBridge


def tf_feature_to_ros_msg_definition(name, feature):
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

    if isinstance(feature, tfds.features.Image):
        return f"sensor_msgs/Image {name}"
    elif isinstance(feature, tfds.features.Text):
        return f"string {name}"
    elif isinstance(feature, tfds.features.Scalar):
        return f"{feature.dtype.name} {name}"
    elif isinstance(feature, tfds.features.Tensor):
        return f"{feature.np_dtype.name}[] {name}"
    else:
        raise NotImplementedError(
            f"feature type {type(feature)} for {feature} not implemented"
        )


def ros2_attribute_to_tf_feature(ros2_attribute, tf_feature):
    """
    This function converts ROS (Robot Operating System) message attributes to TensorFlow dataset features.

    Parameters:
    ros2_attribute (any): The ROS attribute to be converted.
    tf_feature (tfds.core.dataset_info.FeatureConnector): The TensorFlow feature to be converted.

    Returns:
    any: The TensorFlow feature.
    """
    if isinstance(tf_feature, tfds.features.Image):
        bridge = CvBridge()
        image_message = bridge.imgmsg_to_cv2(ros2_attribute)
        return image_message
    elif isinstance(tf_feature, tfds.features.Text):
        return ros2_attribute
    elif isinstance(tf_feature, tfds.features.Scalar):
        return ros2_attribute
    elif isinstance(tf_feature, tfds.features.Tensor):
        return list(ros2_attribute)
    else:
        raise NotImplementedError(
            f"feature type {type(tf_feature)} for {tf_feature} not implemented"
        )


def cast_tensor_to_class_type(tensor, class_type):
    return class_type(tensor.numpy())


def tf_tensor_to_ros2_attribute(tensor, spec_attribute, ros2_type):  # aka in numpy
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
        return str(tensor)
    elif isinstance(spec_attribute, tfds.features.Scalar):
        return cast_tensor_to_class_type(tensor, ros2_type)
    elif isinstance(spec_attribute, tfds.features.Tensor):
        # TODO: need to handle 2D array here
        tensor_dtype = tensor.dtype
        if tensor_dtype == tf.float32 or tensor_dtype == tf.float64:
            return [float(x) for x in tensor.numpy()]
        elif tensor_dtype == tf.int32 or tensor_dtype == tf.int64:
            return [int(x) for x in tensor.numpy()]
        elif tensor_dtype == tf.bool:
            return [bool(x) for x in tensor.numpy()]
        elif tensor_dtype == tf.string:
            return str(tensor.numpy())
        else:
            try:
                return [tensor_dtype(x) for x in tensor.numpy()]
            except Exception as e:
                raise NotImplementedError(
                    f"feature type {type(spec_attribute)} for {spec_attribute} not implemented"
                )
    else:
        raise NotImplementedError(
            f"feature type {type(spec_attribute)} for {spec_attribute} not implemented"
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

    def spec_to_ros2_message_definition(self, spec):
        """
        Converts the feature specification to a ROS2 message definition.

        Args:
            spec (dict): The feature specification.

        Returns:
            str: The ROS2 message definition.
        """
        return "\n".join(
            [tf_feature_to_ros_msg_definition(k, v) for k, v in spec.items()]
        )

    def to_dataset_config(self, dataset_name):
        """
        Converts the DatasetFeatureSpec to a tfds.rlds.rlds_base.DatasetConfig object.

        Args:
            dataset_name (str): The name of the dataset.

        Returns:
            tfds.rlds.rlds_base.DatasetConfig: The dataset configuration.
        """
        return tfds.rlds.rlds_base.DatasetConfig(
            name=dataset_name,
            observation_info=self.observation_spec,
            action_info=self.action_spec,
            reward_info=self.step_spec["reward"],
            discount_info=self.step_spec["discount"],
            step_metadata_info={
                "is_first": self.step_spec["is_first"],
                "is_last": self.step_spec["is_last"],
                "is_terminal": self.step_spec["is_terminal"],
            },
        )

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

        for k, v in self.observation_spec.items():
            observation[k] = ros2_attribute_to_tf_feature(
                getattr(ros2_msg.observation, k), v
            )
        for k, v in self.action_spec.items():
            action[k] = ros2_attribute_to_tf_feature(getattr(ros2_msg.action, k), v)
        for k, v in self.step_spec.items():
            step[k] = ros2_attribute_to_tf_feature(getattr(ros2_msg, k), v)

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
        for k, v in self.step_spec.items():
            if k == "discount":
                #TODO: currently skip discount because the original dataset
                # does not have discount
                continue
            ros2_msg_type = type(getattr(ros2_msg, k))
            converted_value_to_ros2 = tf_tensor_to_ros2_attribute(
                step[k], v, ros2_msg_type
            )
            setattr(ros2_msg, k, converted_value_to_ros2)
        for k, v in self.observation_spec.items():
            ros2_msg_type = type(getattr(ros2_msg.observation, k))
            converted_value_to_ros2 = tf_tensor_to_ros2_attribute(
                observation[k], v, ros2_msg_type
            )
            setattr(ros2_msg.observation, k, converted_value_to_ros2)
        for k, v in self.action_spec.items():
            ros2_msg_type = type(getattr(ros2_msg.action, k))
            converted_value_to_ros2 = tf_tensor_to_ros2_attribute(
                action[k], v, ros2_msg_type
            )
            setattr(ros2_msg.action, k, converted_value_to_ros2)

        return ros2_msg


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
