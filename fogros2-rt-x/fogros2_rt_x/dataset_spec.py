
import tensorflow as tf
import tensorflow_datasets as tfds
from fogros2_rt_x_msgs.msg import Step, Observation, Action

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
        return f'sensor_msgs/Image {name}'
    elif isinstance(feature, tfds.features.Text):
        return f'string {name}'
    elif isinstance(feature, tfds.features.Scalar):
        return f'{feature.dtype.name} {name}'
    elif isinstance(feature, tfds.features.Tensor):
        return f'{feature.np_dtype.name}[] {name}'
    else:
        raise NotImplementedError(f'feature type {type(feature)} for {feature} not implemented')

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
        return ros2_attribute.numpy()
    elif isinstance(tf_feature, tfds.features.Text):
        return ros2_attribute.numpy()
    elif isinstance(tf_feature, tfds.features.Scalar):
        return ros2_attribute.numpy()
    elif isinstance(tf_feature, tfds.features.Tensor):
        return ros2_attribute.numpy()
    else:
        raise NotImplementedError(f'feature type {type(tf_feature)} for {tf_feature} not implemented')

def cast_tensor_to_class_type(tensor, class_type):
    return class_type(tensor.numpy())

def tf_tensor_to_ros2_attribute(tensor, spec_attribute, ros2_type): # aka in numpy
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
        return tensor
    elif isinstance(spec_attribute, tfds.features.Text):
        return tensor
    elif isinstance(spec_attribute, tfds.features.Scalar):
        return cast_tensor_to_class_type(tensor, ros2_type)
    elif isinstance(spec_attribute, tfds.features.Tensor):
        # TODO: need to handle 2D array here
        
        return [float(x) for x in tensor.numpy()] #cast_tensor_to_class_type(tensor, ros2_type)
    else:
        raise NotImplementedError(f'feature type {type(spec_attribute)} for {spec_attribute} not implemented')

class DatasetFeatureSpec:
    def __init__(self, 
        observation_spec, 
        action_spec, 
        step_spec):
        '''
        example: 
        features=FeaturesDict({
        'steps': Dataset({
            # step_spec goes here
            'reward': float64,
            'timestamp': int64,
            'is_first': bool,
            'is_last': bool,
            'is_terminal': bool,
            'language_embedding': Tensor(shape=(5,), dtype=float32),
            'discount': float64,
            # observation spec goes here
            'observation': Tensor(shape=(17,), dtype=float64),
            # action spec goes here
            'action': Tensor(shape=(6,), dtype=float32),
            }),
        }),
        '''
        self.observation_spec = observation_spec
        self.action_spec = action_spec
        self.step_spec = step_spec
        # self.discount = tf.float64
        # self.is_first = tf.bool
        # self.is_last = tf.bool
        # self.is_terminal = tf.bool
        # self.reward = tf.float64

    def spec_to_ros2_message_definition(self, spec):
        '''
        example:
        ---
        string[] language_embedding
        float64 reward
        int64 timestamp
        bool is_first
        '''
        return '\n'.join([tf_feature_to_ros_msg_definition(k, v) for k, v in spec.items()])

    def to_dataset_config(self, dataset_name):
        return tfds.rlds.rlds_base.DatasetConfig(
            name=dataset_name,
            observation_info=self.observation_spec,
            action_info=self.action_spec,
            reward_info=self.step_spec['reward'],
            discount_info=self.step_spec['discount'],
            step_metadata_info={
                'is_first': self.step_spec['is_first'], 
                'is_last': self.step_spec['is_last'], 
                'is_terminal': self.step_spec['is_terminal'],
            }
        )


    def convert_ros2_msg_to_step_tuple(self, ros2_msg):
        observation = dict()
        action = dict()
        step = dict()

        for k, v in self.observation_spec.items():
            observation[k] = ros2_attribute_to_tf_feature(getattr(ros2_msg.observation, k), v)
        for k, v in self.action_spec.items():
            action[k] = ros2_attribute_to_tf_feature(getattr(ros2_msg.action, k), v)
        for k, v in self.step_spec.items():
            step[k] = ros2_attribute_to_tf_feature(getattr(ros2_msg, k), v)

        return observation, action, step

    def convert_tf_step_to_ros2_msg(self, step, action, observation):
        ros2_msg = Step()
        ros2_msg.observation = Observation()
        ros2_msg.action = Action()
        for k, v in self.step_spec.items():
            # TODO: need to handle missing info
            if k == "discount":
                continue 
            setattr(ros2_msg, k, tf_tensor_to_ros2_attribute(step[k], v, type(getattr(ros2_msg, k))))
        for k, v in self.observation_spec.items():
            setattr(ros2_msg.observation, k, tf_tensor_to_ros2_attribute(observation[k], v, type(getattr(ros2_msg.observation, k))))
        for k, v in self.action_spec.items():
            setattr(ros2_msg.action, k, tf_tensor_to_ros2_attribute(action[k], v, type(getattr(ros2_msg.action, k))))
        # for k, v in observation.items():
        #     # setattr(ros2_msg.observation, k, v)
        #     setattr(ros2_msg.observation, k, tf_tensor_to_ros2_attribute(v, getattr(ros2_msg.observation, k)))
        # for k, v in action.items():
        #     # setattr(ros2_msg.action, k, v)
        #     setattr(ros2_msg.action, k, tf_tensor_to_ros2_attribute(v, getattr(ros2_msg.action, k)))
        # for k, v in step.items():
        #     # setattr(ros2_msg, k, v)
        #     setattr(ros2_msg, k, tf_tensor_to_ros2_attribute(v, getattr(ros2_msg, k)))

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

