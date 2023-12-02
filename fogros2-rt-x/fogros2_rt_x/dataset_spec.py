
import tensorflow as tf
import tensorflow_datasets as tfds

def tf_feature_to_ros_msg_definition(name, feature):
    
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