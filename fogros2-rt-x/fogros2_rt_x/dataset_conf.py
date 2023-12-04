
import tensorflow_datasets as tfds
import tensorflow as tf

DATASET_NAME = "bridge"
SAVE_PATH = "/home/ubuntu/open-x-embodiment/playground_ds"
# "gs://test-fogros-rtx-example"

OBSERVATION_SPEC = tfds.features.FeaturesDict({
                'image': tfds.features.Image(shape=(480, 640, 3), dtype=tf.uint8),
                'natural_language_embedding': tfds.features.Tensor(shape=(512,), dtype=tf.float32),
                'natural_language_instruction': tfds.features.Tensor(shape=(40,), dtype=tf.string),
                'state': tfds.features.Tensor(shape=(7,), dtype=tf.float32),
            })

ACTION_SPEC = tfds.features.FeaturesDict({
                'open_gripper': tfds.features.Scalar(dtype=tf.bool),
                'rotation_delta':  tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                'terminate_episode': tfds.features.Scalar(dtype=tf.float64),
                'world_vector': tfds.features.Tensor(shape=(3,), dtype=tf.float32),
            })

STEP_SPEC = {
                'reward': tfds.features.Scalar(dtype=tf.float64),
                'discount': tfds.features.Scalar(dtype=tf.float64),
                'is_first': tfds.features.Scalar(dtype=tf.bool),
                'is_last': tfds.features.Scalar(dtype=tf.bool),
                'is_terminal': tfds.features.Scalar(dtype=tf.bool),
            }