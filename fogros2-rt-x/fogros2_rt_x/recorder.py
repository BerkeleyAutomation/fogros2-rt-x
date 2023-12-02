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
from rclpy.node import Node
from .dataset_utils import *
from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger

import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer


# code borrowed from https://github.com/rail-berkeley/oxe_envlogger/blob/main/oxe_envlogger/dm_env.py
import dm_env
from dm_env import specs
class DummyDmEnv():
    """
    This is a dummy class to use so that the dm_env.Environment interface
    can still receive the observation_space and action_space. So that the
    logging can be done in the dm_env.Environment interface.

    https://github.com/google-deepmind/dm_env
    """

    def __init__(self,
                 observation_space,
                 action_space,
                 step_callback,
                 reset_callback,
                 ):
        """
        :param observation_space: gym observation space
        :param action_space: gym action space
        :param step_callback: callback function to call gymenv.step
        :param reset_callback: callback function to call gymenv.reset
        """
        self.step_callback = step_callback
        self.reset_callback = reset_callback
        self.observation_space = observation_space
        self.action_space = action_space

    def step(self, action) -> dm_env.TimeStep:
        # Note that dm_env.step doesn't accept additional arguments
        val = self.step_callback(action)
        obs, reward, terminate, truncate, info = val
        reward = float(reward)
        if terminate:
            ts = dm_env.termination(reward=reward, observation=obs)
        elif truncate:
            ts = dm_env.truncation(reward=reward, observation=obs)
        else:
            ts = dm_env.transition(reward=reward, observation=obs)
        return ts

    def reset(self) -> dm_env.TimeStep:
        # Note that dm_env.reset doesn't accept additional arguments
        obs, _ = self.reset_callback()
        ts = dm_env.restart(obs)
        return ts

    # def cast_tf_datatype_to_numpy(self, tf_datatype):
    #     # TODO: placeholder
    #     if tf_datatype == tf.float32:
    #         return np.float32
    #     elif tf_datatype == tf.float64:
    #         return np.float64
    #     elif tf_datatype == tf.string:
    #         return np.str
    #     else:
    #         raise NotImplementedError
    
    def cast_tf_datatype_to_numpy(self, tf_datatype):
        return tf_datatype.as_numpy_dtype
    
    def from_tf_feature_to_spec(self, feature) -> specs:
        print(feature)
        spec = {
            key: specs.Array(
                dtype=self.cast_tf_datatype_to_numpy(space.dtype),
                shape=space.shape,
                name=key,
            )
            for key, space in feature.items() if space.dtype != tf.string
        }

        return spec

    def observation_spec(self):
        return self.from_tf_feature_to_spec({
                        # 'image': tfds.features.Image(shape=(480, 640, 3), dtype=tf.uint8),
                        'natural_language_embedding': tfds.features.Tensor(shape=(512,), dtype=tf.float32),
                        # 'natural_language_instruction': tf.string,
                        'state': tfds.features.Tensor(shape=(7,), dtype=tf.float32),
                    })

    def action_spec(self):
        return self.from_tf_feature_to_spec({
                        # 'open_gripper': tf.bool,
                        'rotation_delta':  tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                        # 'terminate_episode': tf.float32,
                        'world_vector': tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                    })

    def reward_spec(self):
        return specs.Array(
            shape=(),
            dtype=np.float64,
            name='reward',
        )

    def discount_spec(self):
        return specs.Array(
            shape=(),
            dtype=np.float64,
            name='discount',
        )


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__("fogros2_rt_x_recorder")

        dataset_config = tfds.rlds.rlds_base.DatasetConfig(
            name='bridge',
            observation_info=tfds.features.FeaturesDict({
                        # 'image': tfds.features.Image(shape=(480, 640, 3), dtype=tf.uint8),
                        'natural_language_embedding': tfds.features.Tensor(shape=(512,), dtype=tf.float32),
                        # 'natural_language_instruction': tf.string,
                        'state': tfds.features.Tensor(shape=(7,), dtype=tf.float32),
                    }),
            action_info=tfds.features.FeaturesDict({
                        # 'open_gripper': tf.bool,
                        'rotation_delta':  tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                        # 'terminate_episode': tf.float32,
                        'world_vector': tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                    }),
            reward_info=tf.float32,
            discount_info=tf.float32,
            step_metadata_info={'is_first': tf.bool, 'is_last': tf.bool, 'is_terminal': tf.bool})

        env = DummyDmEnv(
            observation_space=None,
            action_space=None,
            step_callback=None,
            reset_callback=None,
        )
        self.envlogger = envlogger.EnvLogger(
            env,
            backend = tfds_backend_writer.TFDSBackendWriter(
                data_directory="/home/ubuntu/open-x-embodiment/playground_ds",
                split_name='train',
                max_episodes_per_file=500,
                ds_config=dataset_config),
        )
        self.subscription = self.create_subscription(
            Step, "step_topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, step_msg):
        self.get_logger().warning(
            f"Received step: {step_msg}"
        )


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
