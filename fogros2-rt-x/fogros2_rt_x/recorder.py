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
from envlogger import step_data
import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer
from .dataset_spec import DatasetFeatureSpec

# code borrowed from https://github.com/rail-berkeley/oxe_envlogger/blob/main/oxe_envlogger/dm_env.py
import dm_env


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__("fogros2_rt_x_recorder")

        self.observation_spec = tfds.features.FeaturesDict({
                        # 'image': tfds.features.Image(shape=(480, 640, 3), dtype=tf.uint8),
                        'natural_language_embedding': tfds.features.Tensor(shape=(512,), dtype=tf.float32),
                        # 'natural_language_instruction': tf.string,
                        'state': tfds.features.Tensor(shape=(7,), dtype=tf.float32),
                    })
        self.action_spec = tfds.features.FeaturesDict({
                        'open_gripper': tfds.features.Scalar(dtype=tf.bool),
                        'rotation_delta':  tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                        # 'terminate_episode': tfds.features.Scalar(dtype=tf.float32),
                        'world_vector': tfds.features.Tensor(shape=(3,), dtype=tf.float32),
                    })

        self.feature_spec = DatasetFeatureSpec(
            observation_spec=self.observation_spec,
            action_spec=self.action_spec,
            step_spec={
                'reward': tfds.features.Scalar(dtype=tf.float64),
                'discount': tfds.features.Scalar(dtype=tf.float64),
                'is_first': tfds.features.Scalar(dtype=tf.bool),
                'is_last': tfds.features.Scalar(dtype=tf.bool),
                'is_terminal': tfds.features.Scalar(dtype=tf.bool),
            }
        )
        print(self.feature_spec.spec_to_ros2_message_definition(self.feature_spec.observation_spec))
        print(self.feature_spec.spec_to_ros2_message_definition(self.feature_spec.action_spec))
        print(self.feature_spec.spec_to_ros2_message_definition(self.feature_spec.step_spec))
        self.dataset_config = self.feature_spec.to_dataset_config('bridge')
        # tfds.rlds.rlds_base.DatasetConfig(
        #     name='bridge',
        #     observation_info=self.observation_spec,
        #     action_info=self.action_spec,
        #     reward_info=tf.float64,
        #     discount_info=tf.float64,
        #     step_metadata_info={'is_first': tf.bool, 'is_last': tf.bool, 'is_terminal': tf.bool})
        
        self.last_action = None 
        self.last_observation = None 
        self.last_step = None

        self.writer = tfds_backend_writer.TFDSBackendWriter(
                data_directory="/home/ubuntu/open-x-embodiment/playground_ds",
                split_name='train',
                max_episodes_per_file=1,
                ds_config=self.dataset_config)


        self.subscription = self.create_subscription(
            Step, "step_topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning


    def listener_callback(self, step_msg):
        self.get_logger().warning(
            f"Received step: {str(step_msg)[:100]}"
        )

        # self.last_observation, self.last_action, self.last_reward, discount, is_first, is_last, self.last_is_terminal = self.convert_ros2_msg_to_tf_feature(step_msg)
        # self.envlogger.step(
        #     self.last_action
        # )
        self.last_observation, self.last_action, self.last_step = self.feature_spec.convert_ros2_msg_to_step_tuple(step_msg)
        
        if self.last_step["is_last"]:
            step_type = dm_env.StepType.LAST
        elif self.last_step["is_first"]:
            step_type = dm_env.StepType.FIRST
        else:
            step_type = dm_env.StepType.MID
        
        timestep = dm_env.TimeStep(
            step_type=step_type,
            reward=self.last_step["reward"],
            discount=self.last_step["discount"],
            observation=self.last_observation,
        )

        data = step_data.StepData(timestep = timestep, 
                                  action = self.last_action, 
                                  custom_data = None)
        
        if self.last_step["is_first"]:
            self.writer.record_step(data, is_new_episode=True)
        else:
            self.writer.record_step(data, is_new_episode=False)
        


        # self.envlogger.log_step(
        #     observation=step_msg.observation,
        #     action=step_msg.action,
        #     reward=step_msg.reward,
        #     discount=step_msg.discount,
        #     step_metadata=step_msg.step_metadata,
        # )


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
