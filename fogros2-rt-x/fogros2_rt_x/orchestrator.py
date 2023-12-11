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
from .dataset_conf import *
import functools
import time 

class StreamOrchestrator(Node):
    def __init__(self):
        super().__init__("fogros2_rt_x_orchestrator")
        self.logger = self.get_logger()

        self.declare_parameter("alignment_wait_time", 0.2)  # second
        self.alignment_wait_time = self.get_parameter("alignment_wait_time").value

        self.observation_spec = OBSERVATION_SPEC
        self.action_spec = ACTION_SPEC
        self.step_spec = STEP_SPEC

        self.feature_spec = DatasetFeatureSpec(
            observation_spec=self.observation_spec,
            action_spec=self.action_spec,
            step_spec=self.step_spec,
        )

        # check if the triggering topic is in the action spec
        # this helps to determine the topic that triggers a new step message
        self.feature_spec.check_triggering_topic()

        self.triggering_topic = None
        self._init_observation_topics()
        self._init_action_topics()

        self.observation_msg = Observation()
        self.action_msg = Action()
        self.step_msg = Step()

        self.publisher = self.create_publisher(Step, "step_info", 10)

    def _init_observation_topics(self):
        # create subscriptions for all observation topics
        for observation in self.observation_spec:
            callback = self.create_dynamic_observation_callback(
                observation.ros_topic_name
            )
            self.create_subscription(
                observation.ros_type,
                observation.ros_topic_name,
                callback,
                10,
            )

    def _init_action_topics(self):
        # create publishers for all action topics
        for action in self.action_spec:
            callback = self.create_dynamic_action_callback(action.ros_topic_name)
            self.create_subscription(
                action.ros_type, action.ros_topic_name, callback, 10
            )
            if action.is_triggering_topic:
                self.triggering_topic = action.ros_topic_name
        if self.triggering_topic is None:
            raise RuntimeError(
                "No triggering topic found in action spec, need to choose one action topic as triggering topic"
            )

    def _init_step_information_topics(self):
        for step in self.step_spec:
            callback = self.create_dynamic_step_callback(step.ros_topic_name)
            self.create_subscription(step.ros_type, step.ros_topic_name, callback, 10)

    def create_dynamic_action_callback(self, topic_name):
        def action_callback(self, msg):
            # self.logger.info(f"Received action message on {topic_name}")
            setattr(self.action_msg, topic_name, msg)

            if topic_name == self.triggering_topic:
                self.step_msg.action = self.action_msg
                self.step_msg.observation = self.observation_msg
                self.send_step_msg_with_alignment_wait_time()

        return functools.partial(action_callback, self)

    def create_dynamic_observation_callback(self, topic_name):
        def observation_callback(self, msg):
            # self.logger.info(f"Received observation message on {topic_name}")
            setattr(self.observation_msg, topic_name, msg)

        return functools.partial(observation_callback, self)

    def create_dynamic_step_callback(self, topic_name):
        def step_callback(self, msg):
            # self.logger.info(f"Received step message on {topic_name}")
            setattr(self.step_msg, topic_name, msg)

        return functools.partial(step_callback, self)

    # def send_step_msg(self):
    #     self.publisher.publish(self.step_msg)
    #     self.timer.cancel()
    #     self.timer.destroy()

    # TODO: sleep alignment here is wrong because it is not aligned with the step message
    def send_step_msg_with_alignment_wait_time(self):
        # create a ros2 timer to send step message with alignment wait time
        # self.timer = self.create_timer(self.alignment_wait_time, self.send_step_msg)
        time.sleep(self.alignment_wait_time)
        self.publisher.publish(self.step_msg)


def main(args=None):
    rclpy.init(args=args)

    stream_orchestrator = StreamOrchestrator()

    rclpy.spin(stream_orchestrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stream_orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
