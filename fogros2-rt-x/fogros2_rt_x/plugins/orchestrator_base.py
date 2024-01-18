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

import rclpy
from rclpy.node import Node

from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger
from envlogger import step_data
import tensorflow_datasets as tfds
from envlogger.backends import tfds_backend_writer
import functools

class BaseTopicOrchestrator(Node):
    def __init__(self, config):
        super().__init__("fogros2_rt_x_orchestrator")
        self.logger = self.get_logger()

        self.config = config
        self.feature_spec = self.config.get_dataset_feature_spec()

        self._init_observation_topics()
        self._init_action_topics()
        self._init_step_information_topics()

        self.observation_msg = Observation()
        self.action_msg = Action()
        self.step_msg = Step()

        self.publisher = self.create_publisher(Step, "step_info", 10)

    def _init_observation_topics(self):
        def create_dynamic_observation_callback(topic_name):
            def observation_callback(self, msg):
                self.logger.info(f"Received observation message on {topic_name}")
                setattr(self.observation_msg, topic_name, msg)

            return functools.partial(observation_callback, self)
        # create subscriptions for all observation topics
        for observation in self.feature_spec.observation_spec:
            callback = create_dynamic_observation_callback(
                observation.ros_topic_name
            )
            self.create_subscription(
                observation.ros_type,
                observation.ros_topic_name,
                callback,
                10,
            )

    def _init_action_topics(self):
        def create_dynamic_action_callback( topic_name):
            def action_callback(self, msg):
                self.logger.info(f"Received action message on {topic_name}")
                setattr(self.action_msg, topic_name, msg)
            return functools.partial(action_callback, self)
        # create publishers for all action topics
        for action in self.feature_spec.action_spec:
            callback = create_dynamic_action_callback(action.ros_topic_name)
            self.create_subscription(
                action.ros_type, action.ros_topic_name, callback, 10
            )

    def _init_step_information_topics(self):
        def create_dynamic_step_callback( topic_name):
            def step_callback(self, msg):
                self.logger.info(f"Received step message on {topic_name}")
                setattr(self.step_msg, topic_name, msg)
            return functools.partial(step_callback, self)

        # create subscriptions for all observation topics
        for step_info in self.feature_spec.step_spec:
            callback = create_dynamic_step_callback(
                step_info.ros_topic_name
            )
            self.create_subscription(
                step_info.ros_type,
                step_info.ros_topic_name,
                callback,
                10,
            )


class PerPeriodTopicOrchestrator(BaseTopicOrchestrator):
    def __init__(self, config):
        super().__init__(config)

        self.declare_parameter("per_period_interval", 0.2)  # second
        self.per_period_interval = self.get_parameter("per_period_interval").value
        
        self.create_timer(self.per_period_interval, self.timer_callback)

    def timer_callback(self):
        self.logger.info("Publishing step message")
        self.publisher.publish(self.step_msg)


