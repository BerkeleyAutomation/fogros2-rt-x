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

import dm_env
from envlogger import step_data
import logging


class BaseTopicOrchestrator:
    """
    Base class for topic orchestrators.

    This class provides a basic implementation for handling observation, action, and step topics.
    It also includes methods for resetting the orchestrator and recording new steps and episodes.

    Attributes:
        logger: The logger object for logging messages.
        writer: The writer object for recording steps and episodes.
        reward: The default reward value for steps.
        discount: The default discount value for steps.
        default_observation: The default observation dictionary.
        default_action: The default action dictionary.
        default_step: The default step dictionary.
        step_type: The current step type.
        observation: The current observation dictionary.
        action: The current action dictionary.
        step: The current step dictionary.
    """

    def __init__(self, reward=0.0, discount=1.0):
        self.logger = logging.getLogger(__name__)
        self.writer = None
        self.reward = reward
        self.discount = discount

        # default values
        self.default_observation = dict()
        self.default_action = dict()
        self.default_step = dict()

        self.step_type = dm_env.StepType.FIRST
        self.observation = dict()
        self.action = dict()
        self.step = dict()

    def on_observation_topic(self, topic_name, data):
        """
        Callback method for handling observation topics.

        Args:
            topic_name: The name of the observation topic.
            data: The data received from the observation topic.
        """
        self.observation[topic_name] = data

    def on_action_topic(self, topic_name, data):
        """
        Callback method for handling action topics.

        Args:
            topic_name: The name of the action topic.
            data: The data received from the action topic.
        """
        self.action[topic_name] = data

    def on_step_topic(self, topic_name, data):
        """
        Callback method for handling step topics.

        Args:
            topic_name: The name of the step topic.
            data: The data received from the step topic.
        """
        self.step[topic_name] = data

    def on_timestamp(self, timestamp, topic_name):
        """
        Callback method for handling timestamps.

        Args:
            timestamp: The timestamp value.
            topic_name: The name of the topic associated with the timestamp.
        """
        pass

    def _reset(self):
        """
        Resets the orchestrator to its default state.
        """
        self.step_type = dm_env.StepType.FIRST
        self.observation = self.default_observation
        self.action = self.default_action
        self.step = self.default_step

    def _new_step(self):
        """
        Records a new step using the current state and action.
        """
        timestep = dm_env.TimeStep(
            step_type=self.step_type,
            reward=self.step["reward"] if "reward" in self.step else self.reward,
            discount=self.step["discount"]
            if "discount" in self.step
            else self.discount,
            observation=self.observation,
        )
        stepdata = step_data.StepData(
            timestep=timestep, action=self.action, custom_data=None
        )

        if self.writer is None:
            self.logger.error("Writer is not set")
            return

        if self.step_type == dm_env.StepType.FIRST:
            self.logger.info("New episode")
            self.writer.record_step(stepdata, is_new_episode=True)
            self.step_type = dm_env.StepType.MID
        else:
            self.writer.record_step(stepdata, is_new_episode=False)

    def _new_episode(self):
        """
        Starts a new episode.
        """
        # TODO: figure out the last one
        # self.step_type = dm_env.StepType.LAST
        # self._new_step()
        self._reset()

    def set_writer(self, writer):
        """
        Sets the writer object for recording steps and episodes.

        Args:
            writer: The writer object.
        """
        self.writer = writer

    def set_default_values(self, observation, action, step):
        """
        Sets the default values for observation, action, and step.

        Args:
            observation: The default observation dictionary.
            action: The default action dictionary.
            step: The default step dictionary.
        """
        self.observation = observation
        self.action = action
        self.step = step

        self.default_observation = observation
        self.default_action = action
        self.default_step = step

