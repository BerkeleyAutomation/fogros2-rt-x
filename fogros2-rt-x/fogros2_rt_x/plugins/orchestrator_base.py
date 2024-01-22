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
class BaseTopicOrchestrator():
    def __init__(self,
                reward = 0.0,
                discount = 1.0
                ):
        self.logger = logging.getLogger(__name__)
        self.step_type = dm_env.StepType.FIRST
        self.observation = dict() 
        self.action = dict()
        self.step = dict()
        self.writer = None 

        self.reward = reward
        self.discount = discount

    def on_observation_topic(self, topic_name, data):
        self.observation[topic_name] = data
        
    def on_action_topic(self, topic_name, data):
        self.action[topic_name] = data
        self._new_step()

    def on_step_topic(self, topic_name, data):
        self.step[topic_name] = data

    def on_timestamp(self, timestamp, topic_name):
        pass

    def _new_step(self):

        timestep = dm_env.TimeStep(
                step_type=self.step_type,
                reward=self.step["reward"] if "reward" in self.step else self.reward,
                discount=self.step["discount"] if "discount" in self.step else self.discount,
                observation=self.observation,
            )
        stepdata = step_data.StepData(
                timestep=timestep, 
                action=self.action, 
                custom_data=None
            )

        if self.writer is None:
            self.logger.error("Writer is not set")
            return
        
        if self.step_type == dm_env.StepType.FIRST:
            self.writer.record_step(stepdata, is_new_episode=True)
            self.step_type = dm_env.StepType.MID
        else:
            self.writer.record_step(stepdata, is_new_episode=False)
    
    def _new_episode(self):
        self.step_type = dm_env.StepType.LAST
        self._new_step()

        # reset all the data
        self.step_type = dm_env.StepType.FIRST
        self.reward = 0.0
        self.discount = 0.0
        self.observation = dict()
        self.action = dict()
        self.step = dict()


    def set_writer (self, writer):
        self.writer = writer


