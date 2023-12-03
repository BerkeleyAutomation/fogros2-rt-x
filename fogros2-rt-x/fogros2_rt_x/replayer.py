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
from .dataset_spec import DatasetFeatureSpec
from .dataset_conf import *

class DatasetReplayer(Node):
    def __init__(self, dataset_name):
        super().__init__("fogros2_rt_x_replayer")

        self.publisher = self.create_publisher(Step, "step_topic", 10)
        self.dataset = load_rlds_dataset(dataset_name)
        self.logger = self.get_logger()
        self.logger.info("Loading Dataset " + str(get_dataset_info(["bridge"])))

        timer_period = 10  # seconds
        self.create_timer(timer_period, self.timer_callback)

        self.feature_spec = DatasetFeatureSpec(
            observation_spec=OBSERVATION_SPEC,
            action_spec=ACTION_SPEC,
            step_spec=STEP_SPEC
        )

        self.episode = next(iter(self.dataset))

    def timer_callback(self):
        for step in self.episode['steps']:
            # msg = tf_step_to_ros2_msg(next(iter(self.episode['steps'])))
            msg = self.feature_spec.convert_tf_step_to_ros2_msg(
                step, 
                step["action"], 
                step["observation"])
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DatasetReplayer(dataset_name = "bridge")

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
