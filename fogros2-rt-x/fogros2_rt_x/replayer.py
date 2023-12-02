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

def cast_tensor_to_class_type(tensor, class_type):
    return class_type(tensor.numpy())

def cast_value_to_class_type(tensor, class_type):
    return class_type(tensor)

def tf_step_to_ros2_msg(step):
    ros2_step = Step()
    ros2_step.observation = Observation()
    ros2_step.action = Action()
    ros2_step.reward = cast_tensor_to_class_type(step['reward'], type(ros2_step.reward))
    ros2_step.is_first = cast_tensor_to_class_type(step['is_first'], type(ros2_step.is_first)) 
    ros2_step.is_last = cast_tensor_to_class_type(step['is_last'], type(ros2_step.is_last)) 
    ros2_step.is_terminal = cast_tensor_to_class_type(step['is_terminal'], type(ros2_step.is_terminal)) 
    for k, v in step['observation'].items():
        if k == 'image':
            continue
        field_type = type(getattr(ros2_step.observation, k))
        print(field_type, k, str(field_type))
        if str(field_type) == "<class 'array.array'>":
            # TODO: assume all float32
            setattr(ros2_step.observation, k, [float(x) for x in v.numpy()])
        elif str(field_type) == "<class 'str'>":
            setattr(ros2_step.observation, k, str(v.numpy()))
        else:
            setattr(ros2_step.observation, k, cast_value_to_class_type(v, field_type))
        # print(type(getattr(ros2_step.observation, k)), v.numpy(), type(v.numpy()[0]))
        # setattr(ros2_step.observation, k, [float(x) for x in v.numpy()])
        # setattr(ros2_step.observation, k, cast_value_to_class_type(v, getattr(ros2_step.observation, k)))
    for k, v in step['action'].items():
        field_type = type(getattr(ros2_step.action, k))
        print(field_type, k, str(field_type))
        if str(field_type) == "<class 'array.array'>":
            # TODO: assume all float32
            setattr(ros2_step.action, k, [float(x) for x in v.numpy()])
        elif str(field_type) == "<class 'str'>":
            setattr(ros2_step.action, k, str(v.numpy()))
        else:
            setattr(ros2_step.action, k, cast_value_to_class_type(v, field_type))
    return ros2_step


class DatasetReplayer(Node):
    def __init__(self, dataset_name):
        super().__init__("fogros2_rt_x_replayer")

        self.publisher = self.create_publisher(Step, "step_topic", 10)
        self.dataset = load_rlds_dataset(dataset_name)
        self.logger = self.get_logger()
        self.logger.info("Loading Dataset " + str(get_dataset_info(["bridge"])))

        timer_period = 2  # seconds
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        episode = next(iter(self.dataset))
        msg = tf_step_to_ros2_msg(next(iter(episode['steps'])))
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
