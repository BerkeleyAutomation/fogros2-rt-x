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
from .exporter import DatasetExporter

import rclpy
from rclpy.node import Node
from tensorflow_datasets.core.features import Tensor, Image, FeaturesDict, Scalar, Text

# from .dataset_utils import *
# from fogros2_rt_x_msgs.msg import Step, Observation, Action
import envlogger

# import tensorflow_datasets as tfds
# from envlogger.backends import tfds_backend_writer
# from .dataset_spec import DatasetFeatureSpec
# from .plugins.conf_base import *


from .database_connector import SqliteConnector
from std_srvs.srv import Empty
import random
import pickle 
from .dataset_manager import DatasetManager


def main(args=None):
    rclpy.init(args=args)

    dataset_manager = DatasetManager(
        dataset_directory = "./datasets",
        observation_topics = ["/wrist_image", "/image", "/end_effector_state", "/state"],
        action_topics = ["/action"],
        step_topics = ["/language_embedding", "/language_instruction", "/discount", "/reward"],
    )

    # rclpy.spin(dataset_recorder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # dataset_recorder.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
