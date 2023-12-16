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


import tensorflow_datasets as tfds
import tensorflow as tf
from tensorflow_datasets.core.features import Tensor, Image, FeaturesDict, Scalar, Text
from .dataset_spec import DatasetFeatureSpec, FeatureSpec

# the name of the dataset
DATASET_NAME = "bridge"
# TODO: other information such as citation

# the path can be a local directory or a google cloud storage bucket
SAVE_PATH = "/home/ubuntu/open-x-embodiment/playground_ds"
# "/home/ubuntu/open-x-embodiment/playground_ds"
# "gs://test-fogros-rtx-example"

# this is used for bigquery project metadata storage
# the project should be accessible through google cloud account
BIG_QUERY_PROJECT = "fogros2-rt-x"

# currently we support 
# tfds.features.Image 
# tfds.features.Tensor (1D only)
# tfds.features.Text (for variable length of string)
# tfds.features.Scalar (for any tf.dtype)
# more types upon request 



# example #1 bridge dataset
# OBSERVATION_SPEC = [
#     FeatureSpec("image", Image(shape=(480, 640, 3), dtype=tf.uint8)),
#     FeatureSpec("natural_language_embedding", Tensor(shape=(512,), dtype=tf.float32)),
#     FeatureSpec("natural_language_instruction", Text()),
#     FeatureSpec("state", Tensor(shape=(7,), dtype=tf.float32)),
# ]

# ACTION_SPEC = [
#     FeatureSpec("open_gripper", Scalar(dtype=tf.bool)),
#     FeatureSpec("rotation_delta", Tensor(shape=(3,), dtype=tf.float32), is_triggering_topic=True),
#     FeatureSpec("terminate_episode", Scalar(dtype=tf.float64)),
#     FeatureSpec("world_vector", Tensor(shape=(3,), dtype=tf.float32)),
# ]


# STEP_SPEC = [
#     FeatureSpec("reward", Scalar(dtype=tf.float64)),
#     FeatureSpec("discount", Scalar(dtype=tf.float64), default_value=0.0),
#     FeatureSpec("is_first", Scalar(dtype=tf.bool)),
#     FeatureSpec("is_last", Scalar(dtype=tf.bool)),
#     FeatureSpec("is_terminal", Scalar(dtype=tf.bool)),
# ]


# example #2: berkeley_fanuc_manipulation
OBSERVATION_SPEC = [
    FeatureSpec("wrist_image", Image(shape=(224, 224, 3), dtype=tf.uint8)),
    FeatureSpec("image", Image(shape=(224, 224, 3), dtype=tf.uint8)),
    FeatureSpec("end_effector_state", Tensor(shape=(7,), dtype=tf.float32)),
    FeatureSpec("state", Tensor(shape=(13,), dtype=tf.float32)),
]

ACTION_SPEC = [
    FeatureSpec("action", Tensor(shape=(6,), dtype=tf.float32)),
]

STEP_SPEC = [
    FeatureSpec("reward", Scalar(dtype=tf.float64)),
    FeatureSpec("discount", Scalar(dtype=tf.float64), default_value=0.0),
    FeatureSpec("is_first", Scalar(dtype=tf.bool)),
    FeatureSpec("is_last", Scalar(dtype=tf.bool)),
    FeatureSpec("is_terminal", Scalar(dtype=tf.bool)),
    FeatureSpec("language_embedding", Tensor(shape=(512,), dtype=tf.float32)),
    FeatureSpec("language_instruction", Text()),
]