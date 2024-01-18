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
from fogros2_rt_x.dataset_spec import DatasetFeatureSpec, FeatureSpec


class DatasetConfig:
    """
    A class for storing the configuration of a dataset.

    Args:
        dataset_name (str): The name of the dataset.
        save_path (str): The path to save the dataset.
        big_query_project (str): The big query project to store the dataset metadata.
        observation_spec (list): The list of observation feature specifications.
        action_spec (list): The list of action feature specifications.
        step_spec (list): The list of step feature specifications.
    """

    def __init__(
        self,
        dataset_name,
        save_path,
        big_query_project,
        observation_spec,
        action_spec,
        step_spec,
    ):
        self.dataset_name = dataset_name
        self.save_path = save_path
        self.big_query_project = big_query_project
        self.observation_spec = observation_spec
        self.action_spec = action_spec
        self.step_spec = step_spec

    def get_dataset_feature_spec(self):
        """
        Get the dataset feature specification.

        Returns:
            DatasetFeatureSpec: The dataset feature specification.
        """
        return DatasetFeatureSpec(
            observation_spec=self.observation_spec,
            action_spec=self.action_spec,
            step_spec=self.step_spec,
        )

    def get_dataset_builder(self):
        """
        Get the dataset builder.

        Returns:
            tfds.core.DatasetBuilder: The dataset builder.
        """
        return tfds.core.DatasetBuilder(
            name=self.dataset_name,
            data_dir=self.save_path,
            version=tfds.core.Version("0.0.1"),
            release_notes={
                "0.0.1": "Initial release.",
            },
        )


# example #2: berkeley_fanuc_manipulation
# OBSERVATION_SPEC = [
#     FeatureSpec("wrist_image", Image(shape=(224, 224, 3), dtype=tf.uint8)),
#     FeatureSpec("image", Image(shape=(224, 224, 3), dtype=tf.uint8)),
#     FeatureSpec("end_effector_state", Tensor(shape=(7,), dtype=tf.float32)),
#     FeatureSpec("state", Tensor(shape=(13,), dtype=tf.float32)),
# ]

# ACTION_SPEC = [
#     FeatureSpec("action", Tensor(shape=(6,), dtype=tf.float32)),
# ]

# STEP_SPEC = [
#     FeatureSpec("reward", Scalar(dtype=tf.float64)),
#     FeatureSpec("discount", Scalar(dtype=tf.float64), default_value=0.0),
#     FeatureSpec("is_first", Scalar(dtype=tf.bool)),
#     FeatureSpec("is_last", Scalar(dtype=tf.bool)),
#     FeatureSpec("is_terminal", Scalar(dtype=tf.bool)),
#     FeatureSpec("language_embedding", Tensor(shape=(512,), dtype=tf.float32)),
#     FeatureSpec("language_instruction", Text()),
# ]