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


class BaseDatasetConfig:
    """
    A class for storing the configuration of a dataset.

    Args:
        dataset_name (str): The name of the dataset.
        save_path (str): The path to save the dataset.
        observation_spec (list): The list of observation feature specifications.
        action_spec (list): The list of action feature specifications.
        step_spec (list): The list of step feature specifications.
    """

    def __init__(
        self,
        dataset_name,
        save_path,
        observation_spec,
        action_spec,
        step_spec,
    ):
        self.dataset_name = dataset_name
        self.save_path = save_path
        self.observation_spec = observation_spec
        self.action_spec = action_spec
        self.step_spec = step_spec

        self.observation_tf_dict = self._spec_to_tf_type_dict(observation_spec)
        self.action_tf_dict = self._spec_to_tf_type_dict(action_spec)
        self.step_tf_dict = self._spec_to_tf_type_dict(step_spec)

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

    def get_rlds_dataset_config(self):
        """
        Get the dataset builder.

        Returns:
            tfds.core.DatasetBuilder: The dataset builder.
        """
        return tfds.rlds.rlds_base.DatasetConfig(
            name=self.dataset_name,
            version=tfds.core.Version("0.0.1"),
            release_notes={
                "0.0.1": "Initial release.",
            },
            observation_info=self.observation_tf_dict,
            action_info=self.action_tf_dict,
            reward_info=self.step_tf_dict["reward"],
            discount_info=self.step_tf_dict["discount"],
            # step_metadata_info={
            #     "is_first": self.step_tf_dict["is_first"],
            #     "is_last": self.step_tf_dict["is_last"],
            #     "is_terminal": self.step_tf_dict["is_terminal"],
            # },
        )

    def _spec_to_tf_type_dict(self, spec):
        """
        Converts the feature specification to a dictionary.

        Args:
            spec (list): The feature specification.

        Returns:
            dict: The feature specification in dictionary format.
        """
        return {f.tf_name: f.tf_type for f in spec}
