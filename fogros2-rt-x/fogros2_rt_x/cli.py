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

from ros2cli.verb import VerbExtension

from ros2cli.command import CommandExtension, add_subparsers_on_demand
import os 
from .dataset_spec import DatasetFeatureSpec
from .dataset_conf import *


class FogCommand(CommandExtension):
    """Base 'fog' command ROS 2 CLI extension."""

    def add_arguments(self, parser, cli_name):
        """Add verb parsers."""
        self._subparser = parser
        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(
            parser, cli_name, "_verb", "fogros2_rt_x.verb", required=False
        )

    def main(self, *, parser, args):
        """
        Handle fog command.

        Take in args from CLI, pass to verbs if specified,
        otherwise print help.
        """
        # in case no verb was passed
        if not hasattr(args, "_verb"):
            self._subparser.print_help()
            return 0

        # call the verb's main method
        extension = getattr(args, "_verb")
        return extension.main(args=args)


action_and_observation_str = '''
Action action 
Observation observation'''

class ConfigVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--msg_path",
            nargs="*",
            help="Path to ROS2 message repo, default is to use the colcon workspace path",
        )

    def generate_ros_config(self):
        self.observation_spec = OBSERVATION_SPEC
        self.action_spec = ACTION_SPEC
        self.step_spec = STEP_SPEC
        self.feature_spec = DatasetFeatureSpec(
            observation_spec=self.observation_spec,
            action_spec=self.action_spec,
            step_spec=self.step_spec,
        )

        # observation spec 
        print("=== observation spec ===")
        print(self.feature_spec.feature_spec_list_to_ros2_msg_definition(self.observation_spec))
        with open(self.msg_path + "/Observation.msg", "w") as f:
            f.write(self.feature_spec.feature_spec_list_to_ros2_msg_definition(self.observation_spec))
        print("=== action spec ===")
        print(self.feature_spec.feature_spec_list_to_ros2_msg_definition(self.action_spec))
        with open(self.msg_path + "/Action.msg", "w") as f:
            f.write(self.feature_spec.feature_spec_list_to_ros2_msg_definition(self.action_spec))
        print("=== step spec ===")
        print(self.feature_spec.feature_spec_list_to_ros2_msg_definition(self.step_spec))
        
        with open(self.msg_path + "/Step.msg", "w") as f:
            f.write(self.feature_spec.feature_spec_list_to_ros2_msg_definition(self.step_spec))
            f.write(action_and_observation_str)


    def main(self, *, args):
        """Handle config verb."""
        if args.msg_path is None:
            # get colcon workspace path
            ws_path = os.environ["COLCON_PREFIX_PATH"]
            msg_repo_path = ws_path + "/../fogros2-rt-x/dtype_msgs"
            msg_repo_path_src = ws_path + "/../src/fogros2-rt-x/dtype_msgs"
            # get path to fogros2-rt-x message repo
            if os.path.exists(msg_repo_path):
                self.msg_path = msg_repo_path
            elif os.path.exists(msg_repo_path_src):
                self.msg_path = msg_repo_path_src
            else:
                raise ValueError("default msg_path not found, checked " + msg_repo_path + " and " + msg_repo_path_src + " please specify --msg_path")
        else:
            if args.msg_path[0] != "/":
                raise ValueError("msg_path must be an absolute path")
            self.msg_path = args.msg_path

        # append msg to path for the actual messages 
        self.msg_path = self.msg_path + "/msg/"
        
        print("Writing Configuration to ROS2 message directory path: " + self.msg_path)

        self.generate_ros_config()
        return 0
