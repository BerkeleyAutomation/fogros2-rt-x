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

from .exporter import DatasetExporter
from ros2cli.verb import VerbExtension

from ros2cli.command import CommandExtension, add_subparsers_on_demand
import os
from .dataset_spec import DatasetFeatureSpec
from .plugins.conf_base import *
from .dataset_utils import *
from .dataset_manager import DatasetManager


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


class ExportVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--dataset_name",
            nargs="*",
            help="Name of the dataset, error when not specified, need to match the name in ./fogros2_rt_x/plugins",
        )
        parser.add_argument(
            "--metadata_db_location",
            nargs="*",
            help="Location of the metadata database, by default it stores at the same location as the dataset_dir",
        )
        parser.add_argument(
            "--destination",
            nargs="*",
            help="directory of the exported dataset, can be in gs:// for google cloud storage",
        )


    def main(self, *, args):
        if args.dataset_name is None:
            raise ValueError("dataset_name must be specified")
        else:
            self.dataset_name = args.dataset_name[0]
            (
                observation_topics,
                action_topics,
                step_topics,
                orchestrator,
            ) = get_dataset_config_from_str(self.dataset_name)

        if args.destination is None:
            raise ValueError("destination must be specified")
        else:
            self.destination = args.destination[0]

        print("Exporting dataset: " + self.dataset_name)
        DatasetManager(
            sql_db_location="./metadata.db",
            dataset_name=self.dataset_name,
        ).export_as_rlds(
            observation_topics=observation_topics,
            action_topics=action_topics,
            step_topics=step_topics,
            orchestrator=orchestrator,
            destination=self.destination,
        )
        print("Exporting dataset complete")
        return 0


class LoadVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--dataset_dir",
            nargs="*",
            help="Directory of the dataset, error when not specified",
        )
        parser.add_argument(
            "--dataset_name",
            nargs="*",
            help="Name of the dataset, error when not specified",
        )
        parser.add_argument(
            "--metadata_db_location",
            nargs="*",
            help="Location of the metadata database, by default it stores at the current directory",
        )

    def main(self, *, args):
        if args.dataset_dir is None:
            raise ValueError("dataset_dir must be specified")
        else:
            self.dataset_dir = args.dataset_dir[0]

        if args.metadata_db_location is None:
            self.metadata_db_location = "./metadata.db"
        else:
            self.metadata_db_location = args.metadata_db_location[0]

        if args.dataset_name is None:
            raise ValueError("dataset_name must be specified")
        else:
            self.dataset_name = args.dataset_name[0]

        print("Loading dataset: " + self.dataset_dir)
        print("Storing metadata in : " + self.metadata_db_location)

        DatasetManager(
            sql_db_location=self.metadata_db_location,
            dataset_name=self.dataset_name,
        ).load(dataset_directory=self.dataset_dir)
        print("Loading succeeds")
        return 0
