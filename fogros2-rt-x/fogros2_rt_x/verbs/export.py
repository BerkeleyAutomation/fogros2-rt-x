
from ros2cli.verb import VerbExtension
from fogros2_rt_x.dataset_manager import DatasetManager
from fogros2_rt_x.dataset_utils import get_dataset_config_from_str

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
