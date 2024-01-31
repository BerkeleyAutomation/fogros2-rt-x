
from ros2cli.verb import VerbExtension
from fogros2_rt_x.dataset_manager import DatasetManager

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
