from ros2cli.verb import VerbExtension
from fogros2_rt_x.dataset_manager import DatasetManager
from fogros2_rt_x.dataset_utils import get_dataset_config_from_str


class ExportVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "--dataset_name",
            help="Name of the dataset, error when not specified, need to match the name in metadata.db",
        )

        parser.add_argument(
            "-o" "--topic_observation",
            nargs="+",
            help="topic name of the observation, error when not specified",
        )

        parser.add_argument(
            "-a" "--topic_action",
            nargs="+",
            help="topic name of the action, error when not specified",
        )

        parser.add_argument(
            "-s" "--topic_step",
            nargs="+",
            help="topic name of the step, error when not specified",
        )

        parser.add_argument(
            "--orchstrator",
            nargs="?",
            help="type of orchestrator, needs to match the name in ./fogros2_rt_x/plugins",
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
            self.dataset_name = args.dataset_name

        print("observation_topics: {}".format(args.o__topic_observation))
        print("action_topics: {}".format(args.a__topic_action))
        print("step_topics: {}".format(args.s__topic_step))

        # (
        #     observation_topics,
        #     action_topics,
        #     step_topics,
        #     orchestrator,
        # ) = get_dataset_config_from_str(self.dataset_name)
        observation_topics = args.o__topic_observation
        action_topics = args.a__topic_action
        step_topics = args.s__topic_step
        
        if args.orchstrator is None:
            from fogros2_rt_x.plugins.orchestrator_examples import PerTimeIntervalTopicOrchestrator
            orchestrator=PerTimeIntervalTopicOrchestrator(
                per_step_interval=0.1,  # seconds
                per_episode_interval=1.0, # seconds
            )
        else:
            raise ValueError("TODO: haven't implemented custom orchstrator yet")

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
