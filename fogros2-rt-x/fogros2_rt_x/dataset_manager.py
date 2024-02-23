import logging
from .bag_manager import BagManager
from .backend_writer import CloudBackendWriter
from .conf_base import *
from .dataset_spec import feature_spec_list_to_default_value_dict
import os
from .database_connector import SqliteConnector


class DatasetManager:
    """
    A class for managing datasets.

    Args:
        sql_db_location (str): The location of the SQL database.
        dataset_name (str, optional): The name of the dataset. Defaults to "test_fogros".
    """

    def __init__(
        self, sql_db_location, dataset_name="test_fogros"
    ):
        self.logger = logging.getLogger(__name__)
        self.sql_backend = SqliteConnector(sql_db_location)
        self.dataset_name = dataset_name

    def load(self, dataset_directory):
        """
        Load the dataset from the given directory.

        Args:
            dataset_directory (str): The directory containing the dataset.
        """
        self.dataset_directory = dataset_directory
        self.bag_files = self.get_all_bag_files(self.dataset_directory)
        print(self.bag_files)
        self.metadata_dict = self.get_all_bag_metadata()

        self.create_table_from_metadata(self.metadata_dict[self.bag_files[0]])
        for bag_file in self.bag_files:
            self.insert_metadata_to_sql(self.metadata_dict[bag_file])

    def create_table_from_metadata(self, metadata):
        """
        Create a table in the SQL database based on the metadata.

        Args:
            metadata (dict): The metadata of the bag files.
        """
        # create table based on metadata
        # TODO: currently assume all the bag files have the same metadata
        columns = {}
        for key in metadata:
            if key.endswith("_num_msgs"):
                columns[key] = "INTEGER"
            if (
                key == "start_time"
                or key == "end_time"
                or key == "duration"
                or key == "message_count"
            ):
                columns[key] = "INTEGER"
            else:
                columns[key] = "TEXT"  # store as it is
        columns["should_export_as_rlds"] = "INTEGER"
        print(columns)
        self.sql_backend.create_table(
            table_name=self.dataset_name, columns=columns
        )

    def insert_metadata_to_sql(self, metadata):
        """
        Insert the metadata into the SQL database.

        Args:
            metadata (dict): The metadata of a bag file.
        """
        # by default all the data should be exported as rlds
        metadata["should_export_as_rlds"] = 1
        # insert metadata to sql
        self.sql_backend.insert_data(table_name=self.dataset_name, data=metadata)

    def get_metadata_from_bag_file(self, bag_file):
        """
        Get the metadata from a bag file.

        Args:
            bag_file (str): The path to the bag file.

        Returns:
            dict: The metadata of the bag file.
        """
        bag_manager = BagManager(bag_file)
        # bag_manager.iterate_through_all_messages()
        return bag_manager.get_metadata()

    def get_all_bag_metadata(self):
        """
        Get the metadata of all the bag files in the dataset.

        Returns:
            dict: A dictionary mapping bag file paths to their metadata.
        """
        metadata_dict = {}
        for bag_file in self.bag_files:
            metadata = self.get_metadata_from_bag_file(bag_file)
            # print(metadata)
            metadata_dict[bag_file] = metadata
        return metadata_dict

    def get_all_bag_files(self, directory):
        """
        Get all the bag files in the given directory.

        Args:
            directory (str): The directory to search for bag files.

        Returns:
            list: A list of bag file paths.
        """
        bag_files = []
        # listing all the bag files
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.startswith("rosbag"):
                    bag_files.append(root)
                elif file.endswith(".db3"):
                    bag_files.append(root)
                else:
                    print(f"Skipping {file} as it is not a bag file")

        return bag_files

    def export_as_rlds(
        self, observation_topics, action_topics, step_topics, orchestrator, destination
    ):
        """
        Export the dataset as RLDS format.

        Args:
            observation_topics (list): The topics to be considered as observations.
            action_topics (list): The topics to be considered as actions.
            step_topics (list): The topics to be considered as steps.
            orchestrator: The orchestrator for exporting the dataset.
            destination (str): The destination directory for the exported dataset.
        """
        # get all the data that should be exported as rlds
        bags = self.sql_backend.query_data_with_condition(
            table_name=self.dataset_name,
            columns=["bag_path"],
            conditions={"should_export_as_rlds": 1},
            use_or=True,
        )
        for bag in bags:
            bag_path = bag[0]
            self._export_bag_as_rlds(
                bag_path, observation_topics, action_topics, step_topics, orchestrator,
                destination
            )

    def _export_bag_as_rlds(
        self, bag_path, observation_topics, action_topics, step_topics, orchestrator,
        destination
    ):
        """
        Export a bag file as RLDS format.

        Args:
            bag_path (str): The path to the bag file.
            observation_topics (list): The topics to be considered as observations.
            action_topics (list): The topics to be considered as actions.
            step_topics (list): The topics to be considered as steps.
            orchestrator: The orchestrator for exporting the dataset.
            destination (str): The destination directory for the exported dataset.
        """
        self.bag_manager = BagManager(
            bag_path
        )
        (
            observation_spec,
            action_spec,
            step_spec,
        ) = self.bag_manager.generate_tensorflow_configuration_file(
            observation_topics,
            action_topics,
            step_topics,
        )

        self.config = BaseDatasetConfig(
            dataset_name=self.dataset_name,
            observation_spec=observation_spec,
            action_spec=action_spec,
            step_spec=step_spec,
        )

        self.writer = CloudBackendWriter(
            data_directory=destination,
            max_episodes_per_file=1,
            ds_config=self.config.get_rlds_dataset_config(),
            logger=self.logger,
            metadata_database=None,
        )
        orchestrator.set_writer(self.writer)

        orchestrator.set_default_values(
            feature_spec_list_to_default_value_dict(observation_spec),
            feature_spec_list_to_default_value_dict(action_spec),
            feature_spec_list_to_default_value_dict(step_spec),
        )

        self.bag_manager.iterate_through_all_messages(
            orchestrator,
            observation_topics,
            action_topics,
            step_topics,
        )
