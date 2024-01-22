import logging
from .bag_manager import BagManager
from .backend_writer import CloudBackendWriter
from .conf_base import *
from .dataset_spec import feature_spec_list_to_default_value_dict
import os
from .database_connector import SqliteConnector


class DatasetManager:
    def __init__(
        self, dataset_directory, sql_db_location, dataset_name_in_sql="test_fogros"
    ):
        self.logger = logging.getLogger(__name__)
        self.dataset_directory = dataset_directory
        self.sql_backend = SqliteConnector(sql_db_location)
        self.bag_files = self.get_all_bag_files(self.dataset_directory)
        self.dataset_name_in_sql = dataset_name_in_sql

        self.metadata_dict = self.get_all_bag_metadata()

        self.create_table_from_metadata(self.metadata_dict[self.bag_files[0]])
        for bag_file in self.bag_files:
            self.insert_metadata_to_sql(self.metadata_dict[bag_file])

    def create_table_from_metadata(self, metadata):
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
                columns[key] = "BLOB"  # store as it is
        columns["should_export_as_rlds"] = "INTEGER"
        print(columns)
        self.sql_backend.create_table(
            table_name=self.dataset_name_in_sql, columns=columns
        )

    def insert_metadata_to_sql(self, metadata):
        # by default all the data should be exported as rlds
        metadata["should_export_as_rlds"] = 1
        # insert metadata to sql
        self.sql_backend.insert_data(table_name=self.dataset_name_in_sql, data=metadata)

    def get_metadata_from_bag_file(self, bag_file):
        bag_manager = BagManager(bag_file)
        # bag_manager.iterate_through_all_messages()
        return bag_manager.get_metadata()

    def get_all_bag_metadata(self):
        metadata_dict = {}
        for bag_file in self.bag_files:
            metadata = self.get_metadata_from_bag_file(bag_file)
            # print(metadata)
            metadata_dict[bag_file] = metadata
        return metadata_dict

    def get_all_bag_files(self, directory):
        bag_files = []
        # listing all the bag files
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.startswith("rosbag"):
                    bag_files.append(root)

        return bag_files

    def for_exporter(
        self, observation_topics, action_topics, step_topics, orchestrator
    ):
        self.bag_manager = BagManager(
            orchestrator,
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
            dataset_name="berkeley_fanuc_manipulation",
            save_path="/home/ubuntu/open-x-embodiment/playground_ds",  # TODO: take it down
            observation_spec=observation_spec,
            action_spec=action_spec,
            step_spec=step_spec,
        )

        self.writer = CloudBackendWriter(
            data_directory="/home/ubuntu/open-x-embodiment/playground_ds",
            max_episodes_per_file=1,
            ds_config=self.config.get_rlds_dataset_config(),
            logger=self.logger,
            metadata_database=None,
        )
        self.orchestrator.set_writer(self.writer)

        self.orchestrator.set_default_values(
            feature_spec_list_to_default_value_dict(observation_spec),
            feature_spec_list_to_default_value_dict(action_spec),
            feature_spec_list_to_default_value_dict(step_spec),
        )

        self.bag_manager.iterate_through_all_messages()
